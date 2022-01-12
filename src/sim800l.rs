use super::*;
use crate::context::Context;
use crate::eeprom::EepromAdresses;
use core::marker::PhantomData;

///--------------------------------------------------------------
/// Функции работы с SIM800L
///--------------------------------------------------------------

const SIM800_NUMBER_LEN: usize = 40; // длина буфера телефонного номера SIM800
const SIM800_RCV_BUF_LEN: usize = 1600; // длина буфера приёма данных от SIM800
const DEFAULT_NUM: u8 = 3; // номер ячейки номеров SIM карты с дефолтным номером для SMS
const SMS_COMMAND_LEN: usize = 16; // макисмальная длина команды в пришедем SMS
const SMS_BALANCE_LEN: usize = 20; // макисмальная длина инфо о балансе в пришедем SMS

// Тип отправляемого СМС
pub enum SmsType {
    Normal,
    Balance, // Запрос баланса сим карты у оператора МТС
}

// Объекты возможных состояний машины
struct Initial800;
struct Connected800;
struct Registered800;

// конкретное состояние
struct SimState<S> {
    state: PhantomData<S>,
}

// конкретные типы состояний
type SIMI = SimState<Initial800>;
type SIMC = SimState<Connected800>;
type SIMR = SimState<Registered800>;

// допустимые методы перехода между состояниями
impl From<&SIMI> for SIMC {
    fn from(_val: &SIMI) -> SIMC {
        SimState { state: PhantomData }
    }
}
impl From<&SIMR> for SIMC {
    fn from(_val: &SIMR) -> SIMC {
        SimState { state: PhantomData }
    }
}
impl From<&SIMC> for SIMR {
    fn from(_val: &SIMC) -> SIMR {
        SimState { state: PhantomData }
    }
}

// обертка допустимых состояний для работы match
enum SimStateWrapper {
    Initial800(SIMI),    // не установлена связь с модулем
    Connected800(SIMC),  // связь с модулем есть, нет регистрации в сети
    Registered800(SIMR), // модуль зарегистрирован в сети
}

impl Default for SimStateWrapper {
    fn default() -> Self {
        SimStateWrapper::Initial800(SIMI { state: PhantomData })
    }
}

pub struct Sim800<PINS> {
    state_machine: SimStateWrapper,
    rcv_buf: Vec<u8, SIM800_RCV_BUF_LEN>,
    auth: Vec<Vec<u8, SIM800_NUMBER_LEN>, 3>, // вектор для 3 телефонных номеров
    active_num: u8, // порядковый номер в массиве auth активного телефонного номера
    sim800_port: Sim800PortType<PINS>,
    sim_reset_pin: Sim800PinType,
}

impl<PINS> Sim800<PINS> {
    pub fn new(port: Sim800PortType<PINS>, pin: Sim800PinType) -> Self {
        Sim800 {
            state_machine: Default::default(),
            rcv_buf: Vec::new(),
            auth: Vec::new(), // allowed phone numbers
            active_num: 0,
            sim800_port: port,
            sim_reset_pin: pin,
        }
    }

    // номер, на который будут отправляться СМС
    pub fn get_active_num(&mut self) -> &[u8] {
        &self.auth[(self.active_num - 1) as usize][..]
    }

    // отправка АТ команды и получение ответа от SIM800
    fn send_at_cmd_wait_resp(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8], // команда
        toc: u16,      // timeout for first char
        to: u16,
    ) -> Result<(), Error> {
        // читаем мусор из порта и игнорим все ошибки
        loop {
            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"SIM e0");
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    // к счастью ничего нет
                    break;
                }
                Ok(_) => {} // если что, перезагрузится по сторожевому таймеру
            }
        }
        // отправляем команду
        for cmd in at_cmd {
            block!(self.sim800_port.write(*cmd)).ok();
        }
        // пробуем получить ответ
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        loop {
            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"SIM e1");
                    return Err(Error::SerialError);
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            write_log(b"SIM e2");
                            return Err(Error::TimerError);
                        }
                        Err(nb::Error::WouldBlock) => {
                            // просто ждём ещё таймер
                            ctx.watchdog.feed();
                            continue;
                        }
                        Ok(_) => {
                            // сработал таймер отсчёта таймаута
                            if got_first_char {
                                // отрабатываем ожидание последнего символа
                                if w2_cycles >= to {
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1;
                                    continue;
                                }
                            } else {
                                // отрабатываем ожидание первого символа
                                if w1_cycles >= toc {
                                    break;
                                } else {
                                    w1_cycles += 1;
                                    continue;
                                }
                            }
                        }
                    }
                }
                Ok(x) => {
                    // получили очередной символ от SIM800
                    if self.rcv_buf.len() < SIM800_RCV_BUF_LEN {
                        // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                    } else {
                        break;
                    }
                    got_first_char = true; // после первого символа переходим на ожидание последнего
                    w2_cycles = 0;
                    ctx.at_timer.reset(); // timeout timer restart after each byte recieved
                    continue;
                }
            }
        }
        if self.rcv_buf.len() == 0 {
            // ничего не смогли получить
            write_log(b"SIM nrsp");
            return Err(Error::SerialNoData);
        }
        Ok(())
    }

    // отправка команды за несколько попыток
    fn send_at_cmd_wait_resp_n(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8],
        ans: &[u8],
        toc: u16, // timeout for first char
        to: u16,  // timeout after last char
        tries: u8,
    ) -> Result<(), Error> {
        // no of attempts
        // checks if reply from SIM800L contains ans using tries attempts
        let mut reply: bool = false;
        for _ in 0..tries {
            match self.send_at_cmd_wait_resp(ctx, at_cmd, toc, to) {
                Ok(_) => {}
                Err(Error::SerialNoData) => continue,
                Err(val) => return Err(val),
            };
            if buf_contains(&self.rcv_buf, ans) {
                reply = true;
                break;
            }
            ctx.delay.delay_ms(500_u16); // delay between attempts
            ctx.watchdog.feed();
        }
        if reply {
            return Ok(());
        }
        Err(Error::CmdFail)
    }

    // блокировка приёма звонков
    fn gsm_busy(&mut self, ctx: &mut Context, set_to: bool) -> Result<(), Error> {
        self.send_at_cmd_wait_resp_n(
            ctx,
            if set_to {
                b"AT+GSMBUSY=1\n"
            } else {
                b"AT+GSMBUSY=0\n"
            },
            b"OK\r",
            100,
            10,
            3,
        )
    }

    // команды инициализации до регистрации в сети
    fn init_set_0(&mut self, ctx: &mut Context) -> Result<(), Error> {
        match &self.state_machine {
            SimStateWrapper::Initial800(_) => return Ok(()),
            _ => {}
        }
        // Reset to the factory settings
        self.send_at_cmd_wait_resp_n(ctx, b"AT&F\n", b"OK\r", 100, 10, 3)?;
        // switch off echo
        self.send_at_cmd_wait_resp_n(ctx, b"ATE0\n", b"OK\r", 50, 10, 3)?;
        // setup fixed baud rate 9600
        self.send_at_cmd_wait_resp_n(ctx, b"AT+IPR=9600\n", b"OK\r", 100, 10, 3)
    }

    // executed after module registration in network
    fn init_set_1(&mut self, ctx: &mut Context) -> Result<(), Error> {
        match &self.state_machine {
            SimStateWrapper::Initial800(_) => return Ok(()),
            _ => {}
        }
        // block unsolicited RINGs
        self.gsm_busy(ctx, true)?;
        // Request calling line identification
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CLIP=1\n", b"OK\r", 50, 10, 3)?;
        // Mobile Equipment Error Code
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CMEE=0\n", b"OK\r", 50, 10, 3)?;
        // set the SMS mode to text
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CMGF=1\n", b"OK\r", 50, 10, 3)?;
        // Disable messages about new SMS from the GSM module
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CNMI=2,0\n", b"OK\r", 100, 10, 3)?;
        // send AT command to init memory for SMS in the SIM card
        // response:
        // +CPMS: <usedr>,<totalr>,<usedw>,<totalw>,<useds>,<totals>
        self.send_at_cmd_wait_resp_n(
            ctx,
            b"AT+CPMS=\"SM\",\"SM\",\"SM\"\n",
            b"+CPMS:",
            100,
            100,
            3,
        )?;
        // select phonebook memory storage
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CPBS=\"SM\"\n", b"OK\r", 100, 10, 3)?;
        // Deactivate GPRS PDP context
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CIPSHUT\n", b"SHUT OK", 100, 10, 3)
    }

    // проверка/инициализация связи с модулем
    pub fn check_com(&mut self, ctx: &mut Context) -> bool {
        if self
            .send_at_cmd_wait_resp_n(ctx, b"AT\n", b"OK\r", 50, 10, 20)
            .is_ok()
        {
            match &self.state_machine {
                SimStateWrapper::Initial800(val) => {
                    self.state_machine = SimStateWrapper::Connected800(val.into());
                    self.init_set_0(ctx).ok();
                    write_log(b"SIM800 Connected!");
                }
                _ => {}
            }
            return true;
        } else {
            self.state_machine = Default::default();
            return false;
        }
    }

    // перезагрузка модуля при проблемах
    pub fn reboot(&mut self, ctx: &mut Context) {
        self.sim_reset_pin.set_low().ok();
        ctx.beep();
        ctx.delay.delay_ms(1_500_u16);
        self.sim_reset_pin.set_high().ok();
        ctx.watchdog.feed();
    }

    // проверка регистрации в сети
    pub fn check_reg(&mut self, ctx: &mut Context) -> bool {
        let mut reply: bool = false;
        let ans1 = b"+CREG: 0,1";
        let ans2 = b"+CREG: 0,5";
        for _ in 0..10 {
            // 10 attempts
            if self
                .send_at_cmd_wait_resp(ctx, b"AT+CREG?\n", 100, 20)
                .is_err()
            {
                break; // пропала связь с модулем?
            }
            if buf_contains(&self.rcv_buf, ans1) || buf_contains(&self.rcv_buf, ans2) {
                reply = true;
            } else {
                ctx.watchdog.feed();
                ctx.delay.delay_ms(5000_u16); // delay between attempts
                ctx.watchdog.feed();
                write_log(b"Wait for reg...");
                continue; // пробуем еще подождать регистрации
            }
            match &self.state_machine {
                SimStateWrapper::Connected800(val) => {
                    if reply {
                        self.state_machine = SimStateWrapper::Registered800(val.into());
                        write_log(b"Registered!");
                        reply = self.init_set_1(ctx).is_ok();
                        if reply {
                            reply = self.init_auth(ctx).is_ok();
                        }
                    }
                }
                _ => {}
            }
            if reply {
                // прекращаем попытки
                break;
            }
        }
        reply
    }

    // Считываение первых 3 номеров с SIM карты. Они будут использоваться для авторизации звонков
    fn init_auth(&mut self, ctx: &mut Context) -> Result<(), Error> {
        self.active_num = ctx.eeprom.read_byte(EepromAdresses::Number as u32).unwrap();
        if !(self.active_num > 0 && self.active_num < 4) {
            // неправильный номер в EEPROM
            self.active_num = DEFAULT_NUM; // дефолтное значение
            ctx.eeprom
                .write_byte(EepromAdresses::Number as u32, self.active_num)
                .unwrap();
            ctx.delay.delay_ms(10_u16);
        }

        self.auth.clear();
        let mut reply: bool = false;
        for j in 1..=3 {
            // take first 3 numbers from SIM phonebook
            let mut buf: Vec<u8, 32> = Vec::new();
            write!(buf, "AT+CPBR={}\n", j).unwrap();
            for _ in 0..3 {
                // make 3 attempts
                match self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10) {
                    Ok(()) => {}
                    Err(Error::SerialNoData) => continue,
                    Err(val) => return Err(val),
                }
                if buf_contains(&self.rcv_buf, b"OK\r") {
                    if !buf_contains(&self.rcv_buf, b"+CPBR:") {
                        self.auth
                            .push(Vec::<u8, SIM800_NUMBER_LEN>::from_slice(b"NA").unwrap())
                            .unwrap();
                    } else {
                        // parse for phone number
                        let mut found = false;
                        let mut number: Vec<u8, SIM800_NUMBER_LEN> = Vec::new();
                        for sub in &self.rcv_buf {
                            if sub == &b'"' {
                                if found {
                                    break;
                                }
                                found = true;
                                continue;
                            }
                            if found {
                                //copy chars from SIM800 reply
                                number.push(*sub).unwrap();
                            }
                        }
                        self.auth.push(number).unwrap();
                    }
                    reply = true;
                    break;
                }
                ctx.delay.delay_ms(500_u16); // delay between attempts
                ctx.watchdog.feed();
            }
        }
        if reply {
            return Ok(());
        }
        Err(Error::NoAuthNumbers)
    }

    // отправка простого SMS на дефолтный номер
    pub fn send_sms(&mut self, ctx: &mut Context, msg: &[u8], typ: SmsType) -> Result<(), Error> {
        let mut buf: Vec<u8, 160> = Vec::new();
        match typ {
            Normal => {
                writeln!(
                    buf,
                    r#"AT+CMGS="{}""#,
                    core::str::from_utf8(self.get_active_num())?
                )?;
            }
            // для запроса баланса отправляем СМС на номер МТС 111
            Balance => {
                writeln!(buf, r#"AT+CMGS="111""#)?;
            }
        }
        self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10)?;
        if buf_contains(&self.rcv_buf, b">") {
            buf.clear();
            write!(buf, "{}\u{001a}", core::str::from_utf8(&msg)?)?;
            write_log(b"SMS=");
            write_log(buf.as_slice());
            self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 700, 100)?;
            if buf_contains(&self.rcv_buf, b"+CMGS") {
                ctx.delay.delay_ms(1000_u16); // чуть ждём после посылки
                return Ok(());
            }
            return Err(Error::SmsStage2);
        } else {
            return Err(Error::SmsStage1);
        }
    }

    // check number is auth
    fn is_auth(&mut self, ctx: &mut Context, number: &[u8]) -> bool {
        let mut reply = false;
        for (i, nbr) in (&self.auth).into_iter().enumerate() {
            if nbr == &number {
                //println!("auth num = {}", core::str::from_utf8(&number).unwrap());
                reply = true;
                let number_i = i as u8 + 1;
                if self.active_num != number_i {
                    self.active_num = number_i;
                    let address = u32::from(EepromAdresses::Number as u8);
                    ctx.save_byte(address, self.active_num).ok();
                }
                break;
            }
        }
        reply
    }

    // Определение входящего авторизованного звонка
    pub fn call_detect(&mut self, ctx: &mut Context) -> Result<(), Error> {
        let mut reply = false;
        // enable incoming calls and unsolicited RINGs
        self.gsm_busy(ctx, false)?;

        // пробуем получить RING
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        let mut led_state = false;
        loop {
            ctx.watchdog.feed();

            // мигаем периодически светодиодом - heartbeat
            let cnt3 = SEC_COUNT_LED.load(Ordering::Relaxed);
            if cnt3 > 1 {
                if led_state {
                    ctx.led.set_high().ok();
                    led_state = false;
                } else {
                    ctx.led.set_low().ok();
                    led_state = true;
                }
                SEC_COUNT_LED.store(0, Ordering::Relaxed); // сбрасываем таймер
            }

            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"RING e1");
                    return Err(Error::SerialError);
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            write_log(b"RING e2");
                            return Err(Error::TimerError);
                        }
                        Err(nb::Error::WouldBlock) => {
                            // просто ждём ещё таймер
                            ctx.watchdog.feed();
                            continue;
                        }
                        Ok(_) => {
                            // сработал таймер отсчёта таймаута
                            if got_first_char {
                                // отрабатываем ожидание последнего символа
                                if w2_cycles >= 200 {
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1;
                                    continue;
                                }
                            } else {
                                // отрабатываем ожидание первого символа 10 сек
                                if w1_cycles >= 1000 {
                                    break;
                                } else {
                                    w1_cycles += 1;
                                    continue;
                                }
                            }
                        }
                    }
                }
                Ok(x) => {
                    // получили очередной символ от SIM800
                    if self.rcv_buf.len() < SIM800_RCV_BUF_LEN {
                        // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                    } else {
                        break;
                    }
                    got_first_char = true; // после первого символа переходим на ожидание последнего
                    w2_cycles = 0;
                    ctx.at_timer.reset(); // timeout timer restart after each byte recieved
                    continue;
                }
            }
        }
        if self.rcv_buf.len() == 0 {
            // ничего не смогли получить
            write_log(b"NO RING");
            self.gsm_busy(ctx, true)?; // block GSM RINGs
            return Err(Error::NoRing);
        }

        // check RING number in buf
        let mut number: Vec<u8, SIM800_NUMBER_LEN> = Vec::new();
        if !buf_contains(&self.rcv_buf, b"+CLIP:") {
            number.push(b'N').unwrap();
        } else {
            // ищем номер вызывающего телефона
            let mut found = false;
            for sub in &self.rcv_buf {
                if sub == &b'"' {
                    if found {
                        break;
                    }
                    found = true;
                    continue;
                }
                if found {
                    //copy chars from SIM800 reply
                    if number.len() < SIM800_NUMBER_LEN {
                        number.push(*sub).unwrap();
                    } else {
                        break;
                    }
                }
            }
            // звонок с номера
            write_log(b"RING: ");
            write_log(&number);
        }

        // check number is auth
        if self.is_auth(ctx, &number) {
            reply = true
        }

        // hang up call
        self.send_at_cmd_wait_resp_n(ctx, b"ATH\n", b"OK\r", 50, 10, 5)?;
        // block unsolicited RINGs
        self.gsm_busy(ctx, true)?;
        if reply {
            return Ok(());
        }
        Err(Error::NotAuthCall)
    }

    // очистка памяти SMS сообщений
    fn sms_del(&mut self, ctx: &mut Context) -> Result<(), Error> {
        self.send_at_cmd_wait_resp_n(ctx, b"AT+CMGDA=\"DEL ALL\"\n", b"OK\r", 150, 10, 3)
    }

    pub fn sms_detect(&mut self, ctx: &mut Context) -> Result<Vec<u8, SMS_COMMAND_LEN>, Error> {
        /*self.send_at_cmd_wait_resp(ctx, b"AT+CMGL=\"ALL\"\n", 500, 150).unwrap();
        writeln!(ctx.console, "sms list = {}",
            core::str::from_utf8(self.rcv_buf.as_slice()).unwrap()).ok();*/
        self.send_at_cmd_wait_resp(ctx, b"AT+CMGR=1\n", 500, 150)
            .unwrap();
        // если есть в 1 ячейке SMS, должно прийти что-то типа
        // +CMGR: "REC READ","+79850000000","","21/10/29,16:30:00+12"\r\nTest\r\n\r\nOK\r\n
        // или ERORR, если в этой ячекйке ничего нет
        write_log(b"SMS detected = ");
        write_log(self.rcv_buf.as_slice());
        if buf_contains(&self.rcv_buf, b"\r\n+CMGR:") && buf_contains(&self.rcv_buf, b"\r\nOK\r\n")
        {
            // ищем номер вызывающего телефона
            let mut number: Vec<u8, SIM800_NUMBER_LEN> = Vec::new();
            let mut found_number = false;
            let mut subcount = 1; // ищем с третьих кавычек до 4 кавычек
            for sub in &self.rcv_buf {
                if sub == &b'"' {
                    if found_number {
                        break;
                    }
                    if subcount != 3 {
                        subcount += 1
                    } else {
                        found_number = true
                    }
                    continue;
                }
                if found_number {
                    // копируем символы номера в буфер
                    if number.len() < SIM800_NUMBER_LEN {
                        number.push(*sub).unwrap();
                    } else {
                        break;
                    }
                }
            }

            // проверяем авторизацию телефона
            if !self.is_auth(ctx, &number) {
                self.sms_del(ctx)?;
                return Err(Error::NotAuthCall);
            }

            // ищем в теле сообщения текст команды
            let mut found = false;
            let mut subcount = 1;
            let mut msgpos = 0; // позиция начала тела сообщения
            for (i, sub) in (&self.rcv_buf).into_iter().enumerate() {
                if sub == &b'"' {
                    if subcount != 8 {
                        // ищем с 8ых кавычек
                        subcount += 1;
                        continue;
                    } else {
                        found = true;
                        continue;
                    }
                }
                if found {
                    msgpos = i;
                    break;
                }
            }
            if self.rcv_buf.len() > 2 {
                // ограничиваем размер команды SMS_COMMAND_LEN символами
                let mut body_cmd: Vec<u8, SMS_COMMAND_LEN> = Vec::new();
                let body_text_src = &self.rcv_buf[(msgpos + 2)..]; // вырезаем первые CRLF
                let mut slice_len = body_text_src.len();
                if slice_len > SMS_COMMAND_LEN {
                    slice_len = SMS_COMMAND_LEN
                }
                body_cmd
                    .extend_from_slice(&body_text_src[..slice_len - 1])
                    .unwrap();
                self.sms_del(ctx)?;
                return Ok(body_cmd);
            }
        }
        Err(Error::NoInSMS)
    }

    pub fn balance_sms_detect(
        &mut self,
        ctx: &mut Context,
    ) -> Result<Vec<u8, SMS_BALANCE_LEN>, Error> {
        fn hex2u16(c: u8) -> u16 {
            // from 0 to 9
            if (c >= 0x30) && (c <= 0x39) {
                return (c - 0x30) as u16;
            // from A to F
            } else if (c >= 0x41) && (c <= 0x46) {
                return (c - 0x41 + 10) as u16;
            } else {
                return 0;
            }
        }

        self.send_at_cmd_wait_resp(ctx, b"AT+CMGR=1\n", 500, 150)
            .unwrap();
        // если есть в 1 ячейке SMS, должно прийти что-то типа
        // +CMGR: "REC UNREAD","111","","22/01/10,20:50:06+12"\r\n04110430043B0430043D0441\r\n\r\nOK\r\n
        // или ERORR, если в этой ячекйке ничего нет
        // номер отправителя не проверяется!
        write_log(b"SMS detected = ");
        write_log(self.rcv_buf.as_slice());
        if buf_contains(&self.rcv_buf, b"\r\n+CMGR:") && buf_contains(&self.rcv_buf, b"\r\nOK\r\n")
        {
            let mut ucs2buf: Vec<u8, SMS_BALANCE_LEN> = Vec::new();
            let mut ucs2_start: usize = 0;
            let mut ucs2_stop: usize = 0;
            let mut cmgr_found = false;
            ucs2buf.clear();
            for (index, chr) in (&self.rcv_buf).iter().enumerate() {
                // ищем + как признак начала CMGR
                if *chr == b'+' {
                    cmgr_found = true;
                    continue;
                }
                // ищем начало текста смс
                if cmgr_found && ucs2_start == 0 && *chr == b'\n' {
                    ucs2_start = index + 1;
                    continue;
                }
                // ищем конец текста смс
                if ucs2_start > 0 && *chr == b'\r' {
                    ucs2_stop = index;
                    break;
                }
            }
            if !(ucs2_start > 0 && ucs2_stop > 0) {
                return Err(Error::NoUCS2);
            }
            let ucs2str = &self.rcv_buf[ucs2_start..ucs2_stop];
            let ucs2len = ucs2str.len();
            if ucs2len % 4 != 0 {
                return Err(Error::InvalidUCS2Size);
            }

            let mut idx = 0;
            loop {
                // восстанавливаем юникодный код символа из UCS2
                let code = (hex2u16(ucs2str[idx]) << 12)
                    + (hex2u16(ucs2str[idx + 1]) << 8)
                    + (hex2u16(ucs2str[idx + 2]) << 4)
                    + hex2u16(ucs2str[idx + 3]);
                // берем только значимые для баланса счёта ASCII символы
                match code {
                    0x30..=0x39 | 0x2c..=0x2e => {
                        if ucs2buf.len() < SMS_BALANCE_LEN {
                            ucs2buf.push(code as u8).unwrap();
                        } else {
                            break;
                        }
                    }
                    _ => {}
                }
                // переходим к следующему символу, если он есть
                idx += 4;
                if idx >= ucs2len {
                    break;
                }
            }
            self.sms_del(ctx)?;
            return Ok(ucs2buf);
        }
        Err(Error::NoInSMS)
    }
}
