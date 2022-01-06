/// Функции работы с SIM800L
use stm32f1xx_hal::prelude::*;
//
use core::fmt::Write;
use core::sync::atomic::Ordering;
use embedded_hal::digital::v2::OutputPin;
use heapless::Vec;
use nb::block;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::serial::Serial;

use crate::context::Context;
use crate::eeprom::EepromAdresses;
use crate::interrupts::SEC_COUNT_LED;

/// длина буфера телефонного номера SIM800
const SIM800_NUMBER_LEN: usize = 40;
/// длина буфера приёма данных от SIM800
const SIM800_RCV_BUF_LEN: usize = 1600;

#[derive(Debug, Clone, Copy, PartialEq)]
enum ComStates {
    /// не установлена связь с модулем
    Initial,
    /// связь с модулем есть, нет регистрации в сети
    Connected,
    /// модуль зарегистрирован в сети
    Registered,
}

pub struct Sim800<PINS> {
    state: ComStates,
    rcv_buf: Vec<u8, SIM800_RCV_BUF_LEN>,
    auth: Vec<Vec<u8, SIM800_NUMBER_LEN>, 3>,
    active_num: u8,
    sim800_port: Serial<pac::USART3, PINS>,
    sim_reset_pin: stm32f1xx_hal::gpio::gpioa::PA6<
        stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
    >,
}

impl<PINS> Sim800<PINS> {
    pub fn new(
        port: Serial<pac::USART3, PINS>,
        pin: stm32f1xx_hal::gpio::gpioa::PA6<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
        >,
    ) -> Self {
        Sim800 {
            state: ComStates::Initial,
            rcv_buf: Vec::new(),
            auth: Vec::new(), // allowed phone numbers
            active_num: 0,
            sim800_port: port,
            sim_reset_pin: pin,
        }
    }

    /// перезагрузка модуля при проблемах
    pub fn reboot(&mut self, ctx: &mut Context) {
        self.sim_reset_pin.set_low().unwrap();
        ctx.beep();
        ctx.delay.delay_ms(1_500_u16);
        self.sim_reset_pin.set_high().unwrap();
        ctx.watchdog.feed();
        writeln!(ctx.console, "REBOOT").unwrap();
    }

    /// проверка/инициализация связи с модулем
    pub fn check_com(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let mut r = self.send_at_cmd_wait_resp_n(ctx, b"AT\n", b"OK\r", 50, 10, 20);
        if r.is_ok() {
            if self.state != ComStates::Registered {
                if self.state == ComStates::Initial {
                    r = self.init_set_0(ctx);
                }
                self.state = ComStates::Connected;
            }
        } else {
            self.state = ComStates::Initial;
        }
        r
    }

    /// проверка регистрации в сети
    pub fn check_reg(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let mut reply = Ok(());
        let ans1 = b"+CREG: 0,1";
        let ans2 = b"+CREG: 0,5";
        // 10 attempts
        for _ in 1..=10 {
            let _ = self.send_at_cmd_wait_resp(ctx, b"AT+CREG?\n", 100, 20);
            if self.buf_contains(ans1).is_ok() || self.buf_contains(ans2).is_ok() {
                if self.state != ComStates::Registered {
                    reply = self.init_set_1(ctx);
                    if reply.is_ok() {
                        reply = self.init_auth(ctx);
                        //    for nbr in (&self.auth).into_iter() {
                        //        writeln!(ctx.console, "Num={}", core::str::from_utf8(&nbr).unwrap() );
                        //    }
                    }
                    self.state = ComStates::Registered;
                } else {
                    reply = Ok(());
                }
                break;
            } else {
                self.state = ComStates::Connected;
            }
            ctx.watchdog.feed();
            ctx.delay.delay_ms(5000_u16); // delay between attempts
            ctx.watchdog.feed();
        }
        reply
    }

    /// отправка простого SMS на дефолтный номер
    pub fn send_sms(&mut self, ctx: &mut Context, msg: &[u8]) -> Result<(), ()> {
        let mut reply = Err(());
        let mut buf: Vec<u8, 160> = Vec::new();
        writeln!(
            buf,
            r#"AT+CMGS="{}""#,
            core::str::from_utf8(self.get_active_num()).unwrap()
        )
        .unwrap();
        let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10);
        if self.buf_contains(b">").is_ok() {
            buf.clear();
            write!(buf, "{}\u{001a}", core::str::from_utf8(msg).unwrap()).unwrap();
            writeln!(
                ctx.console,
                "sms msg = {}",
                core::str::from_utf8(&buf).unwrap()
            )
            .unwrap();
            let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 700, 100);
            if self.buf_contains(b"+CMGS").is_ok() {
                reply = Ok(());
            }
        } else {
            reply = Err(());
        }
        ctx.delay.delay_ms(1000_u16); // чуть ждём после посылки
        reply
    }

    /// Определение входящего авторизованного звонка
    pub fn call_detect(&mut self, ctx: &mut Context) -> Result<(), ()> {
        // enable incoming calls and unsolicited RINGs
        if self.gsm_busy(ctx, false).is_err() {
            return Err(());
        }
        let mut reply = Err(());

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
                    ctx.led.set_high().unwrap();
                    led_state = false;
                } else {
                    ctx.led.set_low().unwrap();
                    led_state = true;
                }
                SEC_COUNT_LED.store(0, Ordering::Relaxed); // сбрасываем таймер
            }

            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    writeln!(ctx.console, "RING e1").unwrap();
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "RING e2").unwrap();
                            break;
                        }
                        Err(nb::Error::WouldBlock) => {
                            // просто ждём ещё таймер
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
        if self.rcv_buf.is_empty() {
            // ничего не смогли получить
            writeln!(ctx.console, "NO RING").unwrap();
            let _ = self.gsm_busy(ctx, true); // block GSM RINGs
            return Err(());
        }

        // check RING number in buf
        let mut number: Vec<u8, SIM800_NUMBER_LEN> = Vec::new();
        if self.buf_contains(b"+CLIP:").is_err() {
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
                    number.push(*sub).unwrap();
                }
            }
            // звонок с номера
            writeln!(
                ctx.console,
                "RING: {}",
                core::str::from_utf8(&number).unwrap()
            )
            .unwrap();
        }

        // check number is auth
        for (i, nbr) in (&self.auth).into_iter().enumerate() {
            if nbr == &number {
                //println!("auth num = {}", core::str::from_utf8(&number).unwrap());
                reply = Ok(());
                let number_i = i as u8 + 1;
                if self.active_num != number_i {
                    self.active_num = number_i;
                    let _ = ctx.save_byte(EepromAdresses::Number.into(), self.active_num);
                }
                break;
            }
        }

        // hang up call
        if self
            .send_at_cmd_wait_resp_n(ctx, b"ATH\n", b"OK\r", 50, 10, 3)
            .is_err()
        {
            return Err(());
        }
        // block unsolicited RINGs
        if self.gsm_busy(ctx, true).is_err() {
            return Err(());
        }
        reply
    }

    /// номер, на который будут отправляться СМС
    fn get_active_num(&mut self) -> &[u8] {
        &self.auth[(self.active_num - 1) as usize][..]
    }

    /// поиск подстроки в буфере приёма
    fn buf_contains(&mut self, pattern: &[u8]) -> Result<(), ()> {
        let psize = pattern.len();
        let bsize = self.rcv_buf.len();
        for i in 0..bsize {
            let rlimit = i + psize;
            if rlimit > bsize {
                break;
            }
            let sl = &self.rcv_buf[i..rlimit];
            if sl == pattern {
                return Ok(());
            }
        }
        Err(())
    }

    /// отправка АТ команды и получение ответа от SIM800
    fn send_at_cmd_wait_resp(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8], // команда
        toc: u16,      // timeout for first char
        to: u16,
    ) -> Result<(), ()> {
        // читаем мусор из порта
        loop {
            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    writeln!(ctx.console, "SIM e0").unwrap();
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
                    writeln!(ctx.console, "SIM e1").unwrap();
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "SIM e2").unwrap();
                            break;
                        }
                        Err(nb::Error::WouldBlock) => {
                            // просто ждём ещё таймер
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
        if self.rcv_buf.is_empty() {
            // ничего не смогли получить
            writeln!(ctx.console, "SIM nrsp").unwrap();
            return Err(());
        }
        Ok(())
    }

    /// отправка команды за несколько попыток
    fn send_at_cmd_wait_resp_n(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8],
        ans: &[u8],
        toc: u16, // timeout for first char
        to: u16,  // timeout after last char
        tries: u8,
    ) -> Result<(), ()> {
        // no of attempts
        // checks if reply from SIM800L contains ans using tries attempts
        for _ in 1..=tries {
            let _ = self.send_at_cmd_wait_resp(ctx, at_cmd, toc, to);
            if self.buf_contains(ans).is_ok() {
                return Ok(());
            }
            ctx.delay.delay_ms(500_u16); // delay between attempts
            ctx.watchdog.feed();
        }
        Err(())
    }

    /// блокировка приёма звонков
    fn gsm_busy(&mut self, ctx: &mut Context, set_to: bool) -> Result<(), ()> {
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

    /// команды инициализации до регистрации в сети
    fn init_set_0(&mut self, ctx: &mut Context) -> Result<(), ()> {
        if self.state == ComStates::Initial {
            return Err(());
        }
        // Reset to the factory settings
        let r1 = self.send_at_cmd_wait_resp_n(ctx, b"AT&F\n", b"OK\r", 100, 10, 3);
        // switch off echo
        let r2 = self.send_at_cmd_wait_resp_n(ctx, b"ATE0\n", b"OK\r", 50, 10, 3);
        // setup fixed baud rate 9600
        let r3 = self.send_at_cmd_wait_resp_n(ctx, b"AT+IPR=9600\n", b"OK\r", 100, 10, 3);
        if r1.is_ok() && r2.is_ok() && r3.is_ok() {
            Ok(())
        } else {
            Err(())
        }
    }

    /// executed after module registration in network
    fn init_set_1(&mut self, ctx: &mut Context) -> Result<(), ()> {
        if self.state == ComStates::Initial {
            return Err(());
        }
        // block unsolicited RINGs
        let r0 = self.gsm_busy(ctx, true);
        // Request calling line identification
        let r1 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CLIP=1\n", b"OK\r", 50, 10, 3);
        // Mobile Equipment Error Code
        let r2 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CMEE=0\n", b"OK\r", 50, 10, 3);
        // set the SMS mode to text
        let r3 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CMGF=1\n", b"OK\r", 50, 10, 3);
        // Disable messages about new SMS from the GSM module
        let r4 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CNMI=2,0\n", b"OK\r", 100, 10, 3);
        // send AT command to init memory for SMS in the SIM card
        // response:
        // +CPMS: <usedr>,<totalr>,<usedw>,<totalw>,<useds>,<totals>
        let r5 = self.send_at_cmd_wait_resp_n(
            ctx,
            b"AT+CPMS=\"SM\",\"SM\",\"SM\"\n",
            b"+CPMS:",
            100,
            100,
            3,
        );
        // select phonebook memory storage
        let r6 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CPBS=\"SM\"\n", b"OK\r", 100, 10, 3);
        // Deactivate GPRS PDP context
        let r7 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CIPSHUT\n", b"SHUT OK", 100, 10, 3);
        if r0.is_ok()
            && r1.is_ok()
            && r2.is_ok()
            && r3.is_ok()
            && r4.is_ok()
            && r5.is_ok()
            && r6.is_ok()
            && r7.is_ok()
        {
            Ok(())
        } else {
            Err(())
        }
    }

    /// Считываение первых 3 номеров с SIM карты. Они будут использоваться для авторизации звонков
    fn init_auth(&mut self, ctx: &mut Context) -> Result<(), ()> {
        self.active_num = ctx.eeprom.read_byte(EepromAdresses::Number.into()).unwrap();
        if !(self.active_num > 0 && self.active_num < 4) {
            // неправильный номер в EEPROM
            self.active_num = 3; // дефолтное значение в 3 ячейке
            ctx.eeprom
                .write_byte(EepromAdresses::Number.into(), self.active_num)
                .unwrap();
            ctx.delay.delay_ms(10_u16);
        }

        self.auth.clear();
        for j in 1..=3 {
            // take first 3 numbers from SIM phonebook
            let mut buf: Vec<u8, 32> = Vec::new();
            write!(buf, "AT+CPBR={}\n", j).unwrap();
            for _ in 1..=3 {
                // make 3 attempts
                let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10);
                if self.buf_contains(b"OK\r").is_ok() {
                    if self.buf_contains(b"+CPBR:").is_err() {
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
                    return Ok(());
                }
                ctx.delay.delay_ms(500_u16); // delay between attempts
                ctx.watchdog.feed();
            }
        }
        Err(())
    }
}
