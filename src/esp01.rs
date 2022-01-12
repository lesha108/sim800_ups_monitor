use super::*;
use core::marker::PhantomData;
use crate::context::Context;

//--------------------------------------------------------------
// Функции работы с ESP01
//--------------------------------------------------------------
const ESP_BUF_LEN: usize = 50; // длина буфера приёма статуса ESP

// Объекты возможных состояний машины
#[derive(PartialEq, Eq)]
struct InitialEsp;
#[derive(PartialEq, Eq)]
struct ConnectedEsp;

// конкретное состояние
#[derive(PartialEq, Eq)]
struct EspState<S> {
    state: PhantomData<S>,
}

// конкретные типы состояний
type ESPI = EspState<InitialEsp>;
type ESPC = EspState<ConnectedEsp>;

// допустимые методы перехода между состояниями
impl From<&ESPI> for ESPC {
    fn from(_val: &ESPI) -> ESPC {
        EspState { state: PhantomData }
    }
}
impl From<&ESPC> for ESPI {
    fn from(_val: &ESPC) -> ESPI {
        EspState { state: PhantomData }
    }
}

// обертка допустимых состояний для работы match
#[derive(PartialEq, Eq)]
enum EspStateWrapper {
    InitialEsp(ESPI),   // не установлена связь с локальным модулем ESP
    ConnectedEsp(ESPC), // связь с модулем есть
}

impl Default for EspStateWrapper {
    fn default() -> Self {
        EspStateWrapper::InitialEsp(ESPI { state: PhantomData })
    }
}

pub struct Esp<PINS> {
    state_machine: EspStateWrapper,
    rcv_buf: Vec<u8, ESP_BUF_LEN>,
    esp_port: Esp01PortType<PINS>,
    esp_reset_pin: Esp01PinType,
}

impl<PINS> Esp<PINS> {
    pub fn new(port: Esp01PortType<PINS>, pin: Esp01PinType) -> Self {
        Esp {
            state_machine: Default::default(),
            rcv_buf: Vec::new(),
            esp_port: port,
            esp_reset_pin: pin,
        }
    }

    // перезагрузка модуля при проблемах
    pub fn reboot(&mut self, ctx: &mut Context) {
        self.esp_reset_pin.set_low().ok();
        ctx.beep();
        ctx.delay.delay_ms(1_500_u16);
        self.esp_reset_pin.set_high().ok();
        ctx.watchdog.feed();
    }

    // проверка/инициализация связи с модулем ESP
    pub fn check_com(&mut self, ctx: &mut Context) -> bool {
        if self
            .send_cmd_wait_resp_n(ctx, b"AT\n", b"OK", 50, 10, 3)
            .is_ok()
        {
            match &self.state_machine {
                EspStateWrapper::InitialEsp(val) => {
                    self.state_machine = EspStateWrapper::ConnectedEsp(val.into());
                    write_log(b"ESP Connected!");
                }
                _ => {}
            }
            return true;
        } else {
            self.state_machine = Default::default();
            return false;
        }
    }

    // отправка команды и получение ответа от ESP
    fn send_cmd_wait_resp(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8], // команда
        toc: u16,      // timeout for first char
        to: u16,
    ) -> Result<(), Error> {
        // читаем мусор из порта и игнорим все ошибки
        loop {
            let res = self.esp_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"ESP e0");
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
            block!(self.esp_port.write(*cmd)).ok();
        }
        // пробуем получить ответ
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        loop {
            let res = self.esp_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"ESP e1");
                    return Err(Error::SerialError);
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            write_log(b"ESP e2");
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
                    // получили очередной символ от ESP
                    if self.rcv_buf.len() < ESP_BUF_LEN {
                        // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                        //write_log(b" ESP resp char:");
                        //write_log(&[x]);
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
            write_log(b"ESP nrsp");
            return Err(Error::SerialNoData);
        }
        Ok(())
    }

    // отправка команды за несколько попыток
    fn send_cmd_wait_resp_n(
        &mut self,
        ctx: &mut Context,
        at_cmd: &[u8],
        ans: &[u8],
        toc: u16, // timeout for first char
        to: u16,  // timeout after last char
        tries: u8,
    ) -> Result<(), Error> {
        // no of attempts
        // checks if reply from ESP contains ans using tries attempts
        let mut reply: bool = false;
        for _ in 0..tries {
            match self.send_cmd_wait_resp(ctx, at_cmd, toc, to) {
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

    // PING удаленного модуля реле
    pub fn esp_ping(&mut self, ctx: &mut Context) -> Result<(), Error> {
        if self.state_machine == EspStateWrapper::default() {
            return Err(Error::EspOffLine);
        }
        self.send_cmd_wait_resp_n(ctx, b"PING\n", b"OK", 3500, 20, 1)
    }

    // RESET удаленного модуля реле
    pub fn esp_reset(&mut self, ctx: &mut Context) -> Result<(), Error> {
        if self.state_machine == EspStateWrapper::default() {
            return Err(Error::EspOffLine);
        }
        self.send_cmd_wait_resp_n(ctx, b"RESET\n", b"OK", 15000, 50, 1)
    }

    // ERROR удаленного модуля реле
    pub fn get_error(&mut self) -> &[u8] {
        if self.rcv_buf.starts_with(b"ERROR ") && self.rcv_buf.len() > 6 {
            return &self.rcv_buf[..7];
        }
        b"ERR?"
    }
}
