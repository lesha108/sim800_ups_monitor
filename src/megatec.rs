use super::*;

//--------------------------------------------------------------
// Функции работы с UPS
//--------------------------------------------------------------
const UPS_BUF_LEN: usize = 50; // длина буфера приёма статуса UPS
const TIMEOUT_FIRST_CYCLES: u16 = 200; // wait first char for 2 sec
const TIMEOUT_LAST_CYCLES: u16 = 10; // wait finish char for 100 ms

pub struct Ups<PINS> {
    snd_buf: Vec<u8, 4>,
    rcv_buf: Vec<u8, UPS_BUF_LEN>,
    ups_port: UpsPortType<PINS>,
}

impl<PINS> Ups<PINS> {
    pub fn new(port: UpsPortType<PINS>) -> Self {
        Ups {
            snd_buf: Vec::new(),
            rcv_buf: Vec::new(),
            ups_port: port,
        }
    }

    #[must_use]
    pub fn get_ups(&mut self) -> &[u8] {
        // должно быть что-то типа
        // (218.1 218.1 219.6 000 50.0 2.22 48.0 00000001\r
        // вырезаем первый и последний символы
        &self.rcv_buf[1..self.rcv_buf.len() - 1] // !!! проблема, если буфер пуст, т.е. не вызвали measure
    }

    pub fn measure(&mut self, ctx: &mut Context) -> Result<(), Error> {
        // отправляем команду получения статуса UPS
        self.snd_buf.clear();
        self.snd_buf.extend_from_slice(b"Q1\r").unwrap();
        for cmd in &self.snd_buf {
            block!(self.ups_port.write(*cmd)).ok();
        }
        // пробуем получить ответ
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        loop {
            let res = self.ups_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    write_log(b"UPS e1");
                    self.invalidate();
                    return Err(Error::SerialError);
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    let t = ctx.at_timer.wait();
                    match t {
                        Err(nb::Error::Other(_)) => {
                            write_log(b"UPS e2");
                            self.invalidate();
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
                                if w2_cycles >= TIMEOUT_LAST_CYCLES {
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1;
                                    continue;
                                }
                            } else {
                                // отрабатываем ожидание первого символа
                                if w1_cycles >= TIMEOUT_FIRST_CYCLES {
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
                    // получили очередной символ от UPS
                    if self.rcv_buf.len() < UPS_BUF_LEN {
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
        if self.rcv_buf.len() != 47 {
            // длина нормального ответа от UPS
            write_log(b"UPS ret: ");
            write_log(self.rcv_buf.as_slice());
            self.invalidate();
            return Err(Error::UPSFailure);
        }
        Ok(())
    }

    // стандартное содержимое при ошибке связи с UPS
    fn invalidate(&mut self) {
        self.rcv_buf.clear();
        self.rcv_buf.extend_from_slice(b"!UPS failure!!").unwrap();
    }
}
