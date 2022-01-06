/// Функции работы с UPS
use stm32f1xx_hal::prelude::*;
//
use core::fmt::Write;
use heapless::Vec;
use nb::block;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::serial::Serial;

use crate::Context;

/// длина буфера приёма статуса UPS
const UPS_BUF_LEN: usize = 50;
/// wait first char for 2 sec
const TIMEOUT_FIRST_CYCLES: u16 = 200;
/// wait finish char for 100 ms
const TIMEOUT_LAST_CYCLES: u16 = 10;

pub struct Ups<PINS> {
    snd_buf: Vec<u8, 4>,
    rcv_buf: Vec<u8, UPS_BUF_LEN>,
    ups_port: Serial<pac::USART1, PINS>,
}

impl<PINS> Ups<PINS> {
    pub fn new(port: Serial<pac::USART1, PINS>) -> Self {
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

    pub fn measure(&mut self, ctx: &mut Context) -> Result<(), ()> {
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
                    writeln!(ctx.console, "UPS e1").unwrap();
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    // символ не пришёл ещё
                    match ctx.at_timer.wait() {
                        Ok(_) => {
                            // сработал таймер отсчёта таймаута
                            if got_first_char {
                                // отрабатываем ожидание последнего символа
                                if w2_cycles >= TIMEOUT_LAST_CYCLES {
                                    break; // вылет по таймауту
                                }
                            } else {
                                // отрабатываем ожидание первого символа
                                if w1_cycles >= TIMEOUT_FIRST_CYCLES {
                                    break;
                                }
                            }
                            w2_cycles += 1;
                            continue;
                        }
                        Err(nb::Error::WouldBlock) => {
                            // просто ждём ещё таймер
                            continue;
                        }
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "UPS e2").unwrap();
                            break;
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
            self.rcv_buf.clear();
            self.rcv_buf.extend_from_slice(b"!UPS failure!!").unwrap();
            writeln!(ctx.console, "UPS ret: {:?}", self.rcv_buf.as_slice()).unwrap();
            return Err(());
        }
        Ok(())
    }
}
