use core::fmt::Write;
use embedded_hal::adc::OneShot;
use stm32f1xx_hal::{adc, pac};

use crate::context::Context;
use crate::eeprom::{Eeprom, EepromAdresses, State};
use crate::sim::Sim800;
use crate::traits::Observable;

// константы подбираются экспериментально!!!
/// напряжение питания МК
const VCC: f32 = 3.3;
/// значение ADC, означающее приемлемое напряжение питания
const V220_START: f32 = 1.5;
/// пороговое значение ADC, означающее отсутствие питания
const V220_LIMIT_LOW: f32 = 0.3;

/// структура для отслеживания напряжения 220в питания МК
pub struct V220Control {
    state: State,
    address: u8,
    voltage: f32,
    int_temp: i32,
    analog_input: stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>,
    adc: adc::Adc<pac::ADC1>,
}

impl V220Control {
    pub fn new(
        pin: stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>,
        adc1: adc::Adc<pac::ADC1>,
    ) -> Self {
        V220Control {
            state: State::ColdStart,
            address: EepromAdresses::V220State.into(),
            voltage: 0.0,
            int_temp: 0,
            analog_input: pin,
            adc: adc1,
        }
    }

    /// измерение напряжения 220 в
    pub fn measure(&mut self) -> f32 {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap();
            //3.3d = 4095, 3.3/2 = 2036
            averaged += u32::from(data);
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
        self.voltage
    }

    pub fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    /// измерение температуры чипа
    pub fn measure_temp(&mut self) -> i32 {
        self.int_temp = self.adc.read_temp();
        self.int_temp
    }

    pub fn get_temp(&mut self) -> i32 {
        self.int_temp
    }
}

impl Eeprom for V220Control {
    /// восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) {
        if self.state != State::ColdStart {
            return;
        }
        let address = u32::from(self.address);
        let read_data = ctx.eeprom.read_byte(address).unwrap();
        self.state = State::from(read_data);
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let address = u32::from(self.address);
        let data = self.state.into();
        ctx.save_byte(address, data)
    }
}

impl Observable for V220Control {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            State::ColdStart => {
                self.measure();
                self.state = State::Monitoring;
            }
            State::Monitoring => {
                if self.voltage < V220_LIMIT_LOW {
                    // сетевое напряжение пропало
                    writeln!(ctx.console, "220 failed {}", self.voltage).unwrap();
                    ctx.beep();
                    let _ = sim.send_sms(ctx, b"220 failed");
                    self.state = State::WaitForNormal;
                    return self.save(ctx);
                }
            }
            State::WaitForNormal => {
                if self.voltage > V220_START {
                    // сетевое напряжение вернулось
                    writeln!(ctx.console, "220 on-line").unwrap();
                    ctx.beep();
                    let _ = sim.send_sms(ctx, b"220 on-line");
                    self.state = State::Monitoring;
                    return self.save(ctx);
                }
            }
        }
        Ok(())
    }
}

/// структура для отслеживания напряжения батареи питания МК
pub struct Battery {
    voltage: f32,
    analog_input: stm32f1xx_hal::gpio::gpiob::PB1<stm32f1xx_hal::gpio::Analog>,
    adc: adc::Adc<pac::ADC2>,
}

impl Battery {
    pub fn new(
        pin: stm32f1xx_hal::gpio::gpiob::PB1<stm32f1xx_hal::gpio::Analog>,
        adc2: adc::Adc<pac::ADC2>,
    ) -> Battery {
        Battery {
            voltage: 0.0,
            analog_input: pin,
            adc: adc2,
        }
    }

    pub fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    //todo копипаста
    /// измерение напряжения батареи
    pub fn measure(&mut self) -> f32 {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap();
            //3.3d = 4095, 3.3/2 = 2036
            averaged += u32::from(data);
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
        self.voltage
    }
}
