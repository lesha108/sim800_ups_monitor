use core::fmt::Write;
use embedded_hal::adc::OneShot;
use stm32f1xx_hal::{adc, pac};

use crate::context::Context;
use crate::eeprom::EepromAdresses;
use crate::sim::Sim800;
use crate::traits::Observable;

// константы подбираются экспериментально!!!
/// напряжение питания МК
const VCC: f32 = 3.3;
/// значение ADC, означающее приемлемое напряжение питания
const V220_START: f32 = 1.5;
/// пороговое значение ADC, означающее отсутствие питания
const V220_LIMIT_LOW: f32 = 0.3;

#[derive(PartialEq, Copy, Clone)]
enum V220States {
    /// инициализация измерений
    ColdStart,
    /// ожидание падения напряжения 220 в
    Monitoring,
    /// ожидание восстановления 220 в
    WaitForNormal,
}

impl From<u8> for V220States {
    fn from(value: u8) -> Self {
        match value {
            0 => V220States::ColdStart,
            1 => V220States::Monitoring,
            2 => V220States::WaitForNormal,
            _ => V220States::ColdStart,
        }
    }
}

impl Into<u8> for V220States {
    fn into(self) -> u8 {
        match self {
            V220States::ColdStart => 0,
            V220States::Monitoring => 1,
            V220States::WaitForNormal => 2,
        }
    }
}

/// структура для отслеживания напряжения 220в питания МК
pub struct V220Control {
    state: V220States,
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
            state: V220States::ColdStart,
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
            averaged += data as u32;
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

    /// восстановление состояния из EEPROM после перезагрузки МК
    pub fn load(&mut self, ctx: &mut Context) -> Result<(), ()> {
        if self.state != V220States::ColdStart {
            return Ok(()); // todo
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        self.state = V220States::from(read_data);
        Ok(())
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let data = self.state.into();
        ctx.save_byte(self.address as u32, data)
    }
}

impl Observable for V220Control {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            V220States::ColdStart => {
                self.measure();
                self.state = V220States::Monitoring;
            }
            V220States::Monitoring => {
                if self.voltage < V220_LIMIT_LOW {
                    // сетевое напряжение пропало
                    sim.send_sms(ctx, b"220 failed");
                    writeln!(ctx.console, "220 failed {}", self.voltage).unwrap();
                    ctx.beep();

                    self.state = V220States::WaitForNormal;
                    self.save(ctx);
                }
            }
            V220States::WaitForNormal => {
                if self.voltage > V220_START {
                    // сетевое напряжение вернулось
                    sim.send_sms(ctx, b"220 on-line");
                    writeln!(ctx.console, "220 on-line").unwrap();
                    ctx.beep();

                    self.state = V220States::Monitoring;
                    self.save(ctx);
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
            averaged += data as u32;
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
        self.voltage
    }
}
