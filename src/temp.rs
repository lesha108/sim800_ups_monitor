use core::fmt::Write;
use ds18b20::{Ds18b20, Resolution};

use crate::context::Context;
use crate::eeprom::{Eeprom, EepromAdresses, State};
use crate::sim::Sim800;
use crate::traits::Observable;

/// приемлемое значение температуры
const TEMP_LIMIT_HIGH: f32 = 20.0;
/// пороговое значение, означающее критическое понижение температуры
const TEMP_LIMIT_LOW: f32 = 15.0;

/// структура для отслеживания диапазона датчика температуры DS18B20
pub struct TempControl {
    state: State,
    address: u8,
    temp: f32,
    one_wire_bus: one_wire_bus::OneWire<
        stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
    >,
}

impl TempControl {
    pub fn new(
        bus: one_wire_bus::OneWire<
            stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
        >,
    ) -> Self {
        TempControl {
            state: State::ColdStart,
            address: EepromAdresses::TempState as u8,
            temp: 0.0,
            one_wire_bus: bus,
        }
    }

    pub fn get_temp(&mut self) -> f32 {
        self.temp
    }

    /// измерение температуры
    pub fn measure(&mut self, ctx: &mut Context) -> Result<(), u8> {
        if let Err(e) = Self::inter_measure(self, ctx) {
            writeln!(ctx.console, "e{}", e).unwrap();
            return Err(e);
        }
        Ok(())
    }

    fn inter_measure(&mut self, ctx: &mut Context) -> Result<(), u8> {
        let w1 =
            ds18b20::start_simultaneous_temp_measurement(&mut self.one_wire_bus, &mut ctx.delay);
        if w1.is_err() {
            return Err(1);
        }
        Resolution::Bits12.delay_for_measurement_time(&mut ctx.delay);
        let search_state = None;
        let w0 = self
            .one_wire_bus
            .device_search(search_state.as_ref(), false, &mut ctx.delay);
        match w0 {
            Ok(Some((device_address, _state))) => {
                // search_state = Some(state); // у нас только один датчик, дальше не ищем
                if device_address.family_code() != ds18b20::FAMILY_CODE {
                    return Err(4);
                }
                let w0: core::result::Result<
                    ds18b20::Ds18b20,
                    one_wire_bus::OneWireError<core::convert::Infallible>,
                > = Ds18b20::new(device_address);
                match w0 {
                    Ok(sensor) => {
                        let w3 = sensor.read_data(&mut self.one_wire_bus, &mut ctx.delay);
                        match w3 {
                            Ok(sensor_data) => {
                                //writeln!(console, "Device at {:?} is {}C", device_address, sensor_data.temperature);
                                self.temp = sensor_data.temperature;
                                Ok(())
                            }
                            Err(_) => Err(5),
                        }
                    }
                    Err(_) => Err(4),
                }
            }
            Ok(None) => Err(2),
            Err(_) => Err(3),
        }
    }
}

impl Eeprom for TempControl {
    /// восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) {
        if self.state != State::ColdStart {
            return;
        }
        let address = u32::from(self.address);
        let read_data = ctx.eeprom.read_byte(address).unwrap();
        self.state = read_data.into();
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let address = u32::from(self.address);
        let data = self.state as u8;
        ctx.save_byte(address, data)
    }
}

impl Observable for TempControl {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            State::ColdStart => {
                let _ = self.measure(ctx);
                self.state = State::Monitoring;
            }
            State::Monitoring => {
                if self.temp > -99.0 && self.temp < TEMP_LIMIT_LOW {
                    // температура упала
                    writeln!(ctx.console, "Temp too low {}", self.temp).unwrap();
                    ctx.beep();
                    let _ = sim.send_sms(ctx, b"Temp too low");
                    self.state = State::WaitForNormal;
                    return self.save(ctx);
                }
            }
            State::WaitForNormal => {
                if self.temp > TEMP_LIMIT_HIGH {
                    // температура вернулось в норму
                    writeln!(ctx.console, "Temp OK").unwrap();
                    ctx.beep();
                    let _ = sim.send_sms(ctx, b"Temp OK");
                    self.state = State::Monitoring;
                    return self.save(ctx);
                }
            }
        }
        Ok(())
    }
}
