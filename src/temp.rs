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
            address: EepromAdresses::TempState.into(),
            temp: 0.0,
            one_wire_bus: bus,
        }
    }

    /// измерение температуры
    pub fn measure(&mut self, ctx: &mut Context) -> f32 {
        let w1 =
            ds18b20::start_simultaneous_temp_measurement(&mut self.one_wire_bus, &mut ctx.delay);

        return match w1 {
            Err(_) => {
                writeln!(ctx.console, "OW e1").unwrap();
                -101.0
            }
            Ok(_) => {
                Resolution::Bits12.delay_for_measurement_time(&mut ctx.delay);
                let search_state = None;
                let w2 =
                    self.one_wire_bus
                        .device_search(search_state.as_ref(), false, &mut ctx.delay);
                match w2 {
                    Err(_) => {
                        writeln!(ctx.console, "OW e2").unwrap();
                        -102.0
                    }
                    Ok(None) => {
                        writeln!(ctx.console, "OW none").unwrap();
                        -103.0
                    }
                    Ok(Some((device_address, _state))) => {
                        // search_state = Some(state); // у нас только один датчик, дальше не ищем
                        if device_address.family_code() == ds18b20::FAMILY_CODE {
                            let w0: core::result::Result<
                                ds18b20::Ds18b20,
                                one_wire_bus::OneWireError<core::convert::Infallible>,
                            > = Ds18b20::new(device_address);
                            match w0 {
                                Err(_) => {
                                    writeln!(ctx.console, "OW e5").unwrap();
                                    -104.0
                                }
                                Ok(sensor) => {
                                    let w3 =
                                        sensor.read_data(&mut self.one_wire_bus, &mut ctx.delay);
                                    match w3 {
                                        Err(_) => {
                                            writeln!(ctx.console, "OW e4").unwrap();
                                            -105.0
                                        }
                                        Ok(sensor_data) => {
                                            //writeln!(console, "Device at {:?} is {}C", device_address, sensor_data.temperature);
                                            self.temp = sensor_data.temperature;
                                            self.temp
                                        }
                                    }
                                }
                            }
                        } else {
                            writeln!(ctx.console, "OW e3").unwrap();
                            -106.0
                        }
                    }
                }
            }
        };
    }

    pub fn get_temp(&mut self) -> f32 {
        self.temp
    }
}

impl Eeprom for TempControl {
    /// восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) {
        if self.state != State::ColdStart {
            return;
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        self.state = read_data.into();
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let data = self.state.into();
        ctx.save_byte(self.address as u32, data)
    }
}

impl Observable for TempControl {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            State::ColdStart => {
                self.measure(ctx);
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
