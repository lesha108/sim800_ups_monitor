use super::*;
use core::marker::PhantomData;

use crate::traits::Observable;
use crate::context::Context;
use crate::eeprom::{Eeprom, EepromAdresses};
use crate::sim800l::SmsType::*;

//--------------------------------------------------------------
// Функции отслеживания состояний датчиков 220в и температуры
//--------------------------------------------------------------

// структура для отслеживания диапазона датчика температуры DS18B20

const TEMP_LIMIT_HIGH: f32 = 20.0; // приемлемое значение температуры
const TEMP_LIMIT_LOW: f32 = 15.0; // пороговое значение, означающее критическое понижение температуры

// Объекты возможных состояний машины
struct ColdStartT;
struct MonitoringT;
struct WaitForNormalT;

// конкретное состояние
struct TempState<S> {
    state: PhantomData<S>,
}

// конкретные типы состояний
type TEMPS = TempState<ColdStartT>;
type TEMPM = TempState<MonitoringT>;
type TEMPW = TempState<WaitForNormalT>;

// допустимые методы перехода между состояниями
impl From<&TEMPS> for TEMPM {
    fn from(_val: &TEMPS) -> TEMPM {
        TempState { state: PhantomData }
    }
}
impl From<&TEMPM> for TEMPW {
    fn from(_val: &TEMPM) -> TEMPW {
        TempState { state: PhantomData }
    }
}
impl From<&TEMPW> for TEMPM {
    fn from(_val: &TEMPW) -> TEMPM {
        TempState { state: PhantomData }
    }
}

// обертка допустимых состояний для работы match
enum TempStateWrapper {
    ColdStart(TEMPS),
    Monitoring(TEMPM),
    WaitForNormal(TEMPW),
}

impl Default for TempStateWrapper {
    fn default() -> Self {
        TempStateWrapper::ColdStart(TEMPS { state: PhantomData })
    }
}

// конвертация в u8 и обратно для сохранения в EEPROM
impl From<&TempStateWrapper> for u8 {
    fn from(dr: &TempStateWrapper) -> Self {
        match dr {
            TempStateWrapper::ColdStart(_) => 0,
            TempStateWrapper::Monitoring(_) => 1,
            TempStateWrapper::WaitForNormal(_) => 2,
        }
    }
}

impl From<u8> for TempStateWrapper {
    fn from(dr: u8) -> Self {
        match dr {
            0 => Default::default(),
            1 => TempStateWrapper::Monitoring(TEMPM { state: PhantomData }),
            2 => TempStateWrapper::WaitForNormal(TEMPW { state: PhantomData }),
            _ => Default::default(),
        }
    }
}

// методы работы машины состояний
// создание машины
pub struct TempChecker {
    temp_checking_machine: TempStateWrapper,
    address: u8,
    temp: f32,
    one_wire_bus: OwBusType,
}

impl TempChecker {
    pub fn new(bus: OwBusType) -> Self {
        TempChecker {
            temp_checking_machine: Default::default(),
            address: EepromAdresses::TempState as u8,
            temp: 0.0,
            one_wire_bus: bus,
        }
    }

    #[must_use]
    pub fn get_temp(&mut self) -> f32 {
        self.temp
    }

    // измерение температуры
    // ошибка передаётся как отрицательная температура. Её можно будет увидеть в SMS
    pub fn measure(&mut self, ctx: &mut Context) {
        let w1 =
            ds18b20::start_simultaneous_temp_measurement(&mut self.one_wire_bus, &mut ctx.delay);
        match w1 {
            Err(_) => {
                write_log(b"OW e1");
                self.temp = -101.0;
                return
            }
            Ok(_) => {
                Resolution::Bits12.delay_for_measurement_time(&mut ctx.delay);
                let search_state = None;
                let w2 =
                    self.one_wire_bus
                        .device_search(search_state.as_ref(), false, &mut ctx.delay);
                match w2 {
                    Err(_) => {
                        write_log(b"OW e2");
                        self.temp = -102.0;
                        return 
                    }
                    Ok(None) => {
                        write_log(b"OW none");
                        self.temp = -103.0;
                        return
                    }
                    Ok(Some((device_address, _state))) => {
                        //search_state = Some(state); // у нас только один датчик, дальше не ищем
                        if device_address.family_code() == ds18b20::FAMILY_CODE {
                            let w0: core::result::Result<
                                ds18b20::Ds18b20,
                                one_wire_bus::OneWireError<core::convert::Infallible>,
                            > = Ds18b20::new(device_address);
                            match w0 {
                                Err(_) => {
                                    write_log(b"OW e5");
                                    self.temp = -104.0;
                                    return
                                }
                                Ok(sensor) => {
                                    let w3 =
                                        sensor.read_data(&mut self.one_wire_bus, &mut ctx.delay);
                                    match w3 {
                                        Err(_) => {
                                            write_log(b"OW e4");
                                            self.temp = -105.0;
                                            return
                                        }
                                        Ok(sensor_data) => {
                                            //writeln!(console, "Device at {:?} is {}C", device_address, sensor_data.temperature);
                                            self.temp = sensor_data.temperature;
                                            return
                                        }
                                    }
                                }
                            }
                        } else {
                            write_log(b"OW e3");
                            self.temp = -106.0;
                            return
                        }
                    }
                }
            }
        }
    }
}

impl Eeprom for TempChecker {
    /// восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) {
        let address = u32::from(self.address);
        let read_data = ctx.eeprom.read_byte(address).unwrap();
        self.temp_checking_machine = read_data.into();
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), Error> {
        let address = u32::from(self.address);
        let to_save: u8 = (&self.temp_checking_machine).into();
        ctx.save_byte(address, to_save)
    }
}

impl<PINS> Observable<PINS> for TempChecker {
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), Error> {
        match &self.temp_checking_machine {
            TempStateWrapper::ColdStart(val) => {
                self.temp_checking_machine = TempStateWrapper::Monitoring(val.into());
                self.measure(ctx);
            }
            TempStateWrapper::Monitoring(val) => {
                if self.temp > -99.0 && self.temp < TEMP_LIMIT_LOW {
                    // температура упала
                    sim.send_sms(ctx, b"Temp too low", Normal).ok();
                    write_log(b"SMS Temp too low");
                    ctx.beep();

                    self.temp_checking_machine = TempStateWrapper::WaitForNormal(val.into());
                    self.save(ctx).ok();
                }
            }
            TempStateWrapper::WaitForNormal(val) => {
                if self.temp > TEMP_LIMIT_HIGH {
                    // температура вернулось в норму
                    sim.send_sms(ctx, b"Temp OK", Normal).ok();
                    write_log(b"SMS Temp OK");
                    ctx.beep();

                    self.temp_checking_machine = TempStateWrapper::Monitoring(val.into());
                    self.save(ctx).ok();
                }
            }
        }
        Ok(())
    }
}

// константы подбираются экспериментально!!!
const VCC: f32 = 3.3; // напряжение питания МК
const V220_START: f32 = 1.5; // значение ADC, означающее приемлемое напряжение питания
const V220_LIMIT_LOW: f32 = 0.3; // пороговое значение ADC, означающее отсутствие питания

// Объекты возможных состояний машины
struct ColdStartV;
struct MonitoringV;
struct WaitForNormalV;

// конкретное состояние
struct V220State<S> {
    state: PhantomData<S>,
}

// конкретные типы состояний
type V220S = V220State<ColdStartV>;
type V220M = V220State<MonitoringV>;
type V220W = V220State<WaitForNormalV>;

// допустимые методы перехода между состояниями
impl From<&V220S> for V220M {
    fn from(_val: &V220S) -> V220M {
        V220State { state: PhantomData }
    }
}
impl From<&V220M> for V220W {
    fn from(_val: &V220M) -> V220W {
        V220State { state: PhantomData }
    }
}
impl From<&V220W> for V220M {
    fn from(_val: &V220W) -> V220M {
        V220State { state: PhantomData }
    }
}

// обертка допустимых состояний для работы match
enum V220StateWrapper {
    ColdStart(V220S),
    Monitoring(V220M),
    WaitForNormal(V220W),
}

impl Default for V220StateWrapper {
    fn default() -> Self {
        V220StateWrapper::ColdStart(V220S { state: PhantomData })
    }
}

// конвертация в u8 и обратно для сохранения в EEPROM
impl From<&V220StateWrapper> for u8 {
    fn from(dr: &V220StateWrapper) -> Self {
        match dr {
            V220StateWrapper::ColdStart(_) => 0,
            V220StateWrapper::Monitoring(_) => 1,
            V220StateWrapper::WaitForNormal(_) => 2,
        }
    }
}

impl From<u8> for V220StateWrapper {
    fn from(dr: u8) -> Self {
        match dr {
            0 => Default::default(),
            1 => V220StateWrapper::Monitoring(V220M { state: PhantomData }),
            2 => V220StateWrapper::WaitForNormal(V220W { state: PhantomData }),
            _ => Default::default(),
        }
    }
}

// методы работы машины состояний
// создание машины
pub struct V220Checker {
    v220_checking_machine: V220StateWrapper,
    address: u8,
    voltage: f32,
    int_temp: i32,
    analog_input: AnalogIn0Type,
    adc: Adc1Type,
}

impl V220Checker {
    pub fn new(pin: AnalogIn0Type, adc1: Adc1Type) -> Self {
        V220Checker {
            v220_checking_machine: Default::default(),
            address: EepromAdresses::V220State as u8,
            voltage: 0.0,
            int_temp: 0,
            analog_input: pin,
            adc: adc1,
        }
    }

    // измерение напряжения 220 в
    pub fn measure(&mut self) {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap(); //3.3d = 4095, 3.3/2 = 2036
            averaged += u32::from(data);
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
    }

    #[must_use]
    pub fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    // измерение температуры чипа
    pub fn measure_temp(&mut self) {
        self.int_temp = self.adc.read_temp();
    }

    #[must_use]
    pub fn get_temp(&mut self) -> i32 {
        self.int_temp
    }
}

impl Eeprom for V220Checker {
    /// восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) {
        let address = u32::from(self.address);
        let read_data = ctx.eeprom.read_byte(address).unwrap();
        self.v220_checking_machine = read_data.into();
    }

    /// запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), Error> {
        let address = u32::from(self.address);
        let to_save: u8 = (&self.v220_checking_machine).into();
        ctx.save_byte(address, to_save)
    }
}

impl<PINS> Observable<PINS> for V220Checker {
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), Error> {
        match &self.v220_checking_machine {
            V220StateWrapper::ColdStart(val) => {
                self.v220_checking_machine = V220StateWrapper::Monitoring(val.into());
                self.measure();
            }
            V220StateWrapper::Monitoring(val) => {
                if self.voltage < V220_LIMIT_LOW {
                    // сетевое напряжение пропало
                    sim.send_sms(ctx, b"220 failed", Normal).ok();
                    write_log(b"SMS 220 failed");
                    ctx.beep();

                    self.v220_checking_machine = V220StateWrapper::WaitForNormal(val.into());
                    self.save(ctx).ok();
                }
            }
            V220StateWrapper::WaitForNormal(val) => {
                if self.voltage > V220_START {
                    // сетевое напряжение вернулось
                    sim.send_sms(ctx, b"220 on-line", Normal).ok();
                    write_log(b"SMS 220 on-line");
                    ctx.beep();

                    self.v220_checking_machine = V220StateWrapper::Monitoring(val.into());
                    self.save(ctx).ok();
                }
            }
        }
        Ok(())
    }
}

// структура для отслеживания напряжения батареи питания МК

pub struct Battery {
    voltage: f32,
    analog_input: AnalogIn1Type,
    adc: Adc2Type,
}

impl Battery {
    pub fn new(pin: AnalogIn1Type, adc2: Adc2Type) -> Self {
        Battery {
            voltage: 0.0,
            analog_input: pin,
            adc: adc2,
        }
    }

    #[must_use]
    pub fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    // измерение напряжения батареи
    pub fn measure(&mut self) {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap(); //3.3d = 4095, 3.3/2 = 2036
            averaged += u32::from(data);
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
    }
}
