use crate::context::Context;

pub trait Eeprom {
    fn load(&mut self, ctx: &mut Context);
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EepromAdresses {
    /// состояние датчика напряжения 220 в
    V220State = 0x01,
    /// состояние датчика температуры DS18B20
    TempState = 0x02,
    /// телефонный номер отправки SMS
    Number = 0x04,
}

#[derive(PartialEq, Copy, Clone)]
pub enum State {
    ColdStart = 0,
    Monitoring = 1,
    WaitForNormal = 2,
}

impl From<u8> for State {
    fn from(value: u8) -> Self {
        match value {
            // 0 => State::ColdStart,
            1 => State::Monitoring,
            2 => State::WaitForNormal,
            _ => State::ColdStart,
        }
    }
}
