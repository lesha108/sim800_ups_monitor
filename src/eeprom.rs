use crate::context::Context;

pub trait Eeprom {
    fn load(&mut self, ctx: &mut Context);
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EepromAdresses {
    /// состояние датчика напряжения 220 в
    V220State,
    /// состояние датчика температуры DS18B20
    TempState,
    /// телефонный номер отправки SMS
    Number,
}

impl Into<u8> for EepromAdresses {
    /// возвращает адрес сохраненного значения в чипе EEPROM
    fn into(self) -> u8 {
        match self {
            EepromAdresses::V220State => 0x01,
            EepromAdresses::TempState => 0x02,
            EepromAdresses::Number => 0x04,
        }
    }
}

impl Into<u32> for EepromAdresses {
    fn into(self) -> u32 {
        let r: u8 = Self::into(self);
        u32::from(r)
    }
}

#[derive(PartialEq, Copy, Clone)]
pub enum State {
    ColdStart,
    Monitoring,
    WaitForNormal,
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

impl Into<u8> for State {
    fn into(self) -> u8 {
        match self {
            State::ColdStart => 0,
            State::Monitoring => 1,
            State::WaitForNormal => 2,
        }
    }
}
