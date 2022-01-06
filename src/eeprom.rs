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
        r as u32
    }
}
