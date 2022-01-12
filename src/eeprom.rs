use crate::context::Context;
use crate::errors::Error;

pub trait Eeprom {
    fn load(&mut self, ctx: &mut Context);
    fn save(&mut self, ctx: &mut Context) -> Result<(), Error>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EepromAdresses {
    V220State = 0x01 , // состояние датчика напряжения 220 в
    TempState = 0x02, // состояние датчика температуры DS18B20
    Number = 0x04,    // телефонный номер отправки SMS
}
