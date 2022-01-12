use crate::context::Context;
use crate::sim800l::Sim800;
use crate::errors::Error;

/// характеристика указывает, что объект можно взять на контроль
pub trait Observable<PINS> {
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), Error>;
}

