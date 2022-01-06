use crate::context::Context;
use crate::sim::Sim800;

/// характеристика указывает, что объект можно взять на контроль
pub trait Observable {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()>;
}
