// use stm32f1xx_hal::prelude::*;
//
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::interrupt;
use stm32f1xx_hal::timer::CountDownTimer;

// Статические переменные для работы с прерыванием таймера
pub static mut SEC_TIMER: MaybeUninit<CountDownTimer<pac::TIM2>> = MaybeUninit::uninit();
pub static SEC_COUNT_REBOOT: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_SENSOR: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_LED: AtomicU32 = AtomicU32::new(0);

#[allow(non_snake_case)]
#[interrupt]
fn TIM2() {
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    SEC_COUNT_REBOOT.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_SENSOR.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_LED.fetch_add(1, Ordering::Relaxed);
    sec_timer.clear_update_interrupt_flag();
}
