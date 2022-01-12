use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::interrupt;
use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::usb::UsbBusType;
use usb_device::{bus::UsbBusAllocator, prelude::*};

// переменные и прерывания для работы USB
pub static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
pub static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

fn usb_interrupt() {
    cortex_m::interrupt::free(|_| {
        let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
        let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
        usb_dev.poll(&mut [serial]);
    });
}

// Статические переменные для работы с прерываеним таймера
pub static mut SEC_TIMER: MaybeUninit<CountDownTimer<pac::TIM2>> =
    MaybeUninit::uninit();
pub static SEC_COUNT_REBOOT: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_SENSOR: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_LED: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_ESP: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_BALANCE: AtomicU32 = AtomicU32::new(0);

// обаботчик прерывания таймера
#[interrupt]
fn TIM2() {
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    SEC_COUNT_REBOOT.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_SENSOR.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_LED.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_ESP.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_BALANCE.fetch_add(1, Ordering::Relaxed);
    sec_timer.clear_update_interrupt_flag();
}
