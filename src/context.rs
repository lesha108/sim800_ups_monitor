use stm32f1xx_hal::prelude::*;
//
use shared_bus::*;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::{delay::Delay, pwm::Channel, rtc::Rtc, watchdog::IndependentWatchdog};

use crate::errors::Error;
use crate::sim800l::Sim800;
use crate::traits::Observable;

/// Определение структуры аппаратного контекста
pub struct Context<'a> {
    /// сторожевой таймер
    pub watchdog: IndependentWatchdog,
    /// функции задержки
    pub delay: Delay,
    /// таймер отслеживания таймаутов в последовательных портах
    pub at_timer: CountDownTimer<pac::TIM3>,
    /// часы реального времени
    pub rtc: Rtc,
    /// heartbeat LED
    pub led: stm32f1xx_hal::gpio::gpioc::PC13<
        stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
    >,
    /// доступ к EEPROM
    pub eeprom: eeprom24x::Eeprom24x<
        I2cProxy<
            'a,
            NullMutex<
                stm32f1xx_hal::i2c::BlockingI2c<
                    pac::I2C1,
                    (
                        stm32f1xx_hal::gpio::gpiob::PB6<
                            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                        >,
                        stm32f1xx_hal::gpio::gpiob::PB7<
                            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                        >,
                    ),
                >,
            >,
        >,
        eeprom24x::page_size::B16,
        eeprom24x::addr_size::OneByte,
    >,
    /// функции пищалки
    pub beeper: stm32f1xx_hal::pwm::Pwm<
        pac::TIM1,
        stm32f1xx_hal::timer::Tim1NoRemap,
        stm32f1xx_hal::pwm::C1,
        stm32f1xx_hal::gpio::gpioa::PA8<
            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>,
        >,
    >,
}

impl<'a> Context<'a> {
    pub fn beep(&mut self) {
        self.beeper
            .set_duty(Channel::C1, self.beeper.get_max_duty() / 2);
        self.watchdog.feed();
        self.delay.delay_ms(500_u16);
        self.beeper.set_duty(Channel::C1, 0);
        self.watchdog.feed();
    }

    pub fn save_byte(&mut self, address: u32, data: u8) -> Result<(), Error> {
        self.eeprom.write_byte(address, data).unwrap();
        self.delay.delay_ms(10_u16);
        let read_data = self.eeprom.read_byte(address).unwrap();
        if read_data == data {
            return Ok(());
        } else {
            return Err(Error::EepromFail);
        }
    }

    pub fn reset_rtc(&mut self) {
        self.rtc.set_time(0);
        // Запустить через сутки
        self.rtc.set_alarm(24 * 60 * 60);
    }

    //fn check<O: Observable>(&mut self, control: &mut dyn Observable, sim: &mut Sim800) -> Result<(), Error> {
    pub fn check<PINS>(
        &mut self,
        control: &mut dyn Observable<PINS>,
        sim: &mut Sim800<PINS>,
    ) -> Result<(), Error> {
        control.check(self, sim)
    }
}
