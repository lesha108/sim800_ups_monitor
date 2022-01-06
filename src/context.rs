use stm32f1xx_hal::prelude::*;
//
use stm32f1xx_hal::pac;
use stm32f1xx_hal::serial::Tx;
use stm32f1xx_hal::timer::CountDownTimer;
use stm32f1xx_hal::{delay::Delay, pwm::Channel, rtc::Rtc, watchdog::IndependentWatchdog};

/// Определение структуры аппаратного контекста
pub struct Context {
    pub watchdog: IndependentWatchdog,
    pub delay: Delay,
    pub at_timer: CountDownTimer<pac::TIM3>,
    pub rtc: Rtc,
    pub console: Tx<pac::USART2>,
    pub led: stm32f1xx_hal::gpio::gpioc::PC13<
        stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
    >,

    pub eeprom: eeprom24x::Eeprom24x<
        stm32f1xx_hal::i2c::BlockingI2c<
            stm32f1xx_hal::pac::I2C1,
            (
                stm32f1xx_hal::gpio::gpiob::PB6<
                    stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                >,
                stm32f1xx_hal::gpio::gpiob::PB7<
                    stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                >,
            ),
        >,
        eeprom24x::page_size::B16,
        eeprom24x::addr_size::OneByte,
    >,

    pub beeper: stm32f1xx_hal::pwm::Pwm<
        stm32f1xx_hal::pac::TIM1,
        stm32f1xx_hal::timer::Tim1NoRemap,
        stm32f1xx_hal::pwm::C1,
        stm32f1xx_hal::gpio::gpioa::PA8<
            stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>,
        >,
    >,
}

impl Context {
    pub fn beep(&mut self) {
        self.beeper
            .set_duty(Channel::C1, self.beeper.get_max_duty() / 2);
        self.watchdog.feed();
        self.delay.delay_ms(500_u16);
        self.beeper.set_duty(Channel::C1, 0);
        self.watchdog.feed();
    }

    pub fn save_byte(&mut self, address: u32, data: u8) -> Result<(), ()> {
        self.eeprom.write_byte(address, data).unwrap();
        self.delay.delay_ms(10_u16);
        let read_data = self.eeprom.read_byte(address).unwrap();
        if read_data == data {
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn reset_rtc(&mut self) {
        self.rtc.set_time(0);
        // Запустить через сутки
        self.rtc.set_alarm(24 * 60 * 60);
    }
}
