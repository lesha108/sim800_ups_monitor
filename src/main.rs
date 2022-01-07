#![no_std]
#![no_main]
#![warn(clippy::pedantic)]

mod context;
mod eeprom;
mod interrupts;
mod sim;
mod temp;
mod traits;
mod ups;
mod voltage;

use stm32f1xx_hal::prelude::*;
//
use core::fmt::Write;
use core::sync::atomic::Ordering;
use cortex_m_rt::entry;
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::blocking::delay::DelayMs;
use heapless::Vec;
use one_wire_bus::OneWire;
use panic_halt as _;
use stm32f1xx_hal::{
    adc,
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    pwm::Channel,
    rtc::Rtc,
    serial::{Config, Serial},
    time::MilliSeconds,
    timer::{Event, Timer},
    watchdog::IndependentWatchdog,
};
//use cortex_m::asm;
//use cortex_m_rt::ExceptionFrame;  //  Stack frame for exception handling.
//use cortex_m_semihosting::hio;  //  For displaying messages on the debug console.

use crate::context::Context;
use crate::eeprom::Eeprom;
use crate::interrupts::{SEC_COUNT_REBOOT, SEC_COUNT_SENSOR, SEC_TIMER};
use crate::sim::Sim800;
use crate::temp::TempControl;
use crate::traits::Observable;
use crate::ups::Ups;
use crate::voltage::{Battery, V220Control};

/// Определяем входную функцию
#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    //Show "Hello, world!" on the debug console, which is shown in OpenOCD
    //let mut debug_out = hio::hstdout().unwrap();
    //writeln!(debug_out, "Hello, world!").unwrap();

    // Получаем управление над аппаратными средствами
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz()) // переключились на кварц
        .sysclk(16.mhz()) // должна стать 16
        .adcclk(2.mhz()) // для ADC
        .freeze(&mut flash.acr);
    // Prepare the alternate function I/O registers
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    //let channels = dp.DMA1.split(&mut rcc.ahb);

    // Настройка пина встроенного в bluepill светодиода
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Setup ADC
    let adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
    let adc2 = adc::Adc::adc2(dp.ADC2, &mut rcc.apb2, clocks);
    // Configure pb0, pb1 as an analog input for ADC
    let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
    let ch1 = gpiob.pb1.into_analog(&mut gpiob.crl);

    // Set up the RTC
    // Enable writes to the backup domain
    let mut pwr = dp.PWR;
    let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut rcc.apb1, &mut pwr);
    // Start the RTC
    let rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    // USART1 - ups
    let serial1 = {
        // Configure pa9 as a push_pull output, this will be the tx pin
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;

        Serial::usart1(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(2400.bps()),
            clocks,
            &mut rcc.apb2,
        )
    };

    // USART2 - debug print
    // writeln!(console, "Hello formatted string {}", number).unwrap();
    let console = {
        // Configure pa2 as a push_pull output, this will be the tx pin
        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;

        let serial = Serial::usart2(
            dp.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb1,
        );
        // Split the serial struct into a receiving and a transmitting part
        let (tx_con, _) = serial.split();
        tx_con
    };

    // USART3 - sim800l
    let serial3 = {
        // Configure pb10 as a push_pull output, this will be the tx pin
        let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let rx = gpiob.pb11;

        Serial::usart3(
            dp.USART3,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1,
        )
    };

    // таймер для всяких секундных интервалов
    SEC_COUNT_REBOOT.store(0, Ordering::Relaxed);
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    *sec_timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
    // запускаем прерываение по таймеру
    sec_timer.listen(Event::Update);
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIM2) };

    // таймер для отслеживания таймаутов cимволов USART. должен срабатывать через 10 мс
    let at_timer = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(100.hz());

    // Create a delay timer from the RCC clocks
    let delay = Delay::new(cp.SYST, clocks);

    // ШИМ канал для работы пищалки
    let beeper = {
        // Configure pa8 as a push_pull output
        let pa8 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        // выдаём 1 кГц на этот пин
        let mut pwm =
            Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).pwm(pa8, &mut afio.mapr, 1.khz());
        pwm.enable(Channel::C1);
        pwm.set_duty(Channel::C1, 0);
        pwm
    };

    // I2C setup for EEPROM
    let i2c = {
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 100_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        )
    };

    // Настройка работы чипа EEPROM
    let eeprom = {
        let address = SlaveAddr::default();
        Eeprom24x::new_24x16(i2c, address)
    };

    // Настройка сторожевого таймера
    let mut watchdog = IndependentWatchdog::new(dp.IWDG);
    // запускаем с 20 сек интервалом сброса
    watchdog.start(MilliSeconds(20_000));

    // пин связан с контактом reset SIM800. Притягивание к земле перегружает модуль
    let mut sim800 = {
        let sim_reset_pin = gpioa
            .pa6
            .into_open_drain_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
        Sim800::new(serial3, sim_reset_pin)
    };

    // создаём переменную с контекстом, которую будем передавать в другие функции
    let mut ctx = Context {
        watchdog,
        delay,
        at_timer,
        rtc,
        console,
        led,
        eeprom,
        beeper,
    };
    // Гудок при старте контроллера
    ctx.beep();

    let mut reboot_count: u8 = 0; // число попыток восстановления связи с SIM800

    //

    // объект для взаимодействия с UPS
    let mut ups = Ups::new(serial1);
    let _ = ups.measure(&mut ctx); // первый вызов корректно инициализирует буфер
    ctx.watchdog.feed();

    // объект контроля 220в
    let mut v220control = V220Control::new(ch0, adc1);
    v220control.load(&mut ctx);
    v220control.measure();
    v220control.measure_temp();
    ctx.watchdog.feed();

    // объект для взаимодействия с батареей МК
    let mut battery = Battery::new(ch1, adc2);
    battery.measure();
    ctx.watchdog.feed();

    // OneWire DS18b20 line
    let one_wire_bus = {
        let one_wire_pin = gpiob
            .pb12
            .into_open_drain_output(&mut gpiob.crh)
            .downgrade();
        OneWire::new(one_wire_pin).unwrap()
    };
    // объект контроля температуры DS18B20
    let mut temp_control = TempControl::new(one_wire_bus);
    temp_control.load(&mut ctx);
    let _ = temp_control.measure(&mut ctx);

    loop {
        // сбрасываем сторожевой таймер
        ctx.watchdog.feed();

        // проверяем связь с SIM800L Если нет, то пробуем перегрузить SIM800
        // если не помогает перезагрузка, пробуем перегрузить микроконтроллер
        if sim800.check_com(&mut ctx).is_err() {
            writeln!(ctx.console, "SIM com failed. SIM800 Reboot").unwrap();
            sim800.reboot(&mut ctx);
            for _ in 0..10 {
                ctx.delay.delay_ms(3_000_u16);
                ctx.watchdog.feed();
            }
            // если не помогает перегрузка модуля, делаем ребут всего контроллера
            if reboot_count > 3 {
                writeln!(ctx.console, "Self reboot").unwrap();
                loop {} // должны перегрузиться по сторожевому таймеру
            } else {
                reboot_count += 1;
            }
            continue; // зацикливаемся пока не будет связи с SIM800
        } else {
            reboot_count = 0;
            //writeln!(ctx.console, "SIM OK").unwrap();
        }

        // проверяем регистрацию модуля в сети
        if sim800.check_reg(&mut ctx).is_err() {
            writeln!(ctx.console, "No network").unwrap();
            let cnt = SEC_COUNT_REBOOT.load(Ordering::Relaxed);
            if cnt > 30 * 60 {
                // полчаса нет регистрации в сети - идем в полную перезагрузку
                sim800.reboot(&mut ctx);
                writeln!(ctx.console, "Full reboot").unwrap();
                loop {} // должны перегрузиться по сторожевому таймеру
            } else {
                ctx.delay.delay_ms(3_000_u16);
                continue; // зацикливаемся пока не будет регистрации в сети
            }
        } else {
            SEC_COUNT_REBOOT.store(0, Ordering::Relaxed); // сбрасываем таймер отсутствия регистрации в сети
                                                          //writeln!(ctx.console, "Registered").unwrap();
        }
        ctx.watchdog.feed();

        // проверяем значения датчиков раз в 30 сек
        let cnt2 = SEC_COUNT_SENSOR.load(Ordering::Relaxed);
        if cnt2 > 30 {
            // получить данные о наличии 220в
            v220control.measure();
            // получить данные о температуре чипа
            v220control.measure_temp();
            // температура DS18B20
            let _ = temp_control.measure(&mut ctx);
            // получить строку состояния UPS
            let _ = ups.measure(&mut ctx);
            // получить данные о напряжении батареи МК при напряжении питания 3.3в
            battery.measure();

            SEC_COUNT_SENSOR.store(0, Ordering::Relaxed); // сбрасываем таймер
            ctx.watchdog.feed();
            //writeln!(ctx.console, "temp ret = {0:.1}\n", temp_control.get_temp()).unwrap();
        }

        // проверка диапазонов значений температуры и напряжения
        let _ = v220control.check(&mut ctx, &mut sim800);
        let _ = temp_control.check(&mut ctx, &mut sim800);

        // проверка необходимости отправки ежедневного статус СМС
        let mut need_sms = false;
        if ctx.rtc.wait_alarm().is_ok() {
            writeln!(ctx.console, "Alarm triggered").unwrap();
            need_sms = true;
            ctx.reset_rtc();
        }

        // 10 секунд ждем входящего звонка и при этом мигаем светодиодом
        if sim800.call_detect(&mut ctx).is_ok() {
            writeln!(ctx.console, "Auth call detected!").unwrap();
            need_sms = true;
            ctx.reset_rtc();
        };
        ctx.watchdog.feed();

        // отправка статусного сообщения по СМС
        if need_sms {
            let temp_ext = temp_control.get_temp();
            let temp_int = v220control.get_temp();
            let v220 = v220control.get_voltage();
            let batt = battery.get_voltage();
            let ups_ans = ups.get_ups();

            let mut sms_message: Vec<u8, 160> = Vec::new();
            writeln!(sms_message, "T1={}C", temp_int).unwrap();
            writeln!(sms_message, "T2={0:.1}C", temp_ext).unwrap();
            writeln!(sms_message, "V220={0:.2}V", v220).unwrap();
            writeln!(sms_message, "V12={0:.2}V", batt).unwrap();
            write!(
                sms_message,
                "UPS={}",
                core::str::from_utf8(ups_ans).unwrap()
            )
            .unwrap();

            writeln!(
                ctx.console,
                "SMS={}",
                core::str::from_utf8(&sms_message).unwrap()
            )
            .unwrap();
            let _ = sim800.send_sms(&mut ctx, sms_message.as_slice());
        }
    }
}
