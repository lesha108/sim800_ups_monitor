#![no_std]
#![no_main]

mod context;
mod eeprom;
mod errors;
mod esp01;
mod interrupts;
mod megatec;
mod sensors;
mod sim800l;
mod traits;

use core::fmt::Write;
use core::sync::atomic::Ordering;
use cortex_m_rt::entry;
use ds18b20::*;
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use heapless::*;
use nb::block;
use one_wire_bus::*;
use panic_halt as _;
use stm32f1xx_hal::{
    adc,
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
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

use cortex_m::asm::delay;
use hd44780_driver::*;
use shared_bus::*;
use stm32f1xx_hal::pac::{Interrupt, NVIC};
use stm32f1xx_hal::usb::{Peripheral, UsbBus};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::context::Context;
use crate::eeprom::Eeprom;
use crate::errors::Error;
use crate::esp01::Esp;
use crate::interrupts::*;
use crate::megatec::Ups;
use crate::sensors::{Battery, TempChecker, V220Checker};
use crate::sim800l::{Sim800, SmsType::*};

/// Определение типов данных для сокращёния записи
type AnalogIn1Type = stm32f1xx_hal::gpio::gpiob::PB1<stm32f1xx_hal::gpio::Analog>;
type Adc2Type = adc::Adc<pac::ADC2>;
type AnalogIn0Type = stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>;
type Adc1Type = adc::Adc<pac::ADC1>;
type OwBusType = one_wire_bus::OneWire<
    stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
>;
type Sim800PortType<PINS> = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART3, PINS>;
type Sim800PinType =
    stm32f1xx_hal::gpio::gpioa::PA6<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>;
type Esp01PortType<PINS> = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART2, PINS>;
type Esp01PinType =
    stm32f1xx_hal::gpio::gpioa::PA7<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>;
type UpsPortType<PINS> = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART1, PINS>;

// Определяем входную функцию
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
        .sysclk(48.mhz()) // должна стать 48 для поддержки USB
        .pclk1(24.mhz())
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

    // USART2 - ESP8266
    let serial2 = {
        // Configure pa2 as a push_pull output, this will be the tx pin
        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;
        Serial::usart2(
            dp.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb1,
        )
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

    // настройка USB serial
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low().ok();
    delay(clocks.sysclk().0 / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    // Unsafe to allow access to static variables
    unsafe {
        let bus = UsbBus::new(usb);
        USB_BUS = Some(bus);
        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));
        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
    }

    // таймер для всяких секундных интервалов
    SEC_COUNT_REBOOT.store(0, Ordering::Relaxed);
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    *sec_timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
    // запускаем прерывание по таймеру
    sec_timer.listen(Event::Update);
    unsafe { NVIC::unmask(pac::Interrupt::TIM2) };

    // таймер для отслеживания таймаутов cимволов USART. должен срабатывать через 10 мс
    let at_timer = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(100.hz());

    // Create a delay timer from the RCC clocks
    let mut delay = Delay::new(cp.SYST, clocks);

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

    // I2C setup for EEPROM & LCD
    let bus_i2c = {
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
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
        );
        BusManagerSimple::new(i2c)
    };

    // Настройка работы чипа EEPROM
    let eeprom = {
        let address = SlaveAddr::default();
        Eeprom24x::new_24x16(bus_i2c.acquire_i2c(), address) // на модуле arduino 24x32
    };

    const I2C_ADDRESS: u8 = 0x27;
    let mut lcd = HD44780::new_i2c(bus_i2c.acquire_i2c(), I2C_ADDRESS, &mut delay).unwrap();
    lcd.reset(&mut delay).ok();
    lcd.clear(&mut delay).ok();
    lcd.set_display_mode(
        DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Visible,
            cursor_blink: CursorBlink::On,
        },
        &mut delay,
    ).ok();
    lcd.write_bytes(b"Booting...", &mut delay).ok();

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
    let mut reboot_count: u8 = 0; // число попыток восстановления связи с SIM800

    // пин связан с контактом reset ESP01. Притягивание к земле перегружает модуль
    let mut esp01 = {
        let esp_reset_pin = gpioa
            .pa7
            .into_open_drain_output_with_state(&mut gpioa.crl, stm32f1xx_hal::gpio::State::High);
        Esp::new(serial2, esp_reset_pin)
    };

    // создаём переменную с контекстом, которую будем передавать в другие функции
    let mut ctx = Context {
        watchdog: watchdog, // сторожевой таймер
        delay: delay,       // функции задержки
        at_timer: at_timer, // таймер отслеживания таймаутов в последовательных портах
        rtc: rtc,           // часы реального времени
        eeprom: eeprom,     // доступ к EEPROM
        beeper: beeper,     // функции пищалки
        led: led,           // heartbeat LED
    };
    // Гудок при старте контроллера
    ctx.beep();

    // объект для взаимодействия с UPS
    let mut ups = Ups::new(serial1);
    if ups.measure(&mut ctx).is_err() {
        // первый вызов корректно инициализирует буфер
        write_log(b"UPS error");
    }
    ctx.watchdog.feed();

    // объект для взаимодействия с батареей МК
    let mut battery = Battery::new(ch1, adc2);
    battery.measure();
    ctx.watchdog.feed();

    // объект контроля 220в
    let mut v220control = V220Checker::new(ch0, adc1);
    let _ = v220control.load(&mut ctx);
    v220control.measure();
    v220control.measure_temp();
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
    let mut temp_control = TempChecker::new(one_wire_bus);
    let _ = temp_control.load(&mut ctx);
    temp_control.measure(&mut ctx);

    // ожидание СМС с балансом счёта сим карты
    let mut balance_wait = false;
    loop {
        // сбрасываем сторожевой таймер
        ctx.watchdog.feed();
        // проверяем связь с SIM800L Если нет, то пробуем перегрузить SIM800
        // если не помогает перезагрузка, пробуем перегрузить микроконтроллер
        let chk = sim800.check_com(&mut ctx);
        if !chk {
            write_log(b"SIM com failed. SIM800 Reboot");
            sim800.reboot(&mut ctx);
            for _ in 0..10 {
                ctx.delay.delay_ms(3_000_u16);
                ctx.watchdog.feed();
            }
            // если не помогает перегрузка модуля, делаем ребут всего контроллера
            if reboot_count > 3 {
                write_log(b"Self reboot");
                loop {} // должны перегрузиться по сторожевому таймеру
            } else {
                reboot_count += 1;
            }
            continue; // зацикливаемся пока не будет связи с SIM800
        } else {
            reboot_count = 0;
        }

        // проверяем регистрацию модуля в сети
        let reg_chk = sim800.check_reg(&mut ctx);
        if !reg_chk {
            write_log(b"No network");
            let cnt = SEC_COUNT_REBOOT.load(Ordering::Relaxed);
            if cnt > 30 * 60 {
                // полчаса нет регистрации в сети - идем в полную перезагрузку
                sim800.reboot(&mut ctx);
                write_log(b"Full reboot");
                loop {} // должны перегрузиться по сторожевому таймеру
            } else {
                ctx.delay.delay_ms(3_000_u16);
                continue; // зацикливаемся пока не будет регистрации в сети
            }
        } else {
            SEC_COUNT_REBOOT.store(0, Ordering::Relaxed); // сбрасываем таймер отсутствия регистрации в сети
        }
        ctx.watchdog.feed();

        // проверяем связь с ESP Если нет, то пробуем перегрузить ESP
        // связи не должно быть, если модуль не может к WiFi подключиться
        // проблемы железа не проверяются!
        let chk = esp01.check_com(&mut ctx);
        if !chk {
            let cnt = SEC_COUNT_ESP.load(Ordering::Relaxed);
            if cnt > 10 * 60 {
                // 10 минут ESP не отвечает
                write_log(b"ESP01 com failed. Reboot");
                esp01.reboot(&mut ctx);
                SEC_COUNT_ESP.store(0, Ordering::Relaxed);
            }
        } else {
            SEC_COUNT_ESP.store(0, Ordering::Relaxed); // сбрасываем таймер отсутствия связи с ESP
        }
        ctx.watchdog.feed();

        // проверяем значения датчиков раз в 30 сек
        let cnt2 = SEC_COUNT_SENSOR.load(Ordering::Relaxed);
        if cnt2 > 30 {
            v220control.measure(); // получить данные о наличии 220в
            v220control.measure_temp(); // получить данные о температуре чипа
            temp_control.measure(&mut ctx); // тепература DS18B20
            if ups.measure(&mut ctx).is_err() {
                // получить строку состояния UPS
                write_log(b"UPS error");
            }
            // получить данные о напряжении батареи МК при напряжении питания 3.3в
            battery.measure();

            SEC_COUNT_SENSOR.store(0, Ordering::Relaxed); // сбрасываем таймер
            ctx.watchdog.feed();

            // обновляем инфо на экране LCD
            lcd.reset(&mut ctx.delay).ok();
            lcd.clear(&mut ctx.delay).ok();
            lcd.set_display_mode(
                DisplayMode {
                    display: Display::On,
                    cursor_visibility: Cursor::Visible,
                    cursor_blink: CursorBlink::On,
                },
                &mut ctx.delay,
            ).ok();
            let temp_ext = temp_control.get_temp();
            let mut line: Vec<u8, 20> = Vec::new();
            write!(line, "Temp={0:.1}C", temp_ext).unwrap();
            lcd.write_bytes(&line, &mut ctx.delay).ok();
            // переходим на всторую строку
            lcd.set_cursor_pos(40, &mut ctx.delay).ok();
            line.clear();
            let datetime = sim800.clk(&mut ctx).unwrap_or_else(|_| {
                let mut err: Vec<u8, 60> = Vec::new();
                err.extend_from_slice(b"Bad date").unwrap();
                err
            });
            lcd.write_bytes(&datetime, &mut ctx.delay).ok();        
        }
        // проверка диапазонов значений температуры и напряжения
        //v220control.check(&mut ctx, &mut sim800);
        //temp_control.check(&mut ctx, &mut sim800);
        ctx.check(&mut v220control, &mut sim800).ok();
        ctx.check(&mut temp_control, &mut sim800).ok();

        // проверка необходимости отправки ежедневного статус СМС
        let mut need_sms = false;
        if ctx.rtc.wait_alarm().is_ok() {
            write_log(b"Alarm triggered");
            need_sms = true;
            ctx.reset_rtc();
            // пробуем раз в сутки синхронизировать часы через NTP
            sim800.sync_clk(&mut ctx).ok();
            sim800.sync_clk_close(&mut ctx).ok();
        }

        // 10 секунд ждем входящего звонка и при этом мигаем светодиодом
        if sim800.call_detect(&mut ctx).is_ok() {
            write_log(b"Auth call detected!");
            need_sms = true;
            ctx.reset_rtc();
        };

        // обрабатываем команды во входящих SMS
        if !balance_wait {
            let sms_r = sim800.sms_detect(&mut ctx);
            match sms_r {
                Err(Error::NotAuthCall) => {
                    write_log(b"Not auth SMS detected!");
                }
                Err(_) => {}
                Ok(command) => {
                    // команда перезагрузки МК по сторожевому таймеру
                    if command.starts_with(b"Boot") {
                        write_log(b"Boot command!");
                        loop {}
                    }
                    // команда отсылки статусного СМС, но
                    // без сброса суточного таймера как при звонке
                    if command.starts_with(b"Status") {
                        write_log(b"Status command!");
                        need_sms = true;
                    }
                    // команда проверки связи с удаленным реле на ESP-01
                    if command.starts_with(b"Ping") {
                        write_log(b"PING command!");
                        let ping_result = esp01.esp_ping(&mut ctx);
                        match ping_result {
                            Err(_) => {
                                let error = esp01.get_error();
                                sim800.send_sms(&mut ctx, error, Normal).ok();
                                write_log(b"PING ERROR:");
                                write_log(error);
                            }
                            Ok(_) => {
                                sim800.send_sms(&mut ctx, b"PING OK", Normal).ok();
                                write_log(b"PING OK");
                            }
                        }
                        ctx.beep();
                    }
                    // команда переключения удаленного реле на ESP-01
                    if command.starts_with(b"Reset") {
                        write_log(b"RESET command!");
                        let reset_result = esp01.esp_reset(&mut ctx);
                        match reset_result {
                            Err(_) => {
                                let error = esp01.get_error();
                                sim800.send_sms(&mut ctx, error, Normal).ok();
                                write_log(b"RESET ERROR:");
                                write_log(error);
                            }
                            Ok(_) => {
                                sim800.send_sms(&mut ctx, b"RESET OK", Normal).ok();
                                write_log(b"RESET OK");
                            }
                        }
                        ctx.beep();
                    }
                    // команда запроса баланса сим карты МТС
                    if command.starts_with(b"Balance") {
                        // проверка баланса сим карты МТС - СМС 11 на номер 111
                        write_log(b"Balance command!");
                        sim800.send_sms(&mut ctx, b"11", Balance).ok();
                        // сбрасываем таймер ожидания СМС с балансом
                        SEC_COUNT_SENSOR.store(0, Ordering::Relaxed);
                        balance_wait = true;
                        ctx.beep();
                    }
                    // команда синхронизации времени с МТС
                    if command.starts_with(b"Clk") {
                        write_log(b"Clk command!");
                        let sync_result = sim800.sync_clk(&mut ctx);
                        sim800.sync_clk_close(&mut ctx).ok();
                        match sync_result {
                            Err(_) => {
                                sim800.send_sms(&mut ctx, b"Sync failed", Normal).ok();
                                write_log(b"Sync failed");
                            }
                            Ok(_) => {
                                sim800.send_sms(&mut ctx, b"Sync OK", Normal).ok();
                                write_log(b"Sync OK");
                            }
                        }
                        ctx.beep();
                    }
                }
            }
        }
        ctx.watchdog.feed();

        // ждем СМС с балансом счёта сим карты после команды Balance
        if balance_wait {
            balance_wait = false;
            let sms_b = sim800.balance_sms_detect(&mut ctx);
            match sms_b {
                Err(Error::NoInSMS) => {
                    write_log(b"No balance SMS detected!");
                    let bal_cnt = SEC_COUNT_SENSOR.load(Ordering::Relaxed);
                    // продолжаем ждать 3 минуты СМС с балансом, потом в нормальный режим
                    if bal_cnt <= 60 * 3 {
                        balance_wait = true;
                    }
                }
                Err(Error::NoUCS2) => {
                    write_log(b"Balance not in UCS2!");
                }
                Err(Error::InvalidUCS2Size) => {
                    write_log(b"Balance UCS2 invalid!");
                }
                Err(_) => {}
                Ok(balance) => {
                    let mut sms_message: Vec<u8, 160> = Vec::new();
                    write!(
                        sms_message,
                        "Balance:{}",
                        core::str::from_utf8(&balance).unwrap()
                    )
                    .unwrap();
                    sim800
                        .send_sms(&mut ctx, sms_message.as_slice(), Normal)
                        .ok();
                }
            }
        }
        ctx.watchdog.feed();

        // отправка статусного сообщения по СМС
        if need_sms {
            let temp_ext = temp_control.get_temp();
            let temp_int = v220control.get_temp();
            let v220 = v220control.get_voltage();
            let batt = battery.get_voltage();
            let ups_ans = ups.get_ups();
            let datetime = sim800.clk(&mut ctx).unwrap_or_else(|_| {
                let mut err: Vec<u8, 60> = Vec::new();
                err.extend_from_slice(b"Bad date").unwrap();
                err
            });

            let mut sms_message: Vec<u8, 160> = Vec::new();
            writeln!(sms_message, "T1={}C", temp_int).unwrap();
            writeln!(sms_message, "T2={0:.1}C", temp_ext).unwrap();
            writeln!(sms_message, "V220={0:.2}V", v220).unwrap();
            writeln!(sms_message, "V12={0:.2}V", batt).unwrap();
            write!(
                sms_message,
                "UPS={}\n {}",
                core::str::from_utf8(&ups_ans).unwrap(),
                core::str::from_utf8(&datetime).unwrap()
            )
            .unwrap();
            sim800
                .send_sms(&mut ctx, sms_message.as_slice(), Normal)
                .ok();
        }
    }
}

// глобальная функция логирования в последовательный порт USB
pub fn write_log(to_write: &[u8]) {
    cortex_m::interrupt::free(|_| {
        let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
        serial.write(to_write).ok();
    });
}

// поиск подстроки в буфере
pub fn buf_contains(buffer: &[u8], pattern: &[u8]) -> bool {
    let psize = pattern.len();
    let bsize = buffer.len();
    for i in 0..bsize {
        let rlimit = i + psize;
        if rlimit > bsize {
            break;
        }
        let sl = &buffer[i..rlimit];
        if sl == pattern {
            return true;
        }
    }
    false
}
