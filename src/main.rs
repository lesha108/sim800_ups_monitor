#![no_std]
#![no_main]
#![warn(clippy::pedantic)]

mod context;
mod interrupts;
mod sim;
mod traits;
mod ups;

use stm32f1xx_hal::prelude::*;
//
use core::fmt::Write;
use core::sync::atomic::Ordering;
use cortex_m_rt::entry;
use ds18b20::*;
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::blocking::delay::DelayMs;
use heapless::*;
use one_wire_bus::*;
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
use crate::interrupts::{SEC_COUNT_REBOOT, SEC_COUNT_SENSOR, SEC_TIMER};
use crate::sim::Sim800;
use crate::traits::Observable;
use crate::ups::Ups;

/// Определяем входную функцию
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
            Config::default().baudrate(115200.bps()),
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
        watchdog, // сторожевой таймер
        delay,    // функции задержки
        at_timer, // таймер отслеживания таймаутов в последовательных портах
        rtc,      // часы реального времени
        console,  // последовательный порт отладочной печати
        eeprom,   // доступ к EEPROM
        beeper,   // функции пищалки
        led,      // heartbeat LED
    };
    // Гудок при старте контроллера
    ctx.beep();

    let mut reboot_count: u8 = 0; // число попыток восстановления связи с SIM800

    //

    // объект для взаимодействия с UPS
    let mut ups = Ups::new(serial1);
    ups.measure(&mut ctx); // первый вызов корректно инициализирует буфер
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
    temp_control.measure(&mut ctx);

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
            v220control.measure(); // получить данные о наличии 220в
            v220control.measure_temp(); // получить данные о температуре чипа
            temp_control.measure(&mut ctx); // тепература DS18B20
            ups.measure(&mut ctx); // получить строку состояния UPS
                                   // получить данные о напряжении батареи МК при напряжении питания 3.3в
            battery.measure();

            SEC_COUNT_SENSOR.store(0, Ordering::Relaxed); // сбрасываем таймер
            ctx.watchdog.feed();
            //writeln!(ctx.console, "temp ret = {0:.1}\n", temp_control.get_temp()).unwrap();
        }

        // проверка диапазонов значений температуры и напряжения
        v220control.check(&mut ctx, &mut sim800);
        temp_control.check(&mut ctx, &mut sim800);

        // проверка необходимости отправки ежедневного статус СМС
        let mut need_sms = false;
        let alarm = ctx.rtc.wait_alarm();
        match alarm {
            Err(nb::Error::Other(_)) => {}
            Err(nb::Error::WouldBlock) => {}
            Ok(_) => {
                writeln!(ctx.console, "Alarm triggered").unwrap();
                need_sms = true;
                ctx.reset_rtc();
            }
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
            write!(sms_message, "T1={}C\n", temp_int).unwrap();
            write!(sms_message, "T2={0:.1}C\n", temp_ext).unwrap();
            write!(sms_message, "V220={0:.2}V\n", v220).unwrap();
            write!(sms_message, "V12={0:.2}V\n", batt).unwrap();
            write!(
                sms_message,
                "UPS={}",
                core::str::from_utf8(&ups_ans).unwrap()
            )
            .unwrap();
            sim800.send_sms(&mut ctx, sms_message.as_slice());
            writeln!(
                ctx.console,
                "SMS={}",
                core::str::from_utf8(&sms_message).unwrap()
            )
            .unwrap();
        }
    }
}

//--------------------------------------------------------------
// Функции отслеживания состояний датчиков 220в и температуры
//--------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq)]
enum EepromAdresses {
    V220State,
    // состояние датчика напряжения 220 в
    TempState,
    // состояние датчика температуры DS18B20
    Number, // телефонный номер отправки SMS
}

// возвращает адрес сохраненного значения в чипе EEPROM
impl EepromAdresses {
    fn address(self) -> u8 {
        match self {
            EepromAdresses::V220State => 0x01,
            EepromAdresses::TempState => 0x02,
            EepromAdresses::Number => 0x04,
        }
    }
}

// структура для отслеживания напряжения 220в питания МК

// константы подбираются экспериментально!!!
const VCC: f32 = 3.3;
// напряжение питания МК
const V220_START: f32 = 1.5;
// значение ADC, означающее приемлемое напряжение питания
const V220_LIMIT_LOW: f32 = 0.3; // пороговое значение ADC, означающее отсутствие питания

#[derive(PartialEq)]
enum V220States {
    ColdStart,
    // инициализация измерений
    Monitoring,
    // ожидание падения напряжения 220 в
    WaitForNormal, // ожидание восстановления 220 в
}

struct V220Control {
    state: V220States,
    address: u8,
    voltage: f32,
    int_temp: i32,
    analog_input: stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>,
    adc: adc::Adc<pac::ADC1>,
}

impl V220Control {
    fn new(
        pin: stm32f1xx_hal::gpio::gpiob::PB0<stm32f1xx_hal::gpio::Analog>,
        adc1: adc::Adc<pac::ADC1>,
    ) -> V220Control {
        let a = EepromAdresses::V220State;
        V220Control {
            state: V220States::ColdStart,
            address: a.address(),
            voltage: 0.0,
            int_temp: 0,
            analog_input: pin,
            adc: adc1,
        }
    }

    // восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) -> bool {
        if self.state != V220States::ColdStart {
            return true;
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        match read_data {
            0 => self.state = V220States::ColdStart,
            1 => self.state = V220States::Monitoring,
            2 => self.state = V220States::WaitForNormal,
            _ => self.state = V220States::ColdStart,
        }
        true
    }

    // запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let data = {
            match self.state {
                V220States::ColdStart => 0,
                V220States::Monitoring => 1,
                V220States::WaitForNormal => 2,
            }
        };
        ctx.save_byte(self.address as u32, data)
    }

    // измерение напряжения 220 в
    fn measure(&mut self) -> f32 {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap(); //3.3d = 4095, 3.3/2 = 2036
            averaged += data as u32;
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
        self.voltage
    }

    fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    // измерение температуры чипа
    fn measure_temp(&mut self) -> i32 {
        self.int_temp = self.adc.read_temp();
        self.int_temp
    }

    fn get_temp(&mut self) -> i32 {
        self.int_temp
    }
}

impl Observable for V220Control {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            V220States::ColdStart => {
                self.measure();
                self.state = V220States::Monitoring;
            }
            V220States::Monitoring => {
                if self.voltage < V220_LIMIT_LOW {
                    // сетевое напряжение пропало
                    sim.send_sms(ctx, b"220 failed");
                    writeln!(ctx.console, "220 failed {}", self.voltage).unwrap();
                    ctx.beep();

                    self.state = V220States::WaitForNormal;
                    self.save(ctx);
                }
            }
            V220States::WaitForNormal => {
                if self.voltage > V220_START {
                    // сетевое напряжение вернулось
                    sim.send_sms(ctx, b"220 on-line");
                    writeln!(ctx.console, "220 on-line").unwrap();
                    ctx.beep();

                    self.state = V220States::Monitoring;
                    self.save(ctx);
                }
            }
        }
        Ok(())
    }
}

// структура для отслеживания диапазона датчика температуры DS18B20

const TEMP_LIMIT_HIGH: f32 = 20.0;
// приемлемое значение температуры
const TEMP_LIMIT_LOW: f32 = 15.0; // пороговое значение, означающее критическое понижение температуры

#[derive(PartialEq)]
enum TemperatureStates {
    ColdStart,
    // инициализация измерений
    Monitoring,
    // ожидание выхода температуры за границы контроля
    WaitForNormal, // ожидание восстановления нормальной температуры
}

struct TempControl {
    state: TemperatureStates,
    address: u8,
    temp: f32,
    one_wire_bus: one_wire_bus::OneWire<
        stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
    >,
}

impl TempControl {
    fn new(
        bus: one_wire_bus::OneWire<
            stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
        >,
    ) -> TempControl {
        let a = EepromAdresses::TempState;
        TempControl {
            state: TemperatureStates::ColdStart,
            address: a.address(),
            temp: 0.0,
            one_wire_bus: bus,
        }
    }

    // восстановление состояния из EEPROM после перезагрузки МК
    fn load(&mut self, ctx: &mut Context) -> bool {
        if self.state != TemperatureStates::ColdStart {
            return true;
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        match read_data {
            0 => self.state = TemperatureStates::ColdStart,
            1 => self.state = TemperatureStates::Monitoring,
            2 => self.state = TemperatureStates::WaitForNormal,
            _ => self.state = TemperatureStates::ColdStart,
        }
        true
    }

    // запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> Result<(), ()> {
        let data = {
            match self.state {
                TemperatureStates::ColdStart => 0,
                TemperatureStates::Monitoring => 1,
                TemperatureStates::WaitForNormal => 2,
            }
        };
        ctx.save_byte(self.address as u32, data)
    }

    // измерение температуры
    fn measure(&mut self, ctx: &mut Context) -> f32 {
        let w1 =
            ds18b20::start_simultaneous_temp_measurement(&mut self.one_wire_bus, &mut ctx.delay);
        match w1 {
            Err(_) => {
                writeln!(ctx.console, "OW e1").unwrap();
                return -101.0;
            }
            Ok(_) => {
                Resolution::Bits12.delay_for_measurement_time(&mut ctx.delay);
                let mut search_state = None;
                let w2 =
                    self.one_wire_bus
                        .device_search(search_state.as_ref(), false, &mut ctx.delay);
                match w2 {
                    Err(_) => {
                        writeln!(ctx.console, "OW e2").unwrap();
                        return -102.0;
                    }
                    Ok(None) => {
                        writeln!(ctx.console, "OW none").unwrap();
                        return -103.0;
                    }
                    Ok(Some((device_address, state))) => {
                        search_state = Some(state); // у нас только один датчик, дальше не ищем
                        if device_address.family_code() == ds18b20::FAMILY_CODE {
                            let w0: core::result::Result<
                                ds18b20::Ds18b20,
                                one_wire_bus::OneWireError<core::convert::Infallible>,
                            > = Ds18b20::new(device_address);
                            match w0 {
                                Err(_) => {
                                    writeln!(ctx.console, "OW e5").unwrap();
                                    return -104.0;
                                }
                                Ok(sensor) => {
                                    let w3 =
                                        sensor.read_data(&mut self.one_wire_bus, &mut ctx.delay);
                                    match w3 {
                                        Err(_) => {
                                            writeln!(ctx.console, "OW e4").unwrap();
                                            return -105.0;
                                        }
                                        Ok(sensor_data) => {
                                            //writeln!(console, "Device at {:?} is {}C", device_address, sensor_data.temperature);
                                            self.temp = sensor_data.temperature;
                                            return self.temp;
                                        }
                                    }
                                }
                            }
                        } else {
                            writeln!(ctx.console, "OW e3").unwrap();
                            return -106.0;
                        }
                    }
                }
            }
        }
    }

    fn get_temp(&mut self) -> f32 {
        self.temp
    }
}

impl Observable for TempControl {
    fn check<PINS>(&mut self, ctx: &mut Context, sim: &mut Sim800<PINS>) -> Result<(), ()> {
        match self.state {
            TemperatureStates::ColdStart => {
                self.measure(ctx);
                self.state = TemperatureStates::Monitoring;
            }
            TemperatureStates::Monitoring => {
                if self.temp > -99.0 && self.temp < TEMP_LIMIT_LOW {
                    // температура упала
                    sim.send_sms(ctx, b"Temp too low");
                    writeln!(ctx.console, "Temp too low {}", self.temp).unwrap();
                    ctx.beep();

                    self.state = TemperatureStates::WaitForNormal;
                    self.save(ctx);
                }
            }
            TemperatureStates::WaitForNormal => {
                if self.temp > TEMP_LIMIT_HIGH {
                    // температура вернулось в норму
                    sim.send_sms(ctx, b"Temp OK");
                    writeln!(ctx.console, "Temp OK").unwrap();
                    ctx.beep();

                    self.state = TemperatureStates::Monitoring;
                    self.save(ctx);
                }
            }
        }
        Ok(())
    }
}

// структура для отслеживания напряжения батареи питания МК

struct Battery {
    voltage: f32,
    analog_input: stm32f1xx_hal::gpio::gpiob::PB1<stm32f1xx_hal::gpio::Analog>,
    adc: adc::Adc<pac::ADC2>,
}

impl Battery {
    fn new(
        pin: stm32f1xx_hal::gpio::gpiob::PB1<stm32f1xx_hal::gpio::Analog>,
        adc2: adc::Adc<pac::ADC2>,
    ) -> Battery {
        Battery {
            voltage: 0.0,
            analog_input: pin,
            adc: adc2,
        }
    }

    fn get_voltage(&mut self) -> f32 {
        self.voltage
    }

    // измерение напряжения батареи
    fn measure(&mut self) -> f32 {
        let mut averaged: u32 = 0;
        for _ in 0..8 {
            let data: u16 = self.adc.read(&mut self.analog_input).unwrap(); //3.3d = 4095, 3.3/2 = 2036
            averaged += data as u32;
        }
        self.voltage = averaged as f32 / 8.0 / 4096.0 * VCC;
        self.voltage
    }
}
