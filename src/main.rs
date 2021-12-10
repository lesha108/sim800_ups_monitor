#![no_std]
#![no_main]

use panic_halt as _;
//use core::convert::Infallible;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{
    prelude::*,
    pac,
    pac::interrupt,
    adc,
    rtc::Rtc,
    timer::{Timer, Event},
    time::MilliSeconds,
    pwm::Channel,
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    serial::{Config, Serial},
    watchdog::IndependentWatchdog,
};
use nb::block;
use heapless::*;
use eeprom24x::{Eeprom24x, SlaveAddr};
use one_wire_bus::*;
use ds18b20::*;
//use cortex_m::asm;
//use cortex_m_rt::ExceptionFrame;  //  Stack frame for exception handling.
//use cortex_m_semihosting::hio;  //  For displaying messages on the debug console.

// Статические переменные для работы с прерываеним таймера
static mut SEC_TIMER: MaybeUninit<stm32f1xx_hal::timer::CountDownTimer<pac::TIM2>> =
    MaybeUninit::uninit();
static SEC_COUNT_REBOOT: AtomicU32 = AtomicU32::new(0);
static SEC_COUNT_SENSOR: AtomicU32 = AtomicU32::new(0);
static SEC_COUNT_LED: AtomicU32 = AtomicU32::new(0);

// Определение структуры аппаратного контекста
struct Context {
    watchdog: IndependentWatchdog,
    delay: stm32f1xx_hal::delay::Delay,
    at_timer: stm32f1xx_hal::timer::CountDownTimer<stm32f1xx_hal::pac::TIM3>,
    rtc: Rtc,
    console: stm32f1xx_hal::serial::Tx<stm32f1xx_hal::pac::USART2>,
    led: stm32f1xx_hal::gpio::gpioc::PC13<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,

    eeprom: eeprom24x::Eeprom24x<stm32f1xx_hal::i2c::BlockingI2c<stm32f1xx_hal::pac::I2C1,
     (stm32f1xx_hal::gpio::gpiob::PB6<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>,
     stm32f1xx_hal::gpio::gpiob::PB7<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>,
     eeprom24x::page_size::B16, eeprom24x::addr_size::OneByte>,

    beeper: stm32f1xx_hal::pwm::Pwm<stm32f1xx_hal::pac::TIM1,
     stm32f1xx_hal::timer::Tim1NoRemap, stm32f1xx_hal::pwm::C1,
     stm32f1xx_hal::gpio::gpioa::PA8<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>>,
}

impl Context {
    fn beep(&mut self) {
        self.beeper.set_duty(Channel::C1, self.beeper.get_max_duty()/2);    
        self.watchdog.feed();            
        self.delay.delay_ms(500_u16);
        self.beeper.set_duty(Channel::C1, 0);
        self.watchdog.feed();
    }

    fn save_byte(&mut self, address: u32, data: u8) -> bool {
        self.eeprom.write_byte(address, data).unwrap();
        self.delay.delay_ms(10_u16);
        let read_data = self.eeprom.read_byte(address).unwrap();
        if read_data == data { 
            return true 
        } else {
            return false
        }
    } 

    fn reset_rtc(&mut self) {
        self.rtc.set_time(0);
        // Запустить через сутки
        self.rtc.set_alarm(24*60*60);
    }

    fn check<O: Observable>(&mut self, control: &mut O, sim: &mut Sim800) -> bool {
        control.check(self, sim)
    }
}

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

    let clocks = rcc.cfgr
                    .use_hse(8.mhz()) // переключились на кварц
                    .sysclk(16.mhz()) // должна стать 16
                    .adcclk(2.mhz())  // для ADC
                    .freeze(&mut flash.acr);
    // Prepare the alternate function I/O registers
    let mut afio  = dp.AFIO.constrain(&mut rcc.apb2);
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
            &mut rcc.apb2)
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
            &mut rcc.apb1);
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
            &mut rcc.apb1)
    };

    // таймер для всяких секундных интервалов
    SEC_COUNT_REBOOT.store(0, Ordering::Relaxed);
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    *sec_timer = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
                        .start_count_down(1.hz());
    // запускаем прерываение по таймеру
    sec_timer.listen(Event::Update);
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIM2) };

    // таймер для отслеживания таймаутов cимволов USART. должен срабатывать через 10 мс
    let at_timer = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1)
                        .start_count_down(100.hz());

    // Create a delay timer from the RCC clocks
    let delay = Delay::new(cp.SYST, clocks);

    // ШИМ канал для работы пищалки
    let beeper = {
        // Configure pa8 as a push_pull output
        let pa8 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        // выдаём 1 кГц на этот пин
        let mut pwm = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).pwm(pa8, &mut afio.mapr, 1.khz());
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
    let mut iwdg = IndependentWatchdog::new(dp.IWDG);
    // запускаем с 20 сек интервалом сброса
    iwdg.start(MilliSeconds(20_000));

    // пин связан с контактом reset SIM800. Притягивание к земле перегружает модуль
    let mut sim800 = {
        let sim_reset_pin = gpioa.pa6.into_open_drain_output_with_state(&mut gpioa.crl,
            stm32f1xx_hal::gpio::State::High);
        Sim800::new(serial3, sim_reset_pin)
    };
    let mut reboot_count: u8 = 0; // число попыток восстановления связи с SIM800

    // создаём переменную с контекстом, которую будем передавать в другие функции
    let mut ctx = Context {
        watchdog: iwdg, // сторожевой таймер
        delay: delay, // функции задержки
        at_timer: at_timer, // таймер отслеживания таймаутов в последовательных портах
        rtc: rtc, // часы реального времени
        console: console, // последовательный порт отладочной печати
        eeprom: eeprom, // доступ к EEPROM 
        beeper: beeper, // функции пищалки
        led: led, // heartbeat LED
    };
    // Гудок при старте контроллера
    ctx.beep();

    // объект для взаимодействия с UPS
    let mut ups = Ups::new(serial1);
    ups.measure(&mut ctx); // первый вызов корректно инициализирует буфер
    ctx.watchdog.feed();

    // объект для взаимодействия с батареей МК
    let mut battery = Battery::new(ch1, adc2);
    battery.measure(); 
    ctx.watchdog.feed();

    // объект контроля 220в
    let mut v220control = V220Control::new(ch0, adc1); 
    v220control.load(&mut ctx);
    v220control.measure();
    v220control.measure_temp();
    ctx.watchdog.feed();

    // OneWire DS18b20 line
    let one_wire_bus = {
        let one_wire_pin = gpiob.pb12.into_open_drain_output(&mut gpiob.crh).downgrade();
        OneWire::new(one_wire_pin).unwrap()
    };
    // объект контроля темперетуры DS18B20
    let mut temp_control = TempControl::new(one_wire_bus); 
    temp_control.load(&mut ctx);
    temp_control.measure(&mut ctx);
    
    loop {
        ctx.watchdog.feed(); // сбрасываем сторожевой таймер
        // проверяем связь с SIM800L Если нет, то пробуем перегрузить SIM800
        // если не помогает перезагрузка, пробуем перегрузить микроконтроллер
        let chk = sim800.check_com(&mut ctx);
        if !chk {
            writeln!(ctx.console, "SIM com failed. SIM800 Reboot").unwrap();
            sim800.reboot(&mut ctx);
            for _ in 0..10 {
                ctx.delay.delay_ms(3_000_u16);
                ctx.watchdog.feed();
            }
            // если не помогает перегрузка модуля, делаем ребут всего контроллера
            if reboot_count > 3 {
                writeln!(ctx.console, "Self reboot").unwrap();
                loop {}; // должны перегрузиться по сторожевому таймеру
            } else {
                reboot_count += 1;
            }
            continue; // зацикливаемся пока не будет связи с SIM800
        } else {
            reboot_count = 0;
            //writeln!(ctx.console, "SIM OK").unwrap();
        }

        // проверяем регистрацию модуля в сети
        let reg_chk = sim800.check_reg(&mut ctx);
        if !reg_chk {
            writeln!(ctx.console, "No network").unwrap();
            let cnt = SEC_COUNT_REBOOT.load(Ordering::Relaxed);
            if cnt > 30*60 { // полчаса нет регистрации в сети - идем в полную перезагрузку
                sim800.reboot(&mut ctx);
                writeln!(ctx.console, "Full reboot").unwrap();
                loop {}; // должны перегрузиться по сторожевому таймеру
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
        //v220control.check(&mut ctx, &mut sim800);
        //temp_control.check(&mut ctx, &mut sim800);
        ctx.check(&mut v220control,  &mut sim800);
        ctx.check(&mut temp_control, &mut sim800);
        
        // проверка необходимости отправки ежедневного статус СМС
        let mut need_sms = false;
        let alarm = ctx.rtc.wait_alarm();
        match alarm {
            Err(nb::Error::Other(_)) => { }
            Err(nb::Error::WouldBlock) => { }
            Ok(_) => {
                        writeln!(ctx.console, "Alarm triggered").unwrap();
                        need_sms = true;
                        ctx.reset_rtc();
                    }
        }

        // 10 секунд ждем входящего звонка и при этом мигаем светодиодом
        if sim800.call_detect(&mut ctx) {
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
            write!(sms_message, "UPS={}", core::str::from_utf8(&ups_ans).unwrap()).unwrap();
            sim800.send_sms(&mut ctx, sms_message.as_slice());
            writeln!(ctx.console, "SMS={}", core::str::from_utf8(&sms_message).unwrap()).unwrap();
        }
    }
}

#[interrupt]
fn TIM2() {
    let sec_timer = unsafe { &mut *SEC_TIMER.as_mut_ptr() };
    SEC_COUNT_REBOOT.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_SENSOR.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_LED.fetch_add(1, Ordering::Relaxed);
    sec_timer.clear_update_interrupt_flag();
}

//--------------------------------------------------------------
// Функции работы с UPS
//--------------------------------------------------------------
const UPS_BUF_LEN: usize = 50; // длина буфера приёма статуса UPS
const TIMEOUT_FIRST_CYCLES: u16 = 200; // wait first char for 2 sec
const TIMEOUT_LAST_CYCLES: u16 = 10; // wait finish char for 100 ms

struct Ups {
    snd_buf: Vec<u8, 4>,
    rcv_buf: Vec<u8, UPS_BUF_LEN>,
    ups_port: stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART1,
     (stm32f1xx_hal::gpio::gpioa::PA9<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
     stm32f1xx_hal::gpio::gpioa::PA10<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>,
}

impl Ups {
    fn new(
        port: stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART1,
        (stm32f1xx_hal::gpio::gpioa::PA9<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
        stm32f1xx_hal::gpio::gpioa::PA10<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>
    ) -> Ups {
        Ups { 
            snd_buf: Vec::new(),
            rcv_buf: Vec::new(),
            ups_port: port,
        }
    }

    fn get_ups(&mut self) -> &[u8] {
        // должно быть что-то типа 
        // (218.1 218.1 219.6 000 50.0 2.22 48.0 00000001\r
        // вырезаем первый и последний символы
        &self.rcv_buf[1..self.rcv_buf.len()-1] // !!! проблема, если буфер пуст, т.е. не вызвали measure
    }

    fn measure(&mut self, ctx: &mut Context) ->  bool { 
        // отправляем команду получения статуса UPS
        self.snd_buf.clear();
        self.snd_buf.extend_from_slice(b"Q1\r").unwrap();
        for cmd in &self.snd_buf {
            block!(self.ups_port.write(*cmd)).ok();
        }
        // пробуем получить ответ
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        loop {
            let res = self.ups_port.read();
            match res {
                 Err(nb::Error::Other(_)) => {
                    writeln!(ctx.console, "UPS e1").unwrap();
                    break;
                    }
                 Err(nb::Error::WouldBlock) => { // символ не пришёл ещё
                      let t = ctx.at_timer.wait();
                      match t {
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "UPS e2").unwrap();
                            break;
                            }
                        Err(nb::Error::WouldBlock) => { // просто ждём ещё таймер
                            continue;
                        }
                        Ok(_) => { // сработал таймер отсчёта таймаута
                            if got_first_char { // отрабатываем ожидание последнего символа
                                if w2_cycles >= TIMEOUT_LAST_CYCLES {   
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1; 
                                    continue;
                                }
                            } else { // отрабатываем ожидание первого символа
                                if w1_cycles >= TIMEOUT_FIRST_CYCLES {   
                                    break;
                                } else {
                                    w1_cycles += 1; 
                                    continue;
                                }
                            }
                        }
                    }
                 }
                 Ok(x) => { // получили очередной символ от UPS
                    if self.rcv_buf.len() < UPS_BUF_LEN { // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                    } else {
                        break;
                    }
                    got_first_char = true; // после первого символа переходим на ожидание последнего
                    w2_cycles = 0;
                    ctx.at_timer.reset(); // timeout timer restart after each byte recieved
                    continue;
                },
             }
        }
        if self.rcv_buf.len() != 47 { // длина нормального ответа от UPS
            self.rcv_buf.clear();
            self.rcv_buf.extend_from_slice(b"!UPS failure!!").unwrap();
            writeln!(ctx.console, "UPS ret: {:?}", self.rcv_buf.as_slice()).unwrap();
            return false;
        }
        true
    }
}

//--------------------------------------------------------------
// Функции работы с SIM800L
//--------------------------------------------------------------

const SIM800_NUMBER_LEN: usize = 40; // длина буфера телефонного номера SIM800
const SIM800_RCV_BUF_LEN: usize = 1600; // длина буфера приёма данных от SIM800

#[derive(Debug, Clone, Copy, PartialEq)]
enum ComStates {
    Initial,    // не установлена связь с модулем
    Connected,  // связь с модулем есть, нет регистрации в сети
    Registered, // модуль зарегистрирован в сети
}

struct Sim800 {
    state: ComStates,
    rcv_buf: Vec<u8, SIM800_RCV_BUF_LEN>,
    auth: Vec<Vec<u8, SIM800_NUMBER_LEN>, 3>,
    active_num: u8,
    sim800_port: stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART3,
     (stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
     stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>,
    sim_reset_pin: stm32f1xx_hal::gpio::gpioa::PA6<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>,
}

impl Sim800 {
    fn new(
        port: stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART3,
     (stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
     stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>,
        pin: stm32f1xx_hal::gpio::gpioa::PA6<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>
    ) -> Sim800 {
        Sim800 { 
            state: ComStates::Initial,
            rcv_buf: Vec::new(),
            auth: Vec::new(), // allowed phone numbers
            active_num: 0,
            sim800_port: port,
            sim_reset_pin: pin
        }
    }

    // номер, на который будут отправляться СМС
    fn get_active_num(&mut self) -> &[u8] {
        &self.auth[(self.active_num-1) as usize][..]
    }

    // поиск подстроки в буфере приёма
    fn buf_contains(&mut self, pattern: &[u8]) -> bool {
        let psize = pattern.len();
        let bsize = self.rcv_buf.len();
        for i in 0..bsize {
            let rlimit = i+psize;
            if rlimit > bsize {
                break;
            }
            let sl = &self.rcv_buf[i..rlimit];
            if sl == pattern {
                return true;
            }
        }
        false
    }

    // отправка АТ команды и получение ответа от SIM800
    fn send_at_cmd_wait_resp(&mut self, ctx: &mut Context,
                            at_cmd: &[u8], // команда
                            toc: u16, // timeout for first char
                            to: u16) ->  bool { 
        // читаем мусор из порта
        loop {
            let res = self.sim800_port.read();
            match res {
                Err(nb::Error::Other(_)) => {
                   writeln!(ctx.console, "SIM e0").unwrap();
                   break;
                }
                Err(nb::Error::WouldBlock) => { // к счастью ничего нет
                    break;
                }
                Ok(_) => {} // если что, перезагрузится по сторожевому таймеру
            } 
        }
        // отправляем команду
        for cmd in at_cmd {
            block!(self.sim800_port.write(*cmd)).ok();
        }
        // пробуем получить ответ
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        loop {
            let res = self.sim800_port.read();
            match res {
                 Err(nb::Error::Other(_)) => {
                    writeln!(ctx.console, "SIM e1").unwrap();
                    break;
                    }
                 Err(nb::Error::WouldBlock) => { // символ не пришёл ещё
                      let t = ctx.at_timer.wait();
                      match t {
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "SIM e2").unwrap();
                            break;
                            }
                        Err(nb::Error::WouldBlock) => { // просто ждём ещё таймер
                            continue;
                        }
                        Ok(_) => { // сработал таймер отсчёта таймаута
                            if got_first_char { // отрабатываем ожидание последнего символа
                                if w2_cycles >= to {   
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1; 
                                    continue;
                                }
                            } else { // отрабатываем ожидание первого символа
                                if w1_cycles >= toc {   
                                    break;
                                } else {
                                    w1_cycles += 1; 
                                    continue;
                                }
                            }
                        }
                    }
                 }
                 Ok(x) => { // получили очередной символ от SIM800
                    if self.rcv_buf.len() < SIM800_RCV_BUF_LEN { // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                    } else {
                        break;
                    }
                    got_first_char = true; // после первого символа переходим на ожидание последнего
                    w2_cycles = 0;
                    ctx.at_timer.reset(); // timeout timer restart after each byte recieved
                    continue;
                },
             }
        }
        if self.rcv_buf.len() == 0 { // ничего не смогли получить
            writeln!(ctx.console, "SIM nrsp").unwrap();
            return false;
        }
        true
    }

    // отправка команды за несколько попыток 
    fn send_at_cmd_wait_resp_n(&mut self, ctx: &mut Context,
        at_cmd: &[u8],
        ans: &[u8],
        toc: u16, // timeout for first char
        to: u16, // timeout after last char
        tries: u8) ->  bool { // no of attempts
        // checks if reply from SIM800L contains ans using tries attempts
        let mut reply: bool = false;
        for _ in 1..=tries {
            self.send_at_cmd_wait_resp(ctx, at_cmd, toc, to);
            if self.buf_contains(ans) {
                reply = true;
                break;
            }
            ctx.delay.delay_ms(500_u16); // delay between attempts
            ctx.watchdog.feed();
        }
        reply
    }

    // блокировка приёма звонков
    fn gsm_busy(&mut self, ctx: &mut Context, set_to: bool) ->  bool {
        let command = if set_to {
            b"AT+GSMBUSY=1\n"
        } else {
            b"AT+GSMBUSY=0\n"
        };
        let r1 = self.send_at_cmd_wait_resp_n(ctx, command, b"OK\r", 100, 10, 3);
        if !(r1) {return false;}
        true
    }

    // команды инициализации до регистрации в сети
    fn init_set_0(&mut self, ctx: &mut Context) ->  bool {
        if self.state == ComStates::Initial {
            return false
        }
        // Reset to the factory settings
        let r1 = self.send_at_cmd_wait_resp_n(ctx, b"AT&F\n", b"OK\r", 100, 10, 3);
        // switch off echo
        let r2 = self.send_at_cmd_wait_resp_n(ctx, b"ATE0\n", b"OK\r", 50, 10, 3);
        // setup fixed baud rate 9600
        let r3 = self.send_at_cmd_wait_resp_n(ctx, b"AT+IPR=9600\n", b"OK\r", 100, 10, 3);
        if !(r1 && r2 && r3) {return false;}
        true
    }

    // executed after module registration in network
    fn init_set_1(&mut self, ctx: &mut Context) ->  bool {
        if self.state == ComStates::Initial {
            return false
        }
        // block unsolicited RINGs
        let r0 = self.gsm_busy(ctx, true);
        // Request calling line identification
        let r1 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CLIP=1\n", b"OK\r", 50, 10, 3);
        // Mobile Equipment Error Code
        let r2 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CMEE=0\n", b"OK\r", 50, 10, 3);
        // set the SMS mode to text
        let r3 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CMGF=1\n", b"OK\r", 50, 10, 3);
        // Disable messages about new SMS from the GSM module
        let r4 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CNMI=2,0\n", b"OK\r", 100, 10, 3);
        // send AT command to init memory for SMS in the SIM card
        // response:
        // +CPMS: <usedr>,<totalr>,<usedw>,<totalw>,<useds>,<totals>
        let r5 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CPMS=\"SM\",\"SM\",\"SM\"\n", b"+CPMS:", 100, 100, 3);
        // select phonebook memory storage
        let r6 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CPBS=\"SM\"\n", b"OK\r", 100, 10, 3);
        // Deactivate GPRS PDP context
        let r7 = self.send_at_cmd_wait_resp_n(ctx, b"AT+CIPSHUT\n", b"SHUT OK", 100, 10, 3);
        if !(r0 && r1 && r2 && r3 && r4 && r5 && r6 && r7) {return false;}
        true
    }

    // проверка/инициализация связи с модулем 
    fn check_com(&mut self, ctx: &mut Context) -> bool {
        let mut r = self.send_at_cmd_wait_resp_n(ctx, b"AT\n", b"OK\r", 50, 10, 20);
        if r {
            if self.state != ComStates::Registered {
                if self.state == ComStates::Initial {
                    r = self.init_set_0(ctx);    
                }
                self.state = ComStates::Connected;
            }
        } else {
            self.state = ComStates::Initial;
        }
        r
    }

    // перезагрузка модуля при проблемах 
    fn reboot(&mut self, ctx: &mut Context) -> bool {
        self.sim_reset_pin.set_low().unwrap();
        ctx.beep();
        ctx.delay.delay_ms(1_500_u16);
        self.sim_reset_pin.set_high().unwrap();
        ctx.watchdog.feed();
        writeln!(ctx.console, "REBOOT").unwrap();
        true
    }

    // проверка регистрации в сети
    fn check_reg(&mut self, ctx: &mut Context) -> bool {
        let mut reply: bool = false;
        let ans1 = b"+CREG: 0,1";
        let ans2 = b"+CREG: 0,5";
        for _ in 1..=10 { // 10 attempts
            let _ = self.send_at_cmd_wait_resp(ctx, b"AT+CREG?\n", 100, 20);
            if self.buf_contains(ans1) || self.buf_contains(ans2) {
                if self.state != ComStates::Registered {
                    reply = self.init_set_1(ctx);
                    if reply {
                        reply = self.init_auth(ctx);
                        //    for nbr in (&self.auth).into_iter() {
                        //        writeln!(ctx.console, "Num={}", core::str::from_utf8(&nbr).unwrap() );
                        //    }
                    }
                    self.state = ComStates::Registered; 
                } else {
                    reply = true;
                }
                break;
            } else {
                self.state = ComStates::Connected;
            }
            ctx.watchdog.feed();
            ctx.delay.delay_ms(5000_u16); // delay between attempts
            ctx.watchdog.feed();
        }
        reply
    }

    // Считываение первых 3 номеров с SIM карты. Они будут использоваться для авторизации звонков
    fn init_auth(&mut self, ctx: &mut Context) ->  bool {
        let a = EepromAdresses::Number;
        self.active_num = ctx.eeprom.read_byte(a.address() as u32).unwrap();
        if !(self.active_num > 0 && self.active_num < 4) { // неправильный номер в EEPROM
            self.active_num = 3; // дефолтное значение в 3 ячейке
            ctx.eeprom.write_byte(a.address() as u32, self.active_num).unwrap();
            ctx.delay.delay_ms(10_u16);
        }

        self.auth.clear();
        let mut reply: bool = false;
        for j in 1..=3 { // take first 3 numbers from SIM phonebook
            let mut buf: Vec<u8, 32> = Vec::new();
            write!(buf, "AT+CPBR={}\n", j).unwrap();
            for _ in 1..=3 { // make 3 attempts
                let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10);
                if self.buf_contains(b"OK\r") {
                    if !self.buf_contains(b"+CPBR:") { 
                        self.auth.push(Vec::<u8, SIM800_NUMBER_LEN>::from_slice(b"NA").unwrap()).unwrap();
                    } else {
                        // parse for phone number
                        let mut found = false;
                        let mut number: Vec<u8, SIM800_NUMBER_LEN> = Vec::new();
                        for sub in &self.rcv_buf {
                            if sub == &b'"' {
                                if found {break;}
                                found = true; continue;                            
                            }
                            if found {
                               //copy chars from SIM800 reply
                               number.push(*sub).unwrap();  
                            }
                        }
                        self.auth.push(number).unwrap();
                    }
                    reply = true;
                    break;
                }
                ctx.delay.delay_ms(500_u16); // delay between attempts
                ctx.watchdog.feed();
            }
        }
        reply
    }

    // отправка простого SMS на дефолтный номер
    fn send_sms(&mut self, ctx: &mut Context,
                msg: &[u8]) -> bool {
        let mut reply = false;
        let mut buf: Vec<u8, 160> = Vec::new();
        write!(buf, "AT+CMGS=\"{}\"\n", core::str::from_utf8(self.get_active_num()).unwrap()).unwrap();
        let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 100, 10);
        if self.buf_contains(b">") {
            buf.clear();
            write!(buf, "{}\u{001a}", core::str::from_utf8(&msg).unwrap()).unwrap();
            writeln!(ctx.console, "sms msg = {}", core::str::from_utf8(&buf).unwrap()).unwrap();
            let _ = self.send_at_cmd_wait_resp(ctx, buf.as_slice(), 700, 100);
            if self.buf_contains(b"+CMGS") {
                reply = true;
            }
        } else {
            reply = false;
        }
        ctx.delay.delay_ms(1000_u16); // чуть ждём после посылки
        reply
    }

    // Определение входящего авторизованного звонка
    fn call_detect(&mut self, ctx: &mut Context) -> bool {
        let mut reply = false;
        // enable incoming calls and unsolicited RINGs
        let r0 = self.gsm_busy(ctx, false);
        if !r0  {return false}

        // пробуем получить RING
        self.rcv_buf.clear();
        ctx.at_timer.reset(); // reset timeout counter
        let mut got_first_char = false; // признак, что получили что-то из порта
        let mut w1_cycles = 0; // циклов задержки ожидания первого символа в ответе
        let mut w2_cycles = 0; // циклов задержки ожидания последнего символа в ответе
        let mut led_state = false;
        loop {
            ctx.watchdog.feed();

            // мигаем периодически светодиодом - heartbeat
            let cnt3 = SEC_COUNT_LED.load(Ordering::Relaxed);
            if cnt3 > 1 {
                if led_state {
                    ctx.led.set_high().unwrap();
                    led_state = false;
                } else {
                    ctx.led.set_low().unwrap();
                    led_state = true;
                }
                SEC_COUNT_LED.store(0, Ordering::Relaxed); // сбрасываем таймер
            }

            let res = self.sim800_port.read();
            match res {
                 Err(nb::Error::Other(_)) => {
                    writeln!(ctx.console, "RING e1").unwrap();
                    break;
                    }
                 Err(nb::Error::WouldBlock) => { // символ не пришёл ещё
                      let t = ctx.at_timer.wait();
                      match t {
                        Err(nb::Error::Other(_)) => {
                            writeln!(ctx.console, "RING e2").unwrap();
                            break;
                            }
                        Err(nb::Error::WouldBlock) => { // просто ждём ещё таймер
                            continue;
                        }
                        Ok(_) => { // сработал таймер отсчёта таймаута
                            if got_first_char { // отрабатываем ожидание последнего символа
                                if w2_cycles >= 200 {   
                                    break; // вылет по таймауту
                                } else {
                                    w2_cycles += 1; 
                                    continue;
                                }
                            } else { // отрабатываем ожидание первого символа 10 сек
                                if w1_cycles >= 1000 {   
                                    break;
                                } else {
                                    w1_cycles += 1; 
                                    continue;
                                }
                            }
                        }
                    }
                 }
                 Ok(x) => { // получили очередной символ от SIM800
                    if self.rcv_buf.len() < SIM800_RCV_BUF_LEN { // защита от переполнения буфера
                        self.rcv_buf.push(x).unwrap();
                    } else {
                        break;
                    }
                    got_first_char = true; // после первого символа переходим на ожидание последнего
                    w2_cycles = 0;
                    ctx.at_timer.reset(); // timeout timer restart after each byte recieved
                    continue;
                },
             }
        }
        if self.rcv_buf.len() == 0 { // ничего не смогли получить
            writeln!(ctx.console, "NO RING").unwrap();
            self.gsm_busy(ctx, true); // block GSM RINGs
            return false;
        }

        // check RING number in buf
        let mut number: Vec::<u8, SIM800_NUMBER_LEN> = Vec::new();
        if !self.buf_contains(b"+CLIP:") { 
            number.push(b'N').unwrap();
        } else {
            // ищем номер вызывающего телефона
            let mut found = false;
            for sub in &self.rcv_buf {
                if sub == &b'"' {
                    if found {break;}
                    found = true; continue;                            
                }
                if found {
                   //copy chars from SIM800 reply
                   number.push(*sub).unwrap();  
                }
            }
            // звонок с номера
            writeln!(ctx.console, "RING: {}", core::str::from_utf8(&number).unwrap()).unwrap();
        }

        // check number is auth
        for (i, nbr) in (&self.auth).into_iter().enumerate() {
            if nbr == &number {
                //println!("auth num = {}", core::str::from_utf8(&number).unwrap());
                reply = true;
                let number_i = i as u8 + 1;
                if self.active_num != number_i {
                    self.active_num = number_i;
                    let a = EepromAdresses::Number;
                    ctx.save_byte(a.address() as u32, self.active_num);
                }
                break;
            }
        }

        // hang up call
        let r6 = self.send_at_cmd_wait_resp_n(ctx, b"ATH\n", b"OK\r", 50, 10, 3);
        if !r6  {return false}
        // block unsolicited RINGs
        let r1 = self.gsm_busy(ctx, true);
        if !r1 {return false}
        reply
    }
}

//--------------------------------------------------------------
// Функции отслеживания состояний датчиков 220в и температуры
//--------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq)]
enum EepromAdresses {
    V220State, // состояние датчика напряжения 220 в
    TempState, // состояние датчика температуры DS18B20
    Number     // телефонный номер отправки SMS
}

// возвращает адрес сохраненного значения в чипе EEPROM
impl EepromAdresses {
    fn address(self) -> u8 {
        match self {
            EepromAdresses::V220State => 0x01,
            EepromAdresses::TempState => 0x02, 
            EepromAdresses::Number    => 0x04,         
        }
    }
}

// характеристика указывает, что объект можно взять на контроль
trait Observable {
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800) -> bool;
}

// структура для отслеживания напряжения 220в питания МК

// константы подбираются экспериментально!!!
const VCC: f32 = 3.3; // напряжение питания МК
const V220_START: f32 = 1.5; // значение ADC, означающее приемлемое напряжение питания
const V220_LIMIT_LOW: f32 = 0.3; // пороговое значение ADC, означающее отсутствие питания

#[derive(PartialEq)]
enum V220States {
    ColdStart,  // инициализация измерений
    Monitoring, // ожидание падения напряжения 220 в
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
        adc1: adc::Adc<pac::ADC1>
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
            return true
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        match read_data {
            0 => { self.state = V220States::ColdStart },
            1 => { self.state = V220States::Monitoring }, 
            2 => { self.state = V220States::WaitForNormal },  
            _ => { self.state = V220States::ColdStart },         
        }
        true
    }

    // запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> bool {
        let data = {
            match self.state {
                V220States::ColdStart     => 0,
                V220States::Monitoring    => 1, 
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
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800) -> bool {
        match self.state {
            V220States::ColdStart     => { 
                self.measure();               
                self.state = V220States::Monitoring;
            },
            V220States::Monitoring    => {
                if self.voltage < V220_LIMIT_LOW { // сетевое напряжение пропало
                    sim.send_sms(ctx, b"220 failed");
                    writeln!(ctx.console, "220 failed {}", self.voltage).unwrap();
                    ctx.beep();

                    self.state = V220States::WaitForNormal;
                    self.save(ctx);
                }
            }, 
            V220States::WaitForNormal => {
                if self.voltage > V220_START { // сетевое напряжение вернулось
                    sim.send_sms(ctx, b"220 on-line");
                    writeln!(ctx.console, "220 on-line").unwrap();
                    ctx.beep();

                    self.state = V220States::Monitoring;
                    self.save(ctx);
                }                
            },  
        }       
        true
    }
}

// структура для отслеживания диапазона датчика температуры DS18B20

const TEMP_LIMIT_HIGH: f32 = 20.0; // приемлемое значение температуры
const TEMP_LIMIT_LOW: f32 = 15.0; // пороговое значение, означающее критическое понижение температуры

#[derive(PartialEq)]
enum TemperatureStates {
    ColdStart,  // инициализация измерений
    Monitoring, // ожидание выхода температуры за границы контроля
    WaitForNormal, // ожидание восстановления нормальной температуры
}

struct TempControl {
    state: TemperatureStates,
    address: u8,
    temp: f32,
    one_wire_bus: one_wire_bus::OneWire<stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>>,
}

impl TempControl {
    fn new(
        bus: one_wire_bus::OneWire<stm32f1xx_hal::gpio::Pxx<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>>>
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
            return true
        }
        let read_data = ctx.eeprom.read_byte(self.address as u32).unwrap();
        match read_data {
            0 => { self.state = TemperatureStates::ColdStart },
            1 => { self.state = TemperatureStates::Monitoring }, 
            2 => { self.state = TemperatureStates::WaitForNormal },  
            _ => { self.state = TemperatureStates::ColdStart },         
        }
        true
    }

    // запись состояния в EEPROM
    fn save(&mut self, ctx: &mut Context) -> bool {
        let data = {
            match self.state {
                TemperatureStates::ColdStart     => 0,
                TemperatureStates::Monitoring    => 1, 
                TemperatureStates::WaitForNormal => 2,  
            }       
        };
        ctx.save_byte(self.address as u32, data)
    }

    // измерение температуры
    fn measure(&mut self, ctx: &mut Context) -> f32 {
        let w1 = ds18b20::start_simultaneous_temp_measurement(&mut self.one_wire_bus, &mut ctx.delay);
        match w1 {
            Err(_) => { writeln!(ctx.console, "OW e1").unwrap(); return -101.0;}
            Ok(_) => {
                Resolution::Bits12.delay_for_measurement_time(&mut ctx.delay);
                let mut search_state = None;
                let w2 = self.one_wire_bus.device_search(search_state.as_ref(), false, &mut ctx.delay);
                match w2 {
                    Err(_) => { writeln!(ctx.console, "OW e2").unwrap(); return -102.0;}
                    Ok(None) => { writeln!(ctx.console, "OW none").unwrap(); return -103.0;}
                    Ok(Some((device_address, state))) => {
                        search_state = Some(state); // у нас только один датчик, дальше не ищем
                        if device_address.family_code() == ds18b20::FAMILY_CODE {
                            let w0: core::result::Result<ds18b20::Ds18b20, one_wire_bus::OneWireError<core::convert::Infallible>>
                             = Ds18b20::new(device_address);
                            match w0 {
                                Err(_) => { writeln!(ctx.console, "OW e5").unwrap(); return -104.0;}
                                Ok(sensor) => {
                                    let w3 = sensor.read_data(&mut self.one_wire_bus, &mut ctx.delay);
                                    match w3 {
                                        Err(_) => { writeln!(ctx.console, "OW e4").unwrap(); return -105.0;}
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
    fn check(&mut self, ctx: &mut Context, sim: &mut Sim800) -> bool {
        match self.state {
            TemperatureStates::ColdStart     => { 
                self.measure(ctx);               
                self.state = TemperatureStates::Monitoring;
            },
            TemperatureStates::Monitoring    => {
                if self.temp > -99.0 && self.temp < TEMP_LIMIT_LOW { // температура упала
                    sim.send_sms(ctx, b"Temp too low");
                    writeln!(ctx.console, "Temp too low {}", self.temp).unwrap();
                    ctx.beep();

                    self.state = TemperatureStates::WaitForNormal;
                    self.save(ctx);
                }
            }, 
            TemperatureStates::WaitForNormal => {
                if self.temp > TEMP_LIMIT_HIGH { // температура вернулось в норму
                    sim.send_sms(ctx, b"Temp OK");
                    writeln!(ctx.console, "Temp OK").unwrap();
                    ctx.beep();

                    self.state = TemperatureStates::Monitoring;
                    self.save(ctx);
                }                
            },  
        }       
        true
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
        adc2: adc::Adc<pac::ADC2>
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
