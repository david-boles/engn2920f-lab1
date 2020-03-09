#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

//use cortex_m::asm;
// use cortex_m::iprintln;

use cortex_m_rt::entry;

use cortex_m_semihosting::{hprintln};

use stm32h7xx_hal::{nb::block, serial, pac, timer::GetClk, gpio::ExtiPin, gpio::Edge::RISING, gpio::gpioe::PE11, gpio::gpioe::PE13, gpio::Input, gpio::Output, gpio::Floating, gpio::PushPull, interrupt, device::NVIC, device::TIM2, prelude::*};

use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};

use cortex_m::interrupt::{free, Mutex};

use core::cell::RefCell;
use core::ops::DerefMut;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;

static PC_IN: Mutex<RefCell<Option<PE11<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static PC_TIMER: Mutex<RefCell<Option<stm32h7xx_hal::timer::Timer<TIM2>>>> = Mutex::new(RefCell::new(None));
static PC_PERIOD: AtomicU32 = AtomicU32::new(0);

const samples: usize = 10000; // Number of samples (taken every interrupt) to record and print.
#[derive(Copy, Clone)]
struct Sample {
    i: usize, // Sample index
    pp: u32, // Pulse period, measured in 1/(2^16)ths of a second
    rps: f32, // Revolutions per second (1/24th the frequency of pulses)
}
static DATA_I: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0_usize)); // Index of the sample currently being worked on. = samples indicates capture is complete.
static DATA: Mutex<RefCell<[Sample; samples]>> = Mutex::new(RefCell::new([Sample{i: 0_usize, pp: 0_u32, rps: 0_f32}; samples]));

// Output pin for testing
static TEST: Mutex<RefCell<Option<PE13<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut stm32_peripherals = pac::Peripherals::take().unwrap();

    let stim = &mut cortex_m_peripherals.ITM.stim[0];

    // Constrain and Freeze power
    let operating_voltage = stm32_peripherals.PWR.constrain().freeze();

    // Configure system clock to 100 MHz and 
    let rcc = stm32_peripherals.RCC.constrain();
    let mut ccdr = rcc.sys_ck(100.mhz()).freeze(operating_voltage, &stm32_peripherals.SYSCFG);


    // Configure UART for Virtual COM Port
    let gpiod = stm32_peripherals.GPIOD.split(&mut ccdr.ahb4);
    let tx = gpiod.pd8.into_alternate_af7();
    let rx = gpiod.pd9.into_alternate_af7();
    let serial = stm32_peripherals
        .USART3
        .usart((tx, rx), serial::config::Config::default().baudrate(115_200_u32.bps()), &mut ccdr)
        .unwrap();
    let (mut tx, mut rx) = serial.split();

    // Enable and Configure GPIO
    let gpioe = stm32_peripherals.GPIOE.split(&mut ccdr.ahb4);
    // PWM output: PE9 (D6) - TIMER_A_PWM1, TIM1_CH1
    let mut pwm_out = stm32_peripherals.TIM1.pwm(gpioe.pe9.into_alternate_af1(), 1000.hz(), &mut ccdr);
    pwm_out.set_duty(65535_u16/*65535_u16*/);
    pwm_out.enable();
    // Pulse chain input: PE11 (D5)
    let mut pc_in = gpioe.pe11.into_floating_input();
    pc_in.make_interrupt_source(&mut stm32_peripherals.SYSCFG);
    pc_in.enable_interrupt(&mut stm32_peripherals.EXTI);
    pc_in.trigger_on_edge(&mut stm32_peripherals.EXTI, RISING);
    // Globally expose the pulse counter pin so the interrupt can clear itself
    free(|cs| {
        PC_IN.borrow(cs).replace(Some(pc_in));
    });

    // Configure pulse chain period timer
    let mut pc_timer = stm32_peripherals.TIM2.timer(1.hz() /*2^16 increments per period*/, &mut ccdr);
    // Globally expose the timer so the interrupt can use it.
    free(|cs| {
        PC_TIMER.borrow(cs).replace(Some(pc_timer));
    });

    // Configure GPIO pin for testing
    let mut test = gpioe.pe13.into_push_pull_output();free(|cs| {
        TEST.borrow(cs).replace(Some(test));
    });

    let mut delay = cortex_m_peripherals.SYST.delay(ccdr.clocks);
    




    // Let the disk spin down for a bit before starting PID control, it'll have spun up during init.
    hprintln!("Waiting for the disk to spin down...").unwrap();
    delay.delay_ms(5000_u16);
    pwm_out.set_duty(65535_u16 / 2/*65535_u16*/);
    hprintln!("Starting PID controller!").unwrap();

    // Enable interrupts
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    loop {
        // let p: u32 = PC_PERIOD.load(Ordering::Relaxed);
        // delay.delay_ms(10_u16);
        // hprintln!("{} increments, {} Hz", p, 65535.0 / (p as f32)).unwrap();
        // writeln!(tx, "Hello, world!").unwrap();
        free(|cs| {
            let mut data_i_ref = DATA_I.borrow(cs).borrow_mut();
            let data_i = data_i_ref.deref_mut();

            if *data_i == samples {
                hprintln!("Data collected! Writing to console...").unwrap();
                let mut data_ref = DATA.borrow(cs).borrow_mut();
                let data = data_ref.deref_mut();

                for sample in data.iter() {
                    writeln!(tx, "{},{},{}", sample.i, sample.pp, sample.rps).unwrap();
                }

                hprintln!("Data transferred!").unwrap();
                delay.delay_ms(60000_u16);
            }
        });

        delay.delay_ms(1000_u16);
    }
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        if let Some(ref mut pin) = PC_IN.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut timer) = PC_TIMER.borrow(cs).borrow_mut().deref_mut() {
                let mut data_i_ref = DATA_I.borrow(cs).borrow_mut();
                let mut data_i = data_i_ref.deref_mut();
                let mut data_ref = DATA.borrow(cs).borrow_mut();
                let mut data = data_ref.deref_mut();

                // Capture current count and immediately reset to start counting for the next interrupt
                let count: u32 = timer.counter();
                timer.reset_counter();

                // PID
                let speed: f32 = 65535.0 / ((count as f32) * 24.0);

                // Collect samples if applicable.
                if *data_i < samples {
                    data[*data_i] = Sample{
                        i: *data_i,
                        pp: count,
                        rps: speed,
                    };
                    *data_i += 1;
                }
                
                PC_PERIOD.store(count, Ordering::Relaxed);
            }
            pin.clear_interrupt_pending_bit();
        }
    });
}

// vscode autocomplete doesn't show traits I guess '\O/`
// doc for structs appear in multiple modules, 
//interrupts correspond to which pin??? determining which pin has interrupts
//enabling interrupts
//semihosting messes with interrupts
//clocks only count w/ u16s but return count as u32s
//clock frequency is when it gets reset?
//setting clock increment frequency?