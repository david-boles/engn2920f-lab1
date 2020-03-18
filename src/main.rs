#![no_std]
#![no_main]

// --- IMPORTS ---

// Panic (Rust feature) handling method
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

use cortex_m_semihosting::{hprintln};

use stm32h7xx_hal::{nb::block, serial, pac, timer::GetClk, gpio::ExtiPin, gpio::Edge::RISING, gpio::gpioe::PE9, gpio::gpioe::PE11, gpio::gpioe::PE13, gpio::Input, gpio::Output, gpio::Floating, gpio::PushPull, interrupt, device::NVIC, device::TIM1, device::TIM2, prelude::*};

use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::SYST;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;



// --- GLOBAL CONSTANTS, STRUCTS, AND VARIABLES ---

// Configuration
const pulsesToCombine: usize = 24; // Number of pulses to combine into one sample and PID update.
const samples: usize = 1000; // Number of samples to record and report.

// Struct for storing and then reporting a measurement sample. These correspond to pulsesToCombine pulses.
#[derive(Copy, Clone)]
struct Sample {
    dt: f32, // Time since last sample (not pulse)
    rps: f32, // Revolutions per second (1/24th the frequency of pulses)
}

// Mutexes around shared variables
static PWM_OUT: Mutex<RefCell<Option<stm32h7xx_hal::pwm::Pwm<TIM1, stm32h7xx_hal::pwm::C1>>>> = Mutex::new(RefCell::new(None)); // The PWM output struct.
static PC_IN: Mutex<RefCell<Option<PE11<Input<Floating>>>>> = Mutex::new(RefCell::new(None)); // The pulse chain input pin struct.
static PC_TIMER: Mutex<RefCell<Option<stm32h7xx_hal::timer::Timer<TIM2>>>> = Mutex::new(RefCell::new(None)); // The timer struct for measuring pulse chain period.
static DATA_I: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0_usize)); // Index of the sample currently being worked on. = samples indicates capture is complete.
static DATA: Mutex<RefCell<[Sample; samples]>> = Mutex::new(RefCell::new([Sample{dt: 0_f32, rps: 0_f32}; samples])); // Array of all the samples, initialized to 0.
static TEST: Mutex<RefCell<Option<PE13<Output<PushPull>>>>> = Mutex::new(RefCell::new(None)); // Testing pin to observe callback timing.



// --- MAIN FUNCTION, SETUP AND REPORTING ---

#[entry]
fn main() -> ! {
    // Get access to internal cortex and chip peripherals.
    let mut cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut stm32_peripherals = pac::Peripherals::take().unwrap();

    // Constrain and freeze power configuration
    let operating_voltage = stm32_peripherals.PWR.constrain().freeze();

    // Configure system clock to 100 MHz and other main clocks.
    let rcc = stm32_peripherals.RCC.constrain();
    let mut ccdr = rcc.sys_ck(100.mhz()).freeze(operating_voltage, &stm32_peripherals.SYSCFG);

    // Configure UART for the board's Virtual COM Port
    let gpiod = stm32_peripherals.GPIOD.split(&mut ccdr.ahb4);
    let tx = gpiod.pd8.into_alternate_af7();
    let rx = gpiod.pd9.into_alternate_af7();
    let serial = stm32_peripherals
        .USART3
        .usart((tx, rx), serial::config::Config::default().baudrate(115_200_u32.bps()), &mut ccdr)
        .unwrap();
    let (mut tx, mut rx) = serial.split();

    // Enable and configure GPIO
    let gpioe = stm32_peripherals.GPIOE.split(&mut ccdr.ahb4);
    // PWM output: PE9 (D6) - TIMER_A_PWM1, TIM1_CH1
    let mut pwm_out = stm32_peripherals.TIM1.pwm(gpioe.pe9.into_alternate_af1(), 1000.hz(), &mut ccdr);
    pwm_out.set_duty(65535_u16/*start off*/);
    pwm_out.enable();
    // Globally expose the pwm output so the interrupt can control it
    free(|cs| {
        PWM_OUT.borrow(cs).replace(Some(pwm_out));
    });
    // Pulse chain input: PE11 (D5)
    let mut pc_in = gpioe.pe11.into_floating_input();
    pc_in.make_interrupt_source(&mut stm32_peripherals.SYSCFG);
    pc_in.enable_interrupt(&mut stm32_peripherals.EXTI);
    pc_in.trigger_on_edge(&mut stm32_peripherals.EXTI, RISING);
    // Globally expose the pulse counter pin so the interrupt can clear itself
    free(|cs| {
        PC_IN.borrow(cs).replace(Some(pc_in));
    });
    // Test output pin: PE13
    let mut test = gpioe.pe13.into_push_pull_output();
    // Globally expose the pin so the interrupt can use it.
    free(|cs| {
        TEST.borrow(cs).replace(Some(test));
    });

    // Configure pulse chain period timer
    let mut pc_timer = stm32_peripherals.TIM2.timer(1.hz() /*2^16 increments per period*/, &mut ccdr);
    // Globally expose the timer so the interrupt can use it.
    free(|cs| {
        PC_TIMER.borrow(cs).replace(Some(pc_timer));
    });

    // Configure a delay provider using the system timer.
    let mut delay = cortex_m_peripherals.SYST.delay(ccdr.clocks);   



    // --- STARTUP ---

    // Let the disk spin down for a bit before starting PID control, it'll have spun up during init.
    hprintln!("Waiting for the disk to spin down...").unwrap();
    delay.delay_ms(2000_u16);
    hprintln!("Starting PID controller!").unwrap();

    // Enable interrupts, starts PID control.
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    loop {
        free(|cs| {
            // Check whether the interrupt has finished capturing data.
            let mut data_i_ref = DATA_I.borrow(cs).borrow_mut();
            let data_i = data_i_ref.deref_mut();

            if *data_i == samples {
                // Write the data to the console.
                hprintln!("Data collected! Writing to console...").unwrap();
                let mut data_ref = DATA.borrow(cs).borrow_mut();
                let data = data_ref.deref_mut();

                for sample in data.iter() {
                    writeln!(tx, "{},{}", sample.dt, sample.rps).unwrap(); // Lines written as CSV
                }

                hprintln!("Data transferred!").unwrap();

                // Wait a while so that the data isn't written out more than once.
                delay.delay_ms(60000_u16);
            }
        });

        // Wait another sec to check again.
        delay.delay_ms(1000_u16);
    }
}



// --- PULSE CHAIN INTERRUPT, PID CONTROLLER ---

#[interrupt]
fn EXTI15_10() {

    // Persistent variables
    static mut combine_i: usize = 0; // Index of how many pulses have been stored to combine into one sample.
    static mut combined: [u32; pulsesToCombine] = [0_u32; pulsesToCombine]; // Array to store pulses to combine, initialized to 0.

    static mut last_error: f32 = 0.0; // Error from the last sample.
    static mut error_int: f32 = 0.0; // The integrated error over time.

    free(|cs| {
        if let Some(ref mut pc_in) = PC_IN.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut pwm_out) = PWM_OUT.borrow(cs).borrow_mut().deref_mut() {
                if let Some(ref mut test) = TEST.borrow(cs).borrow_mut().deref_mut() {
                    if let Some(ref mut timer) = PC_TIMER.borrow(cs).borrow_mut().deref_mut() {
                        // Capture current count and immediately reset timer to start counting for the next interrupt
                        let count: u32 = timer.counter();
                        timer.reset_counter();

                        // Indicate externally that the interrupt is running
                        test.set_high().unwrap();

                        // Store this count to get averaged
                        combined[*combine_i] = count;
                        *combine_i += 1;

                        // If time to update PID... (a new full "sample")
                        if *combine_i == pulsesToCombine {
                            *combine_i = 0;
                            
                            // Get access to data storage variables.
                            let mut data_i_ref = DATA_I.borrow(cs).borrow_mut();
                            let mut data_i = data_i_ref.deref_mut();
                            let mut data_ref = DATA.borrow(cs).borrow_mut();
                            let mut data = data_ref.deref_mut();
        
                            // PID
                            let count_sum: u32 = combined.iter().sum(); // Sum the total count
                            let dt: f32 = (count_sum as f32) / 65535.0; // Convert the total count to delta time
                            let rps: f32 = (pulsesToCombine as f32) / (dt * 24.0); // Convert the delta time to a speed
                            let error = (
                                if *data_i < 300 {
                                    10.0
                                }else if *data_i < 600 {
                                    14.0
                                }else {
                                    6.0
                                }
                            ) - rps; // Determine the speed error. It's hardcoded, currently as changing between three values.
                            *error_int += error * dt; // Update the integral with the new error sample
                            let mut output: f32 = 
                                (0.2 * error) + // P term
                                (0.007 * *error_int) + // I term
                                (0.035 * ((error - *last_error) / dt)) + // D term
                                0.3; // Constant term
                            *last_error = error; // Update the last error to be the current one.

                            // Constrain and set output
                            if output > 1.0 {
                                output = 1.0;
                            }else if output < 0.0 {
                                output = 0.0;
                            }
                            pwm_out.set_duty((65535.0 * (1.0 - output)) as u16);
        
                            // Collect samples if they haven't all been already.
                            if *data_i < samples {
                                data[*data_i] = Sample{
                                    dt,
                                    rps,
                                };
                                *data_i += 1; // Increment sample index counter.
                            }
                        }

                        // Indicate externally that the interrupt is stopping
                        test.set_low().unwrap();
                    }
                }
            }
            // Prevent the interrupt from running again until there's a new pulse
            pc_in.clear_interrupt_pending_bit();
        }
    });
}
