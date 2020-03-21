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

use stm32h7xx_hal::{nb::block, serial, pac, timer::GetClk, gpio::ExtiPin, gpio::Edge::RISING, gpio::gpioe::PE9, gpio::gpioe::PE11, gpio::gpioe::PE13, gpio::Input, gpio::Output, gpio::Floating, gpio::PushPull, interrupt, device::NVIC, device::TIM1, device::TIM2, device::TIM3, prelude::*};

use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::SYST;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;



// --- GLOBAL CONSTANTS, STRUCTS, AND VARIABLES ---

// Configuration
const MAX_RPS: f32 = 40.0; // Used for normalizing inputs and denormalizing outputs of the PID loop.

// Mutexes around shared variables
static PC_IN: Mutex<RefCell<Option<PE11<Input<Floating>>>>> = Mutex::new(RefCell::new(None)); // The pulse chain input pin struct.
static PC_TIMER: Mutex<RefCell<Option<stm32h7xx_hal::timer::Timer<TIM2>>>> = Mutex::new(RefCell::new(None)); // The timer struct for measuring pulse chain period.
static LAST_PERIOD_TICKS: AtomicU32 = AtomicU32::new(0);



// --- MAIN FUNCTION, SETUP, PID, AND REPORTING ---

#[entry]
fn main() -> ! {
    // Get access to internal cortex and chip peripherals.
    let cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
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
    // PWM output: PA6, TIM3_CH1
    let mut pwm_out = stm32_peripherals.TIM3.pwm(stm32_peripherals.GPIOA.split(&mut ccdr.ahb4).pa6.into_alternate_af2(), 1000.hz(), &mut ccdr);
    pwm_out.set_duty(65535_u16/*start off*/);
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
    let pc_timer = stm32_peripherals.TIM2.timer(1.hz() /*2^16 increments per period*/, &mut ccdr);
    // Globally expose the timer so the interrupt can use it.
    free(|cs| {
        PC_TIMER.borrow(cs).replace(Some(pc_timer));
    });

    // Configure timer for running the PID loop
    let mut update_timer = stm32_peripherals.TIM1.timer(1.hz(), &mut ccdr);

    // Configure a delay provider using the system timer.
    let mut delay = cortex_m_peripherals.SYST.delay(ccdr.clocks);   



    // --- STARTUP ---

    // Let the disk spin down for a bit before starting PID control, it'll have spun up during init.
    hprintln!("Waiting for the disk to spin down...").unwrap();
    delay.delay_ms(2000_u16);
    hprintln!("Starting PID controller!").unwrap();

    // Enable interrupts, start detecting encoder pulses.
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    // Configurable parameters
    let mut setpoint_rps: f32 = 0.0;
    let mut gains: (f32, f32, f32) = (0.0, 0.0, 0.0); // K_P, K_I, K_D
    let mut scalars: (f32, f32, f32) = compute_scalars(gains, 0.25); // for error @ t, error @ t - 1, error @ t - 2
    
    // Z^-n values
    let mut output_tm1: f32 = 0.0;
    let mut error_tm1: f32 = 0.0;
    let mut error_tm2: f32 = 0.0;

    // Circular FIFO averaging buffer
    let mut filter_index: usize = 0;
    let mut to_average: [u32; 500] = [0; 500];

    // UART command buffer. Receives 6 characters of ascii, lower index indicates recieved before.
    // 0th character is 'S' (setpoint), 'P' (proportional gain), 'I' (integral gain), 'D' (derivative gain).
    // 1-5th characters are base 10 number * 10 ^-3 (e.g. "12345" -> 12.345)
    let mut command_buf: [u8; 6] = [0; 6];

    // Records of speed, setpoint, error, and output values taken every 0.25 seconds to print at the end.
    let mut record_index: usize = 0;
    let mut records: [(f32, f32, f32, f32); 500] = [(0.0, 0.0, 0.0, 0.0); 500];

    loop {
        // The measurement, LAST_PERIOD_TICKS, could be updating at an absolute maximum of 24*~40 = 960Hz -> every ~ 1 ms.
        // We want to catch every sample so we'll update at least that fast too.
        // We start the timer immediately and then wait for it to complete once we've done all the processing for this update.
        update_timer.start(1.ms());
        // Because of the sensor "noise" from construction variation, we want to run the PID loop slower than a full revolution would take.
        // Assuming it revolves at slowest around 4 Hz, the PID loop should be run every 250 "updates".
        // As a basic filter to try and only keep the frequencies in the signal below at most 1/2 that, I'm just going to average the past 500 samples.

        to_average[filter_index] = LAST_PERIOD_TICKS.load(Ordering::Relaxed);
        filter_index = (filter_index + 1) % 500;

        // Check if PID loop should be updated.
        if (filter_index == 0) || (filter_index == 250) {
            let average_period_ticks: u32 = to_average.iter().sum::<u32>() / 500; // u32 is big enough for sum as individual samples only go up to u16. Dividing as integer is fine, there's enough resolution.
            let rps: f32 = (1.0 / (average_period_ticks as f32)) * (1.0/(65535.0 * 24.0));

            // PID!
            let error: f32 = normalize_error(setpoint_rps - rps);
            let output: f32 = constrain(output_tm1 + (scalars.0 * error) + (scalars.1 * error_tm1) + (scalars.2 * error_tm2), 0.0, 1.0); // Constrain the output as anti-windup measure.
            error_tm2 = error_tm1;
            error_tm1 = error;
            output_tm1 = output;

            // Update output
            pwm_out.set_duty(denormalize_output(output));

            // Update records
            records[record_index] = (rps, setpoint_rps, error, output);
            record_index += 1;
            if record_index == 500 {
                // If all the samples are taken, stop PID control to dump all the data.
                pwm_out.set_duty(65535);
                break
            }
        }

        // Check if there's new data being recieved from the UART
        match rx.read() {
            // If there was an error or there's no data in the buffer (a particular error variant), don't do anything.
            Err(_) => {},
            Ok(b) => {
                // Shift in the new data
                shift_in(&mut command_buf, b);
                // Check if it's potentially a full command, uppercase ASCII in the first spot.
                if (command_buf[0] >= 65) && (command_buf[0] <= 90) {
                    match to_num(command_buf) { // Try converting the last five bytes to a number
                        Some(num) => match command_buf[0] { // Set the appropriate number and update the scalars if needed.
                            83 /*S*/ => setpoint_rps = num,
                            80 /*P*/ => {
                                gains.0 = num;
                                scalars = compute_scalars(gains, 0.25);
                            }
                            73 /*I*/ => {
                                gains.1 = num;
                                scalars = compute_scalars(gains, 0.25);
                            }
                            68 /*D*/ => {
                                gains.2 = num;
                                scalars = compute_scalars(gains, 0.25);
                            }
                            _ => {} // Not a valid command character, ignore
                        }
                        None => {} // Not a valid number, ignore
                    }
                }
            },
        }

        // Wait until the timer has completed before starting another 1ms update cycle.
        block!(update_timer.wait()).ok();
    }

    // Dump the records as CSV
    hprintln!("Transferring data...").unwrap();
    for r in records.iter() {
        writeln!(tx, "{},{},{},{}", r.0, r.1, r.2, r.3).unwrap(); // Lines written as CSV
    }
    hprintln!("Data transferred!").unwrap();

    loop {}
}

// Takes in a error speed, returns a normalized error from -1 to 1.
fn normalize_error(rps: f32) -> f32 {
    constrain(rps / MAX_RPS, -1.0, 1.0)
}

// Takes in a PID loop output (with windup protection from 0-1), returns the PWM value to output.
// Attempts to correct for ESC nonlinearities by inverting the approximation rps = 50*duty_cycle + 20
fn denormalize_output(output: f32) -> u16 {
    // Convert normalized to RPS, to duty cycle, constrain, then to u16 to get that duty cycle
    ((1.0 - constrain(((output * MAX_RPS) - 20.0) * 0.02, 0.0, 1.0)) * 65535.0) as u16
}

// Shift buffer towards lower indices and add new to the end
fn shift_in(buf: &mut [u8; 6], new: u8) {
    for i in 0..4 {
        buf[i] = buf[i+1];
    }
    buf[5] = new;
}

// Try converting the last 5 characters in the buffer to a number (multiplied by 10^-3)
fn to_num(buf: [u8; 6]) -> Option<f32> {
    let mut num: f32 = 0.0;
    for i in 1..5 {
        match ascii_to_digit(buf[i]) {
            Some(digit) => {
                num += digit * ten_pow(2 - (i as i32));
            }
            None => return None
        }
    }
    Some(num)
}

// Calculate 10 to a power. Unfortunately the default rust implementation isn't stable for no_std
fn ten_pow(exp: i32) -> f32 {
    if exp == 0 {
        1.0
    }else if exp < 0 {
        0.1 * ten_pow(exp + 1)
    }else {
        10.0 * ten_pow(exp - 1)
    }
}

// Try converting an ascii character into the corresponding number 0-9
fn ascii_to_digit(byte: u8) -> Option<f32> {
    if (byte >= 48) && (byte <= 57) {
        Some((byte - 48) as f32)
    } else {
        None
    }
}

// Constrain a number to be within and upper and lower bound.
fn constrain(val: f32, min: f32, max: f32) -> f32 {
    if val < min {
        min
    }else if val > max {
        max
    } else {
        val
    }
}

// Compute the scalars for the difference equation formulation of a PID loop.
// Takes in K_P, K_I, and K_D as well as the update period.
fn compute_scalars(gains: (f32, f32, f32), t: f32) -> (f32, f32, f32) {
    ((gains.0) +        (gains.1 * (t/2.0)) + (gains.2 / t),
     (gains.0 * -1.0) + (gains.1 * (t/2.0)) + (gains.2 * (-2.0 / t)),
                                              (gains.2 / t))
}



// --- PULSE CHAIN INTERRUPT ---

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        if let Some(ref mut pc_in) = PC_IN.borrow(cs).borrow_mut().deref_mut() {
            if let Some(ref mut timer) = PC_TIMER.borrow(cs).borrow_mut().deref_mut() {
                // Capture current count and immediately reset timer to start counting for the next interrupt
                let count: u32 = timer.counter();
                timer.reset_counter();
                // Save the new count
                LAST_PERIOD_TICKS.store(count, Ordering::Relaxed);
            }
            // Prevent the interrupt from running again until there's a new pulse
            pc_in.clear_interrupt_pending_bit();
        }
    });
}
