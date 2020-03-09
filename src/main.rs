#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

//use cortex_m::asm;
use cortex_m_rt::entry;

use cortex_m_semihosting::{hprintln};

use stm32h7xx_hal::{pac, timer::GetClk, gpio::ExtiPin, gpio::Edge::RISING, gpio::gpioe::PE11, gpio::gpioe::PE13, gpio::Input, gpio::Output, gpio::Floating, gpio::PushPull, interrupt, device::NVIC, device::TIM2, prelude::*};

use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};

use cortex_m::interrupt::{free, Mutex};

use core::cell::RefCell;
use core::ops::DerefMut;
use core::sync::atomic::{AtomicU32, Ordering};

static PC_IN: Mutex<RefCell<Option<PE11<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static PC_TIMER: Mutex<RefCell<Option<stm32h7xx_hal::timer::Timer<TIM2>>>> = Mutex::new(RefCell::new(None));
static PC_PERIOD: AtomicU32 = AtomicU32::new(0);
static TEST: Mutex<RefCell<Option<PE13<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
//static DATA: Mutex<RefCell<[f32; 10000]>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut stm32_peripherals = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let operating_voltage = stm32_peripherals.PWR.constrain().freeze();

    // Configure system clock to 100 MHz and 
    let rcc = stm32_peripherals.RCC.constrain();
let mut ccdr = rcc.sys_ck(100.mhz()).freeze(operating_voltage, &stm32_peripherals.SYSCFG);


    // Enable and Configure GPIO
    let gpioe = stm32_peripherals.GPIOE.split(&mut ccdr.ahb4);
    // PWM output: PE9 (D6) - TIMER_A_PWM1, TIM1_CH1
    let mut pwm_out = stm32_peripherals.TIM1.pwm(gpioe.pe9.into_alternate_af1(), 1000.hz(), &mut ccdr);
    pwm_out.set_duty(16000/*65535_u16*/);
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

    let mut test = gpioe.pe13.into_push_pull_output();free(|cs| {
        TEST.borrow(cs).replace(Some(test));
    });
    
    let mut delay = cortex_m_peripherals.SYST.delay(ccdr.clocks);

    // Enable interrupts
    unsafe {
        NVIC::unmask(interrupt::EXTI15_10);
    }

    loop {
        let p: u32 = PC_PERIOD.load(Ordering::Relaxed);
        delay.delay_ms(10_u16);
        hprintln!("{} Hz", 65535.0 / (p as f32)).unwrap();
        delay.delay_ms(100_u16);
    }
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        let mut pc_in_ref = PC_IN.borrow(cs).borrow_mut();
        if let Some(ref mut pin) = pc_in_ref.deref_mut() {
            let mut pc_timer_ref = PC_TIMER.borrow(cs).borrow_mut();
            if let Some(ref mut timer) = pc_timer_ref.deref_mut() {
                let count: u32 = timer.counter();
                timer.reset_counter();
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