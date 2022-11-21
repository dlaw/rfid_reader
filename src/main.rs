#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use core::sync::atomic::{AtomicBool, Ordering};
    use dwt_systick_monotonic::{fugit, DwtSystick};
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f4xx_hal::{gpio::*, pac::*, prelude::*, timer::Timer};

    const CLOCK_FREQ_HZ: u32 = 48_000_000;
    type Duration = fugit::Duration<u32, 1, CLOCK_FREQ_HZ>;
    type Instant = fugit::Instant<u32, 1, CLOCK_FREQ_HZ>;

    const CLOCK_EDGES_PER_BIT: u16 = 100;
    const CARD_SEQ_LEN: usize = 96;
    const CARD_SEQ: u128 = 0x11abcdabcdabc1111111111; // bogus value

    #[shared]
    struct Shared {
        request_unlock: AtomicBool,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        latch: Pin<'B', 3, Output<PushPull>>,
        rfid_out: Pin<'B', 8, Input>,
        timer: TIM4,
    }

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = DwtSystick<CLOCK_FREQ_HZ>;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("door running");

        // System clock and monotonic timer
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(CLOCK_FREQ_HZ.Hz()).freeze();
        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            CLOCK_FREQ_HZ,
        );

        // GPIO pins
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();
        let _rfid_nshd = gpiob.pb9.into_open_drain_output_in_state(PinState::Low);
        let _rfid_clk = gpiob.pb7.into_alternate_open_drain::<2>(); // TIM4 TI2
        let mut rfid_out = gpiob.pb8.into_pull_down_input();

        // Configure TIM4 as a hardware counter for CLK edges using TI2 input.
        // Register configuration per ST RM0383, section 13.3.3.
        // Use the HAL to enable and reset, then release for manual register config.
        let timer = Timer::new(ctx.device.TIM4, &clocks).release();
        timer
            .ccmr1_input()
            .write(|w| w.cc2s().ti2().ic2f().bits(0b0011));
        timer.ccer.write(|w| w.cc2np().set_bit().cc2p().set_bit());
        timer.smcr.write(|w| w.sms().ext_clock_mode().ts().ti2fp2());
        timer.cr1.write(|w| w.cen().set_bit());

        // Enable edge-triggered interrupt for RFID OUT pin
        rfid_out.make_interrupt_source(&mut ctx.device.SYSCFG.constrain());
        rfid_out.enable_interrupt(&mut ctx.device.EXTI);
        rfid_out.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        (
            Shared {
                request_unlock: AtomicBool::new(false),
            },
            Local {
                led: gpioc.pc13.into_push_pull_output_in_state(PinState::High),
                latch: gpiob.pb3.into_push_pull_output_in_state(PinState::Low),
                rfid_out,
                timer,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(shared = [&request_unlock], local = [led, latch])]
    fn idle(ctx: idle::Context) -> ! {
        let mut unlock_until: Option<Instant> = None;
        loop {
            if ctx.shared.request_unlock.swap(false, Ordering::Relaxed) {
                rprintln!("unlock");
                unlock_until.replace(monotonics::now() + Duration::secs(3));
            } else if let Some(t) = unlock_until {
                if t < monotonics::now() {
                    rprintln!("lock");
                    unlock_until.take();
                }
            }
            let unlock = unlock_until.is_some();
            ctx.local.led.set_state(PinState::from(!unlock)); // LED inverted in hardware
            ctx.local.latch.set_state(PinState::from(unlock));
        }
    }

    #[task(binds = EXTI9_5, shared = [&request_unlock], local = [
        rfid_out,
        timer,
        last_edge_times: [u16; 12] = [0; 12],   // units of CLK edges (TIM4 counts)
        last_bit_end_time: Option<u16> = None,  // units of CLK edges (TIM4 counts)
        demodulated_bits: u128 = 0,
    ])]
    fn on_exti(ctx: on_exti::Context) {
        ctx.local.rfid_out.clear_interrupt_pending_bit();
        let clk_count: u16 = ctx.local.timer.cnt.read().cnt().bits();
        let edge_times = ctx.local.last_edge_times;
        let bit_end_time = ctx.local.last_bit_end_time;
        let elapsed_time = bit_end_time.map_or(CLOCK_EDGES_PER_BIT, |t| clk_count.wrapping_sub(t));
        if elapsed_time >= CLOCK_EDGES_PER_BIT + 10 {
            *bit_end_time = None;
        }
        // Modulated zero bit: 8 clock edges per data edge
        // Check if the last 12 data edges covered 96 +/- 3 clock edges
        let zero_time = clk_count.wrapping_sub(edge_times[11]);
        let zero_bit = 93 <= zero_time && zero_time <= 99;
        // Modulated one bit: 10 clock edges per data edge
        // Check if the last 10 data edges covered 100 +/- 3 clock edges
        let one_time = clk_count.wrapping_sub(edge_times[9]);
        let one_bit = 97 <= one_time && one_time <= 103;
        if (elapsed_time > CLOCK_EDGES_PER_BIT - 10) && (zero_bit || one_bit) {
            *bit_end_time = match bit_end_time {
                // 1.25 kHz modulation: 100 clock edges per modulated bit
                Some(t) => Some(t.wrapping_add(CLOCK_EDGES_PER_BIT)),
                None => Some(clk_count),
            };
            let bits = ctx.local.demodulated_bits;
            *bits = (*bits << 1) | (one_bit as u128);
            if *bits & ((1 << CARD_SEQ_LEN) - 1) == CARD_SEQ {
                ctx.shared.request_unlock.store(true, Ordering::Relaxed);
            }
        }
        edge_times.rotate_right(1);
        edge_times[0] = clk_count;
    }
}
