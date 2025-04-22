//! This example demonstrates the loop playback of audio mapped on SDRAM.
//! By setting the D16 pin low, you can record audio from the SAI input.
//! It overwrites the audio buffer from start to end and automatically stops recording after 10 seconds.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use daisy_embassy::audio::{AudioConfig, Fs, Interface};
use daisy_embassy::hal::fmc::Fmc;
use daisy_embassy::hal::gpio::{Level, Output, Speed};
use daisy_embassy::hal::interrupt;
use daisy_embassy::hal::interrupt::{InterruptExt, Priority};
use daisy_embassy::hal::peripherals::FMC;
use daisy_embassy::hal::{exti::ExtiInput, gpio::Pull};
use daisy_embassy::sdram::FmcDevice;
use daisy_embassy::{audio::HALF_DMA_BUFFER_LENGTH, hal, new_daisy_board, sdram::SDRAM_SIZE};
use defmt::{debug, info};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::join::join;
use embassy_time::{Delay, Timer};
use stm32_fmc::Sdram;
use {defmt_rtt as _, panic_probe as _};

//take 48000(Hz) * 10(Sec) * 2(stereo)
const BLOCK_LENGTH: usize = HALF_DMA_BUFFER_LENGTH;
const LOOPER_LENGTH: usize = SDRAM_SIZE / core::mem::size_of::<u32>();
const SILENCE: u32 = u32::MAX / 2;
static RECORD_PRESSED: AtomicBool = AtomicBool::new(false);
static CLEAR_PRESSED: AtomicBool = AtomicBool::new(false);
static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
}

enum LooperState {
    Cleared,
    Record,
    Playback,
    Overdub,
}

fn record(
    recording_input: &[u32],
    loop_buffer: &mut [u32],
    cursor: usize,
    looper_state: &mut LooperState,
    record_end: &mut usize,
) {
    if cursor + BLOCK_LENGTH >= LOOPER_LENGTH {
        *looper_state = LooperState::Playback;
        *record_end = cursor;
        info!("out of memory");
        return;
    }
    loop_buffer[cursor..cursor + BLOCK_LENGTH].copy_from_slice(recording_input);
}

fn overdub(recording_input: &[u32], loop_buffer: &mut [u32], cursor: usize, record_end: usize) {
    let remain = BLOCK_LENGTH.min(record_end - cursor);
    let frac = BLOCK_LENGTH - remain;
    loop_buffer[cursor..(cursor + remain)].copy_from_slice(&recording_input[0..remain]);
    if frac > 0 {
        loop_buffer[0..frac].copy_from_slice(&recording_input[remain..BLOCK_LENGTH]);
    }
}

fn playback(output: &mut [u32], loop_buffer: &[u32], cursor: usize, record_end: usize) {
    let remain = BLOCK_LENGTH.min(record_end - cursor);
    let frac = BLOCK_LENGTH - remain;
    for (i, val) in loop_buffer[cursor..(cursor + remain)].iter().enumerate() {
        output[i] += *val;
    }
    if frac > 0 {
        for (i, val) in loop_buffer[0..frac].iter().enumerate() {
            output[remain + i] += *val;
        }
    }
}

fn increase_cursor(cursor: &mut usize, record_end: usize) {
    *cursor += BLOCK_LENGTH;
    if record_end != 0 && *cursor >= record_end {
        *cursor -= record_end;
        info!("loop!!");
    }
}

#[embassy_executor::task]
async fn run_audio(
    mut interface: Interface<'static>,
    mut sdram: Sdram<Fmc<'static, FMC>, FmcDevice>,
    mut record_led: Output<'static>,
    mut playback_led: Output<'static>,
) {
    let mut delay = Delay;
    let loop_buffer = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay) as *mut _;

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, SDRAM_SIZE / core::mem::size_of::<u32>())
    };

    //clear loop_buffer
    for smp in loop_buffer.iter_mut() {
        *smp = SILENCE;
    }

    let mut record_end = 0;
    let mut cursor = 0;

    let mut looper_state = LooperState::Cleared;

    interface
        .start(|input, output| {
            output.copy_from_slice(input); // Passthrough
            if RECORD_PRESSED.load(Ordering::SeqCst) {
                match looper_state {
                    LooperState::Cleared => looper_state = LooperState::Record,
                    LooperState::Record => {
                        looper_state = LooperState::Playback;
                        record_end = cursor;
                    }
                    LooperState::Playback => looper_state = LooperState::Overdub,
                    LooperState::Overdub => looper_state = LooperState::Playback,
                }
                RECORD_PRESSED.store(false, Ordering::SeqCst);
            }

            if CLEAR_PRESSED.load(Ordering::SeqCst) {
                looper_state = LooperState::Cleared;
                record_end = 0;
                cursor = 0;
                CLEAR_PRESSED.store(false, Ordering::SeqCst);
            }

            match looper_state {
                LooperState::Cleared => {}
                LooperState::Record => {
                    record(
                        output,
                        loop_buffer,
                        cursor,
                        &mut looper_state,
                        &mut record_end,
                    );
                    increase_cursor(&mut cursor, record_end);
                }
                LooperState::Playback => {
                    playback(output, loop_buffer, cursor, record_end);
                    increase_cursor(&mut cursor, record_end);
                }
                LooperState::Overdub => {
                    playback(output, loop_buffer, cursor, record_end);
                    overdub(output, loop_buffer, cursor, record_end);
                    increase_cursor(&mut cursor, record_end);
                }
            }

            match looper_state {
                LooperState::Cleared => {
                    record_led.set_low();
                    playback_led.set_low();
                }
                LooperState::Record => {
                    record_led.set_high();
                    playback_led.set_low();
                }
                LooperState::Playback => {
                    record_led.set_low();
                    playback_led.set_high();
                }
                LooperState::Overdub => {
                    record_led.set_high();
                    playback_led.set_high();
                }
            }
        })
        .await;
}

pub fn rcc_config() -> hal::Config {
    let mut config = hal::Config::default();
    use hal::rcc::*;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL240,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV20),
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL50,
        divp: Some(PllDiv::DIV1),
        divq: None,
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV6,
        mul: PllMul::MUL295,
        divp: Some(PllDiv::DIV16),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV32),
    });
    config.rcc.sys = Sysclk::PLL1_P; // 480MHz
    config.rcc.mux.fmcsel = hal::pac::rcc::vals::Fmcsel::PLL2_R; // 100MHz
    config.rcc.mux.sai1sel = hal::pac::rcc::vals::Saisel::PLL3_P; // 49.2MHz
    config.rcc.mux.usbsel = hal::pac::rcc::vals::Usbsel::PLL1_Q; // 48MHz
    config.rcc.mux.adcsel = hal::pac::rcc::vals::Adcsel::PLL2_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // 240 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.hse = Some(Hse {
        freq: hal::time::Hertz::mhz(16),
        mode: HseMode::Oscillator,
    });
    config
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    debug!("====program start====");
    let config = rcc_config();
    let p = hal::init(config);
    let mut c = cortex_m::Peripherals::take().unwrap();
    let board = new_daisy_board!(p);
    let interface = board
        .audio_peripherals
        .prepare_interface(AudioConfig { fs: Fs::Fs48000 })
        .await;
    let sdram = board.sdram.build(&mut c.MPU, &mut c.SCB);

    let led_left = Output::new(board.pins.d22, Level::Low, Speed::Low);
    let led_right = Output::new(board.pins.d23, Level::Low, Speed::Low);

    let mut fs_left = ExtiInput::new(board.pins.d26, p.EXTI11, Pull::Up);
    let fs_left_future = async {
        loop {
            fs_left.wait_for_falling_edge().await;
            info!("record pressed");
            RECORD_PRESSED.store(true, Ordering::SeqCst);
            Timer::after_millis(30).await; // debounce
            fs_left.wait_for_rising_edge().await;
            Timer::after_millis(30).await; // debounce
        }
    };
    let mut fs_right = ExtiInput::new(board.pins.d25, p.EXTI0, Pull::Up);
    let fs_right_future = async {
        loop {
            fs_right.wait_for_falling_edge().await;
            info!("clear pressed");
            CLEAR_PRESSED.store(true, Ordering::SeqCst);
            Timer::after_millis(30).await; // debounce
            fs_right.wait_for_rising_edge().await;
            Timer::after_millis(30).await; // debounce
        }
    };

    interrupt::SAI1.set_priority(Priority::P6);
    let spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
    defmt::unwrap!(spawner.spawn(run_audio(interface, sdram, led_left, led_right)));
    join(fs_left_future, fs_right_future).await;
}
