//! This example demonstrates the loop playback of audio mapped on SDRAM.
//! By setting the D16 pin low, you can record audio from the SAI input.
//! It overwrites the audio buffer from start to end and automatically stops recording after 10 seconds.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use audio::run_audio;
use daisy_embassy::audio::{AudioConfig, Fs};
use daisy_embassy::hal::gpio::{Level, Output, Speed};
use daisy_embassy::hal::interrupt;
use daisy_embassy::hal::interrupt::{InterruptExt, Priority};
use daisy_embassy::hal::{exti::ExtiInput, gpio::Pull};
use daisy_embassy::{audio::HALF_DMA_BUFFER_LENGTH, hal, new_daisy_board};
use defmt::{debug, info};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::join::join;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod audio;

//take 48000(Hz) * 10(Sec) * 2(stereo)
const BLOCK_LENGTH: usize = HALF_DMA_BUFFER_LENGTH;
static RECORD_PRESSED: AtomicBool = AtomicBool::new(false);
static CLEAR_PRESSED: AtomicBool = AtomicBool::new(false);
static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
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
