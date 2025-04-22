use core::sync::atomic::Ordering;

use crate::BLOCK_LENGTH;
use crate::CLEAR_PRESSED;
use crate::RECORD_PRESSED;
use daisy_embassy::audio::Interface;
use daisy_embassy::hal::fmc::Fmc;
use daisy_embassy::hal::gpio::Output;
use daisy_embassy::hal::peripherals::FMC;
use daisy_embassy::sdram::FmcDevice;
use daisy_embassy::sdram::SDRAM_SIZE;
use defmt::info;
use embassy_time::Delay;
use stm32_fmc::Sdram;

use {defmt_rtt as _, panic_probe as _};

const SILENCE: u32 = u32::MAX / 2;

enum LooperState {
    Start,
    Record,
    Play,
}

fn advance_cursor(cursor: &mut usize, loopback: usize) -> bool {
    *cursor += BLOCK_LENGTH;
    if *cursor >= loopback {
        info!("loop, cursor: {}, loopback: {}", cursor, loopback);
        *cursor -= loopback;
        return true;
    }
    false
}

#[embassy_executor::task]
pub async fn run_audio(
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

    // clear loop_buffer
    for smp in loop_buffer.iter_mut() {
        *smp = SILENCE;
    }

    let mut delay_1 = DelayLine::new(0, loop_buffer.len() / 4 - 1, DelayLineState::Recording);
    let mut delay_2 = DelayLine::new(
        loop_buffer.len() / 4,
        loop_buffer.len() / 4 * 2 - 1,
        DelayLineState::Playback(Iteration::Second),
    );
    let mut delay_3 = DelayLine::new(
        loop_buffer.len() / 4 * 2,
        loop_buffer.len() - 1,
        DelayLineState::Playback(Iteration::First),
    );

    let mut looper_state = LooperState::Start;
    let mut cursor = 0;
    let mut loopback = 0;
    info!(
        "loop_buffer: {:x}, {:x}",
        loop_buffer.as_ptr(),
        loop_buffer.len() * size_of::<u32>()
    );
    interface
        .start(|input, output| {
            let record_pressed = RECORD_PRESSED.swap(false, Ordering::SeqCst);
            if CLEAR_PRESSED.load(Ordering::SeqCst) {
                CLEAR_PRESSED.store(false, Ordering::SeqCst);
                looper_state = LooperState::Start;
                cursor = 0;
                loopback = 0;
                record_led.set_low();
                playback_led.set_low();
            }
            output.copy_from_slice(input); // Passthrough
            match looper_state {
                LooperState::Start => {
                    if record_pressed {
                        looper_state = LooperState::Record;
                        record_led.set_high();
                        playback_led.set_low();

                        delay_1.write(input, loop_buffer, cursor);
                        cursor += BLOCK_LENGTH;
                    }
                }
                LooperState::Record => {
                    if record_pressed {
                        loopback = cursor;

                        looper_state = LooperState::Play;
                        record_led.set_high();
                        playback_led.set_low();

                        cursor = 0;

                        delay_1.state = DelayLineState::Playback(Iteration::First);
                        delay_2.state = DelayLineState::Recording;
                        delay_3.state = DelayLineState::Playback(Iteration::Second);

                        //triggers bug
                        delay_1.execute(input, loop_buffer, output, cursor);

                        // manual inline prevents bug
                        // match &delay_1.state {
                        //     DelayLineState::Recording => {
                        //         delay_1.write(input, loop_buffer, cursor);
                        //     }
                        //     DelayLineState::Playback(_) => {
                        //         for (i, smp) in output.iter_mut().enumerate() {
                        //             let index = delay_1.start + cursor + i;
                        //             if index > delay_1.end {
                        //                 panic!("out of memory");
                        //             }
                        //             // info!("Playback: loopbuffer len: {}", loop_buffer.len());
                        //             *smp += loop_buffer[index];
                        //         }
                        //     }
                        // }

                        delay_2.execute(input, loop_buffer, output, cursor);
                        // match &delay_2.state {
                        //     DelayLineState::Recording => {
                        //         delay_2.write(input, loop_buffer, cursor);
                        //     }
                        //     DelayLineState::Playback(_) => {
                        //         for (i, smp) in output.iter_mut().enumerate() {
                        //             let index = delay_2.start + cursor + i;
                        //             if index > delay_2.end {
                        //                 panic!("out of memory");
                        //             }
                        //             // info!("Playback: loopbuffer len: {}", loop_buffer.len());
                        //             *smp += loop_buffer[index];
                        //         }
                        //     }
                        // }
                        if advance_cursor(&mut cursor, loopback) {
                            info!("Record: advancing state");
                            delay_1.advance();
                            delay_2.advance();
                            delay_3.advance();
                        }

                        return;
                    }
                    delay_1.write(input, loop_buffer, cursor);
                    cursor += BLOCK_LENGTH;
                }
                LooperState::Play => {
                    delay_1.execute(input, output, loop_buffer, cursor);
                    delay_2.execute(input, output, loop_buffer, cursor);
                    delay_3.execute(input, output, loop_buffer, cursor);
                    if advance_cursor(&mut cursor, loopback) {
                        info!("Play: advancing state");
                        delay_1.advance();
                        delay_2.advance();
                        delay_3.advance();
                    }
                }
            }
        })
        .await;
}

struct DelayLine {
    start: usize,
    end: usize,
    state: DelayLineState,
}

enum DelayLineState {
    Recording,
    Playback(Iteration),
}

enum Iteration {
    First,
    Second,
}

impl DelayLine {
    pub fn new(start: usize, end: usize, start_state: DelayLineState) -> DelayLine {
        DelayLine {
            start,
            end,
            state: start_state,
        }
    }

    pub fn execute(
        &mut self,
        input: &[u32],
        output: &mut [u32],
        loop_buffer: &mut [u32],
        cursor: usize,
    ) {
        info!(
            "loop_buffer: {:x}, {:x}",
            loop_buffer.as_ptr(),
            loop_buffer.len() * size_of::<u32>()
        );
        match &self.state {
            DelayLineState::Recording => {
                self.write(input, loop_buffer, cursor);
            }
            DelayLineState::Playback(_) => {
                for (i, smp) in output.iter_mut().enumerate() {
                    let index = self.start + cursor + i;
                    if index > self.end {
                        panic!("out of memory");
                    }
                    // info!("Playback: loopbuffer len: {}", loop_buffer.len());
                    *smp += loop_buffer[index];
                }
            }
        }
    }

    pub fn advance(&mut self) {
        self.state = match self.state {
            DelayLineState::Recording => DelayLineState::Playback(Iteration::First),
            DelayLineState::Playback(Iteration::First) => {
                DelayLineState::Playback(Iteration::Second)
            }
            DelayLineState::Playback(Iteration::Second) => DelayLineState::Recording,
        }
    }

    pub fn write(&self, input: &[u32], loop_buffer: &mut [u32], cursor: usize) {
        for (i, smp) in input.iter().enumerate() {
            let index = self.start + cursor + i;
            if index > self.end {
                panic!("out of memory");
            }
            loop_buffer[self.start + cursor + i] = *smp;
        }
    }
}
