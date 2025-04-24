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

struct StatusLeds {
    led_red: Output<'static>,
    led_green: Output<'static>,
}

impl StatusLeds {
    fn new(led_red: Output<'static>, led_green: Output<'static>) -> Self {
        Self { led_red, led_green }
    }

    fn toggle_red(&mut self) {
        self.led_red.toggle();
    }

    fn display_state(&mut self, state: &CanonLooperState) {
        match state {
            CanonLooperState::Start => {
                self.led_red.set_low();
                self.led_green.set_low();
            }
            CanonLooperState::Record => {
                self.led_red.set_high();
                self.led_green.set_low();
            }
            CanonLooperState::Play => {
                self.led_red.set_high();
                self.led_green.set_high();
            }
        }
    }
}

struct CanonLooper<'a> {
    state: CanonLooperState,
    delay_lines: [DelayLine; 3],
    cursor: usize,
    loopback: usize,
    buffer: &'a mut [u32],
    status_leds: StatusLeds,
}

enum CanonLooperState {
    Start,
    Record,
    Play,
}

impl Default for CanonLooperState {
    fn default() -> Self {
        Self::Start
    }
}

impl<'a> CanonLooper<'a> {
    pub fn new(
        buffer: &'a mut [u32],
        mut led_red: Output<'static>,
        mut led_green: Output<'static>,
    ) -> Self {
        led_green.set_low();
        led_red.set_low();
        Self {
            state: Default::default(),
            delay_lines: [
                DelayLine::new(0, buffer.len() / 4 - 1, DelayLineState::Recording),
                DelayLine::new(
                    buffer.len() / 4,
                    buffer.len() / 4 * 2 - 1,
                    DelayLineState::Wait(0),
                ),
                DelayLine::new(
                    buffer.len() / 4 * 2,
                    buffer.len() - 1,
                    DelayLineState::Wait(1),
                ),
            ],
            cursor: 0,
            loopback: 0,
            buffer,
            status_leds: StatusLeds::new(led_red, led_green),
        }
    }

    fn reset(&mut self) {
        self.transition_to_state(CanonLooperState::Start);

        self.delay_lines[0].state = DelayLineState::Recording;
        self.delay_lines[1].state = DelayLineState::Wait(0);
        self.delay_lines[2].state = DelayLineState::Wait(1);

        self.cursor = 0;
        self.loopback = 0;
    }

    fn run(
        &mut self,
        record_pressed: bool,
        clear_pressed: bool,
        input: &[u32],
        output: &mut [u32],
    ) {
        if clear_pressed {
            self.reset();
        }
        match self.state {
            CanonLooperState::Start => self.run_start(record_pressed, input),
            CanonLooperState::Record => self.run_record(record_pressed, input, output),
            CanonLooperState::Play => self.run_play(input, output),
        }
    }

    fn run_start(&mut self, record_pressed: bool, input: &[u32]) {
        if record_pressed {
            self.transition_to_state(CanonLooperState::Record);

            self.delay_lines[0].write(input, self.buffer, self.cursor);
            self.cursor += BLOCK_LENGTH;
        }
    }

    fn run_record(&mut self, record_pressed: bool, input: &[u32], output: &mut [u32]) {
        if record_pressed {
            self.loopback = self.cursor;

            self.transition_to_state(CanonLooperState::Play);
            self.cursor = 0;

            self.delay_lines.iter_mut().for_each(|d| d.advance());

            self.delay_lines[0].execute(input, output, self.buffer, self.cursor);
            self.delay_lines[1].execute(input, output, self.buffer, self.cursor);
            if self.advance_cursor() {
                info!("Record: advancing state");
                self.delay_lines.iter_mut().for_each(|d| d.advance());
            }

            return;
        }
        self.delay_lines[0].write(input, self.buffer, self.cursor);
        self.cursor += BLOCK_LENGTH;
    }

    fn run_play(&mut self, input: &[u32], output: &mut [u32]) {
        self.delay_lines
            .iter_mut()
            .for_each(|d| d.execute(input, output, self.buffer, self.cursor));
        if self.advance_cursor() {
            info!("Play: advancing state");
            self.delay_lines.iter_mut().for_each(|d| d.advance());
            self.status_leds.toggle_red();
        }
    }

    fn advance_cursor(&mut self) -> bool {
        self.cursor += BLOCK_LENGTH;
        if self.cursor >= self.loopback {
            self.cursor -= self.loopback;
            return true;
        }
        false
    }

    fn transition_to_state(&mut self, new_state: CanonLooperState) {
        self.state = new_state;
        self.status_leds.display_state(&self.state);
    }
}

#[embassy_executor::task]
pub async fn run_audio(
    mut interface: Interface<'static>,
    mut sdram: Sdram<Fmc<'static, FMC>, FmcDevice>,
    led_red: Output<'static>,
    led_green: Output<'static>,
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

    let mut canon_looper = CanonLooper::new(loop_buffer, led_red, led_green);
    interface
        .start(|input, output| {
            let record_pressed = RECORD_PRESSED.swap(false, Ordering::SeqCst);
            let clear_pressed = CLEAR_PRESSED.swap(false, Ordering::SeqCst);
            output.copy_from_slice(input); // Passthrough
            canon_looper.run(record_pressed, clear_pressed, input, output);
        })
        .await;
}

struct DelayLine {
    start: usize,
    end: usize,
    state: DelayLineState,
}

enum DelayLineState {
    Wait(u32),
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
        match &self.state {
            DelayLineState::Wait(_) => {}
            DelayLineState::Recording => {
                self.write(input, loop_buffer, cursor);
            }
            DelayLineState::Playback(_) => {
                for (i, smp) in output.iter_mut().enumerate() {
                    let index = self.start + cursor + i;
                    if index > self.end {
                        panic!("out of memory");
                    }
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
            DelayLineState::Wait(i) => {
                if i == 0 {
                    DelayLineState::Recording
                } else {
                    DelayLineState::Wait(i - 1)
                }
            }
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
