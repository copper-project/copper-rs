//! PIO-based SDIO implementation, ported from rp-sdio (rp2040) to RP2350.
//! This is experimental and may require tuning for the RP2350 PIO timing.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::needless_range_loop)]

use crate::errors::SdioError;
use crate::registers::{SdCic, SdCid, SdCsd, SdOcr};
use cortex_m::singleton;
use defmt::info;
use embedded_hal_0_2::blocking::delay::DelayUs;
use embedded_io::{Read, Seek, SeekFrom, Write};
use hal::dma::SingleChannel as SingleChannelDma;
use hal::pio::{
    InstalledProgram, MovStatusConfig, PIOBuilder, PIOExt, PinDir, Running, Rx, ShiftDirection,
    StateMachine, Tx, UninitStateMachine, PIO, SM0, SM1,
};
use hal::timer::TimerDevice;
use hal::Timer;
use pio::pio_file;
use rp235x_hal as hal;

/// Timeout for SD commands, in milliseconds
pub const SD_CMD_TIMEOUT_MS: u64 = 1_000;

/// Startup time for sd card, in milliseconds
pub const SD_STARTUP_MS: u64 = 2;

/// OCR voltage range, set to 3.2-3.4
pub const SD_OCR_VOLT_RANGE: u32 = 0b0011_0000__0000_0000_0000_0000;

/// Block length for SD operations
pub const SD_BLOCK_LEN: usize = 512;

/// Size of the quaduple-crc at the end of a block
pub const SD_BLOCK_CRC_LEN: usize = 8;

/// Size of the end signal at the end of tx transmissions
pub const SD_BLOCK_END_LEN: usize = 4;

/// Size of all tx/rx transfers
pub const SD_BLOCK_LEN_TOTAL: usize = SD_BLOCK_LEN + SD_BLOCK_CRC_LEN + SD_BLOCK_END_LEN;

/// Multiplier for nibbles
pub const SD_NIBBLE_MULT: usize = 2;

/// Divider for words
pub const SD_WORD_DIV: usize = 4;

/// Index of the crc in the working block
pub const SD_CRC_IDX_MSB: usize = SD_BLOCK_LEN / SD_WORD_DIV;
pub const SD_CRC_IDX_LSB: usize = SD_CRC_IDX_MSB + 1;
pub const SD_TEND_IDX: usize = SD_CRC_IDX_LSB + 1;

/// Clock speed divider for initialization transfers (slower is safer for bring-up)
/// PIO runs from 120 MHz; divider is integer + fractional. Start very slow to prove cmd toggles.
pub const SD_CLK_DIV_INIT: u16 = 600;

/// Retry count
pub const SD_RETRIES: usize = 3;

/// CRC7 Table used for calculating all 7-bit sd-card crcs
pub static CRC7_TABLE: [u8; 256] = [
    0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e, 0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
    0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c, 0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
    0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a, 0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
    0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28, 0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
    0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6, 0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
    0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84, 0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
    0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2, 0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
    0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
    0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc, 0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
    0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce, 0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
    0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98, 0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
    0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa, 0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
    0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34, 0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
    0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06, 0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
    0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50, 0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
    0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62, 0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2,
];

/// SDIO 4bit interface struct
pub struct Sdio4bit<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> {
    dma: Option<DmaCh>,
    timer: &'a mut Timer<D>,
    sm_cmd: StateMachine<(P, SM0), Running>,
    sm_cmd_rx: Rx<(P, SM0)>,
    sm_cmd_tx: Tx<(P, SM0)>,
    sm_dat: Option<UninitStateMachine<(P, SM1)>>,
    sd_dat_base_id: u8,
    program_data_rx: Option<InstalledProgram<P>>,
    program_data_tx: Option<InstalledProgram<P>>,
    cid: SdCid,
    rca: u16,
    csd: SdCsd,
    working_block: Option<&'static mut [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV]>,
    working_block_num: u32,
    dirty: bool,
    position: u64,
    size: u32,
    sd_full_clk_div: u16,
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> Sdio4bit<'a, DmaCh, P, D> {
    pub fn new(
        pio: &mut PIO<P>,
        dma: DmaCh,
        timer: &'a mut Timer<D>,
        sm0: UninitStateMachine<(P, SM0)>,
        sm1: UninitStateMachine<(P, SM1)>,
        sd_clk_id: u8,
        sd_cmd_id: u8,
        sd_dat_base_id: u8,
        sd_full_clk_div: u16,
    ) -> Self {
        // Note: `PIOx.split()` resets the PIO block, which clears the GPIOBASE bit.
        // Callers targeting GPIO32+ must re-set GPIOBASE (bit 4) after splitting.
        info!("SDIO: setting up PIO programs");
        let program_cmd_clk =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_cmd_clk")).program;
        let program_data_rx =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_rx")).program;
        let program_data_tx =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_tx")).program;

        let program_cmd_clk = pio.install(&program_cmd_clk).unwrap();
        let program_data_rx = pio.install(&program_data_rx).unwrap();
        let program_data_tx = pio.install(&program_data_tx).unwrap();

        let (mut sm_cmd, sm_cmd_rx, sm_cmd_tx) =
            PIOBuilder::from_installed_program(program_cmd_clk)
                .set_mov_status_config(MovStatusConfig::Tx(2))
                .set_pins(sd_cmd_id, 1)
                .out_pins(sd_cmd_id, 1)
                .in_pin_base(sd_cmd_id)
                .jmp_pin(sd_cmd_id)
                .side_set_pin_base(sd_clk_id)
                .out_shift_direction(ShiftDirection::Left)
                .in_shift_direction(ShiftDirection::Left)
                .pull_threshold(32)
                .clock_divisor_fixed_point(SD_CLK_DIV_INIT, 0)
                .autopush(true)
                .autopull(true)
                .build(sm0);

        let working_block = singleton!(: [u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV] = [0xAF; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV]).unwrap();

        sm_cmd.set_pindirs([(sd_clk_id, PinDir::Output), (sd_cmd_id, PinDir::Output)]);
        let sm_cmd = sm_cmd.start();

        let sm_dat = sm1;

        Self {
            dma: Some(dma),
            timer,
            sm_cmd,
            sm_cmd_rx,
            sm_cmd_tx,
            sm_dat: Some(sm_dat),
            sd_dat_base_id,
            program_data_rx: Some(program_data_rx),
            program_data_tx: Some(program_data_tx),
            cid: SdCid::new([0; 4]),
            rca: 0,
            csd: SdCsd::new([0; 4]),
            working_block: Some(working_block),
            working_block_num: 0,
            dirty: false,
            position: 0,
            size: 0,
            sd_full_clk_div,
        }
    }

    pub fn send_command(&mut self, command: SdCmd) -> Result<SdCmdResponse, SdioError> {
        // Ensure a clean SM state each command
        self.sm_cmd.clear_fifos();
        self.sm_cmd.restart();

        if command.is_acmd() {
            self.send_command(SdCmd::AppCmd(self.rca))?;
        }

        let command_data = command.format();
        self.sm_cmd_tx.write(command_data[0]);
        self.sm_cmd_tx.write(command_data[1]);

        self.wait_response(command)
    }

    pub fn send_command_short(&mut self, command: SdCmd) -> Result<(), SdioError> {
        self.sm_cmd.clear_fifos();
        self.sm_cmd.restart();

        if command.is_acmd() {
            self.send_command(SdCmd::AppCmd(self.rca))?;
        }

        let command_data = command.format();
        self.sm_cmd_tx.write(command_data[0]);
        self.sm_cmd_tx.write(command_data[1]);

        Ok(())
    }

    pub fn wait_response(&mut self, command: SdCmd) -> Result<SdCmdResponse, SdioError> {
        let response_type = command.get_cmd_response();
        let resp_len_bits = response_type.get_response_len();

        if resp_len_bits == 0 {
            return Ok(SdCmdResponse::R0);
        }

        let resp_len = ((resp_len_bits / 32) + 1) as usize;
        let mut resp_buf: [u32; 5] = [0; 5];

        for i in 0..resp_len {
            let start = self.timer.get_counter();
            while self.sm_cmd_rx.is_empty() {
                let now = self.timer.get_counter();
                if now.checked_duration_since(start).unwrap().to_micros() > SD_CMD_TIMEOUT_MS * 1000
                {
                    return Err(SdioError::CmdTimeout {
                        cmd: command.get_cmd_index() as u8,
                        time_ms: SD_CMD_TIMEOUT_MS,
                    });
                }
            }
            resp_buf[i] = self.sm_cmd_rx.read().unwrap();
        }

        // Align the final word so higher-level parsing matches expectations
        resp_buf[resp_len - 1] <<= 32 - (resp_len_bits % 32);

        let response = match response_type {
            SdCmdResponseType::R0 => SdCmdResponse::R0,
            SdCmdResponseType::R1 => {
                let status =
                    ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                SdCmdResponse::R1(status)
            }
            SdCmdResponseType::R1b => {
                let status =
                    ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                SdCmdResponse::R1b(status)
            }
            SdCmdResponseType::R2 => {
                let mut words = [0u32; 4];
                words[0] = ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                words[1] = ((resp_buf[1] & 0x00FF_FFFF) << 8) | ((resp_buf[2] & 0xFF00_0000) >> 24);
                words[2] = ((resp_buf[2] & 0x00FF_FFFF) << 8) | ((resp_buf[3] & 0xFF00_0000) >> 24);
                words[3] = ((resp_buf[3] & 0x00FF_FFFF) << 8) | ((resp_buf[4] & 0xFF00_0000) >> 24);
                SdCmdResponse::R2(words)
            }
            SdCmdResponseType::R3 => {
                let ocr = ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                SdCmdResponse::R3(ocr)
            }
            SdCmdResponseType::R6 => {
                let rca: u16 = ((resp_buf[0] & 0x00FF_FF00) >> 8) as u16;
                SdCmdResponse::R6((rca as u32) << 16)
            }
            SdCmdResponseType::R7 => {
                let val = ((resp_buf[0] & 0x00FF_FFFF) << 8) | ((resp_buf[1] & 0xFF00_0000) >> 24);
                SdCmdResponse::R7(val)
            }
        };

        Ok(response)
    }

    pub fn wait_ready(&mut self) -> Result<(), SdioError> {
        let cmd = SdCmd::SelectDeselectCard(self.rca);

        self.send_command(cmd)?;
        let r = self.wait_response(cmd)?;
        if r != SdCmdResponse::R1b(0) {
            return Err(SdioError::WrongCmd {
                good_cmd: SdCmd::SelectDeselectCard(self.rca).get_cmd_index() as u8,
                bad_cmd: r.get_cmd_index() as u8,
            });
        }

        Ok(())
    }

    pub fn read_block(
        &mut self,
        block: u32,
        buf: &mut [u8; SD_BLOCK_LEN],
    ) -> Result<(), SdioError> {
        self.read_write_block(block, buf, SdCmd::ReadSingleBlock(block))?;
        Ok(())
    }

    pub fn write_block(&mut self, block: u32, buf: &[u8; SD_BLOCK_LEN]) -> Result<(), SdioError> {
        let mut copy = [0u8; SD_BLOCK_LEN];
        copy.copy_from_slice(buf);
        self.read_write_block(block, &mut copy, SdCmd::WriteBlock(block))?;
        Ok(())
    }

    fn read_write_block(
        &mut self,
        block: u32,
        buf: &mut [u8; SD_BLOCK_LEN],
        cmd: SdCmd,
    ) -> Result<(), SdioError> {
        let block_addr = block * SD_BLOCK_LEN as u32;

        self.send_command(cmd.with_arg(block_addr))?;
        let r = self.wait_response(cmd)?;

        match r {
            SdCmdResponse::R1(_)
            | SdCmdResponse::R1b(_)
            | SdCmdResponse::R3(_)
            | SdCmdResponse::R6(_)
            | SdCmdResponse::R7(_) => {}
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: cmd.get_cmd_index() as u8,
                    bad_cmd: r.get_cmd_index() as u8,
                })
            }
        }

        match cmd {
            SdCmd::ReadSingleBlock(_) => self.read_block_inner(buf),
            SdCmd::WriteBlock(_) => self.write_block_inner(buf),
            _ => unreachable!(),
        }
    }

    fn read_block_inner(&mut self, buf: &mut [u8; SD_BLOCK_LEN]) -> Result<(), SdioError> {
        self.sm_cmd_tx.write(SdCmd::ReadSingleBlock(0).format()[0]);
        self.sm_cmd_tx.write(SdCmd::ReadSingleBlock(0).format()[1]);

        let sm_dat_uninit = self.sm_dat.take().unwrap();
        let program_data_rx = self.program_data_rx.take().unwrap();

        let (sm_dat, mut sm_dat_rx, sm_dat_tx) =
            PIOBuilder::from_installed_program(program_data_rx)
                .set_pins(self.sd_dat_base_id, 4)
                .out_pins(self.sd_dat_base_id, 4)
                .in_pin_base(self.sd_dat_base_id)
                .jmp_pin(self.sd_dat_base_id)
                .side_set_pin_base(self.sd_dat_base_id + 4)
                .out_shift_direction(ShiftDirection::Right)
                .in_shift_direction(ShiftDirection::Left)
                .clock_divisor_fixed_point(self.sd_full_clk_div, 0)
                .autopush(true)
                .autopull(true)
                .build(sm_dat_uninit);

        let sm_dat = sm_dat.start();

        let working_block = self.working_block.take().unwrap();
        for word in working_block.iter_mut() {
            loop {
                if let Some(val) = sm_dat_rx.read() {
                    *word = val;
                    break;
                }
            }
        }

        let (sm_uninit, program_data_rx) = sm_dat.uninit(sm_dat_rx, sm_dat_tx);
        self.sm_dat = Some(sm_uninit);
        self.program_data_rx = Some(program_data_rx);

        let mut byte_block = [0u8; SD_BLOCK_LEN_TOTAL];
        for i in 0..(SD_BLOCK_LEN_TOTAL / SD_WORD_DIV) {
            let val = working_block[i].swap_bytes();
            for j in 0..SD_WORD_DIV {
                byte_block[(i * SD_WORD_DIV) + j] = (val >> (8 * j)) as u8;
            }
        }

        let mut crc_buf = [0u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV];
        crc_buf[..(SD_BLOCK_LEN / SD_WORD_DIV)].copy_from_slice(working_block);
        let crc = crc16_4bit(&crc_buf[..(SD_BLOCK_LEN / SD_WORD_DIV)]);

        let crc_msb = ((crc >> 32) as u32).swap_bytes();
        let crc_lsb = (crc as u32).swap_bytes();

        if working_block[SD_CRC_IDX_MSB] != crc_msb || working_block[SD_CRC_IDX_LSB] != crc_lsb {
            let good_crc = (crc_msb as u64) << 32 | crc_lsb as u64;
            let bad_crc =
                (working_block[SD_CRC_IDX_MSB] as u64) << 32 | working_block[SD_CRC_IDX_LSB] as u64;
            self.working_block = Some(working_block);
            return Err(SdioError::BadRxCrc16 { good_crc, bad_crc });
        }

        self.working_block = Some(working_block);
        buf.copy_from_slice(&byte_block[..SD_BLOCK_LEN]);
        Ok(())
    }

    fn write_block_inner(&mut self, buf: &mut [u8; SD_BLOCK_LEN]) -> Result<(), SdioError> {
        let mut byte_block = [0u8; SD_BLOCK_LEN_TOTAL];
        byte_block[..SD_BLOCK_LEN].copy_from_slice(buf);

        let working_block = self.working_block.take().unwrap();
        for i in 0..(SD_BLOCK_LEN_TOTAL / SD_WORD_DIV) {
            let mut val: u32 = 0;
            for j in 0..SD_WORD_DIV {
                val |= (byte_block[(i * SD_WORD_DIV) + j] as u32) << (8 * j);
            }
            working_block[i] = val.swap_bytes();
        }

        let mut crc_buf = [0u32; SD_BLOCK_LEN_TOTAL / SD_WORD_DIV];
        crc_buf[..(SD_BLOCK_LEN / SD_WORD_DIV)]
            .copy_from_slice(&working_block[..(SD_BLOCK_LEN / SD_WORD_DIV)]);
        let crc = crc16_4bit(&crc_buf[..(SD_BLOCK_LEN / SD_WORD_DIV)]);

        working_block[SD_CRC_IDX_MSB] = ((crc >> 32) as u32).swap_bytes();
        working_block[SD_CRC_IDX_LSB] = (crc as u32).swap_bytes();
        working_block[SD_TEND_IDX] = 0xFFFF_FFFF;

        let sm_dat_uninit = self.sm_dat.take().unwrap();
        let program_data_tx = self.program_data_tx.take().unwrap();
        let (sm_dat, sm_dat_rx_unused, mut sm_dat_tx) =
            PIOBuilder::from_installed_program(program_data_tx)
                .set_pins(self.sd_dat_base_id, 4)
                .out_pins(self.sd_dat_base_id, 4)
                .in_pin_base(self.sd_dat_base_id)
                .jmp_pin(self.sd_dat_base_id)
                .side_set_pin_base(self.sd_dat_base_id + 4)
                .out_shift_direction(ShiftDirection::Right)
                .in_shift_direction(ShiftDirection::Left)
                .clock_divisor_fixed_point(self.sd_full_clk_div, 0)
                .autopush(true)
                .autopull(true)
                .build(sm_dat_uninit);

        let sm_dat = sm_dat.start();

        for word in working_block.iter() {
            while !sm_dat_tx.write(*word) {}
        }

        let (sm_uninit, program_data_tx) = sm_dat.uninit(sm_dat_rx_unused, sm_dat_tx);
        self.sm_dat = Some(sm_uninit);
        self.program_data_tx = Some(program_data_tx);
        self.working_block = Some(working_block);

        let sm_cmd_tx = &mut self.sm_cmd_tx;
        sm_cmd_tx.write(SdCmd::WriteBlock(0).format()[0]);
        sm_cmd_tx.write(SdCmd::WriteBlock(0).format()[1]);

        let r = self.wait_response(SdCmd::WriteBlock(0))?;
        match r {
            SdCmdResponse::R1(res) => {
                if res & (1 << 5) > 0 {
                    return Err(SdioError::IllegalCommand {});
                }
            }
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: SdCmd::WriteBlock(0).get_cmd_index() as u8,
                    bad_cmd: r.get_cmd_index() as u8,
                })
            }
        }

        Ok(())
    }

    pub fn init_card(&mut self) -> Result<(), SdioError> {
        // Re-initialize the command state machine each attempt to avoid stale FIFO/shift state
        self.sm_cmd.clear_fifos();
        self.sm_cmd.restart();
        info!("SDIO: init_card start");
        self.timer.delay_us((SD_STARTUP_MS as u32) * 1000);

        self.send_command_short(SdCmd::GoIdleState)?;
        let r = self.wait_response(SdCmd::GoIdleState)?;
        if r != SdCmdResponse::R0 {
            return Err(SdioError::WrongCmd {
                good_cmd: SdCmd::GoIdleState.get_cmd_index() as u8,
                bad_cmd: r.get_cmd_index() as u8,
            });
        }

        let _cic = match self.send_command(SdCmd::SendIfCond(0xAA))? {
            SdCmdResponse::R7(r) => {
                let check = (r & 0xFF) as u8;
                let volt = ((r >> 8) & 0xF) as u8;
                info!(
                    "CMD8 resp raw=0x{:08x}, check=0x{:02x}, volt=0b{:04b}",
                    r, check, volt
                );
                if check != 0xAA {
                    return Err(SdioError::BadCheck {
                        good_check: 0xAA,
                        bad_check: check,
                    });
                }
                if volt != 0b1 {
                    return Err(SdioError::BadVoltage { bad_volt: volt });
                }
                SdCic {
                    supports_1p2v: (r & (1 << 12)) > 0,
                    supports_pcie: (r & (1 << 13)) > 0,
                }
            }
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: SdCmd::SendIfCond(0xAA).get_cmd_index() as u8,
                    bad_cmd: 0,
                })
            }
        };

        let ocr = loop {
            self.send_command_short(SdCmd::SdAppOpCond(true, false, false, SD_OCR_VOLT_RANGE))?;
            match self.wait_response(SdCmd::SdAppOpCond(true, false, false, SD_OCR_VOLT_RANGE))? {
                SdCmdResponse::R3(res) => {
                    let ocr = SdOcr { ocr: res };
                    if !ocr.is_busy() {
                        break ocr;
                    }
                }
                _ => {
                    return Err(SdioError::WrongCmd {
                        good_cmd: SdCmd::SdAppOpCond(true, false, false, SD_OCR_VOLT_RANGE)
                            .get_cmd_index() as u8,
                        bad_cmd: 0,
                    })
                }
            }
        };

        if ocr.get_voltage_window() & SD_OCR_VOLT_RANGE == 0 {
            return Err(SdioError::BadVoltRange {
                range: ocr.get_voltage_window(),
            });
        }

        let res = self.send_command(SdCmd::AllSendCid)?;
        match res {
            SdCmdResponse::R2(r) => self.cid = SdCid::new(r),
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: SdCmd::AllSendCid.get_cmd_index() as u8,
                    bad_cmd: res.get_cmd_index() as u8,
                })
            }
        }

        let res = self.send_command(SdCmd::SendRelativeAddr)?;
        self.rca = match res {
            SdCmdResponse::R6(r) => (r >> 16) as u16,
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: SdCmd::SendRelativeAddr.get_cmd_index() as u8,
                    bad_cmd: res.get_cmd_index() as u8,
                })
            }
        };

        let res = self.send_command(SdCmd::SendCsd(self.rca))?;
        self.csd = match res {
            SdCmdResponse::R2(r) => SdCsd::new(r),
            _ => {
                return Err(SdioError::WrongCmd {
                    good_cmd: SdCmd::SendCsd(self.rca).get_cmd_index() as u8,
                    bad_cmd: res.get_cmd_index() as u8,
                })
            }
        };

        self.send_command(SdCmd::SelectDeselectCard(self.rca))?;
        self.wait_response(SdCmd::SelectDeselectCard(self.rca))?;

        self.send_command(SdCmd::SetBusWidth(true))?;
        let r = self.wait_response(SdCmd::SetBusWidth(true))?;
        if r != SdCmdResponse::R1(0) {
            return Err(SdioError::WrongCmd {
                good_cmd: SdCmd::SetBusWidth(true).get_cmd_index() as u8,
                bad_cmd: r.get_cmd_index() as u8,
            });
        }

        self.send_command(SdCmd::SetBlockLen(SD_BLOCK_LEN as u32))?;
        let r = self.wait_response(SdCmd::SetBlockLen(SD_BLOCK_LEN as u32))?;
        if r != SdCmdResponse::R1(0) {
            return Err(SdioError::WrongCmd {
                good_cmd: SdCmd::SetBlockLen(SD_BLOCK_LEN as u32).get_cmd_index() as u8,
                bad_cmd: r.get_cmd_index() as u8,
            });
        }

        Ok(())
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> embedded_io::ErrorType
    for Sdio4bit<'a, DmaCh, P, D>
{
    type Error = SdioError;
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> Read for Sdio4bit<'a, DmaCh, P, D> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut bytes_read = 0;
        while bytes_read < buf.len() {
            let blk = (self.position / SD_BLOCK_LEN as u64) as u32;
            let mut block_buf = [0u8; SD_BLOCK_LEN];
            self.read_block(blk, &mut block_buf)?;
            let offset = (self.position % SD_BLOCK_LEN as u64) as usize;
            let n = core::cmp::min(SD_BLOCK_LEN - offset, buf.len() - bytes_read);
            buf[bytes_read..bytes_read + n].copy_from_slice(&block_buf[offset..offset + n]);
            bytes_read += n;
            self.position += n as u64;
        }
        Ok(bytes_read)
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> Write for Sdio4bit<'a, DmaCh, P, D> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut bytes_written = 0;
        while bytes_written < buf.len() {
            let blk = (self.position / SD_BLOCK_LEN as u64) as u32;
            let mut block_buf = [0u8; SD_BLOCK_LEN];
            self.read_block(blk, &mut block_buf)?;
            let offset = (self.position % SD_BLOCK_LEN as u64) as usize;
            let n = core::cmp::min(SD_BLOCK_LEN - offset, buf.len() - bytes_written);
            block_buf[offset..offset + n].copy_from_slice(&buf[bytes_written..bytes_written + n]);
            self.write_block(blk, &block_buf)?;
            bytes_written += n;
            self.position += n as u64;
        }
        Ok(bytes_written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<'a, DmaCh: SingleChannelDma, P: PIOExt, D: TimerDevice> Seek for Sdio4bit<'a, DmaCh, P, D> {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        self.position = match pos {
            SeekFrom::Start(x) => x,
            SeekFrom::End(x) => self.size as u64 + x as u64,
            SeekFrom::Current(x) => self.position + x as u64,
        };
        Ok(self.position)
    }
}

#[derive(PartialEq, Clone, Copy)]
#[repr(u16)]
pub enum SdCmd {
    GoIdleState = FLAG_R0 | 0,
    AllSendCid = FLAG_R2 | 2,
    SendRelativeAddr = FLAG_R6 | 3,
    SelectDeselectCard(u16) = FLAG_R1B | 7,
    SendIfCond(u8) = FLAG_R7 | 8,
    SendCsd(u16) = FLAG_R2 | 9,
    SetBlockLen(u32) = FLAG_R1 | 16,
    ReadSingleBlock(u32) = FLAG_R1 | 17,
    WriteBlock(u32) = FLAG_R1 | 24,
    AppCmd(u16) = FLAG_R1 | 55,
    SetBusWidth(bool) = FLAG_R1 | FLAG_ACMD | 6,
    SdAppOpCond(bool, bool, bool, u32) = FLAG_R3 | FLAG_ACMD | 41,
}

impl SdCmd {
    #[inline]
    fn flags(&self) -> u16 {
        match self {
            SdCmd::GoIdleState => FLAG_R0 | 0,
            SdCmd::AllSendCid => FLAG_R2 | 2,
            SdCmd::SendRelativeAddr => FLAG_R6 | 3,
            SdCmd::SelectDeselectCard(_) => FLAG_R1B | 7,
            SdCmd::SendIfCond(_) => FLAG_R7 | 8,
            SdCmd::SendCsd(_) => FLAG_R2 | 9,
            SdCmd::SetBlockLen(_) => FLAG_R1 | 16,
            SdCmd::ReadSingleBlock(_) => FLAG_R1 | 17,
            SdCmd::WriteBlock(_) => FLAG_R1 | 24,
            SdCmd::AppCmd(_) => FLAG_R1 | 55,
            SdCmd::SetBusWidth(_) => FLAG_R1 | FLAG_ACMD | 6,
            SdCmd::SdAppOpCond(_, _, _, _) => FLAG_R3 | FLAG_ACMD | 41,
        }
    }

    #[inline]
    pub fn get_cmd_index(&self) -> u32 {
        (self.flags() & 0x3f) as u32
    }

    pub fn get_cmd_response(&self) -> SdCmdResponseType {
        let val = self.flags();

        match val & FLAGS_R {
            FLAG_R1 => SdCmdResponseType::R1,
            FLAG_R1B => SdCmdResponseType::R1b,
            FLAG_R2 => SdCmdResponseType::R2,
            FLAG_R3 => SdCmdResponseType::R3,
            FLAG_R6 => SdCmdResponseType::R6,
            FLAG_R7 => SdCmdResponseType::R7,
            _ => SdCmdResponseType::R0,
        }
    }

    pub fn format(&self) -> [u32; 2] {
        // Build the 48-bit command packet:
        // [start=0][transmit=1][cmd_index(6)][arg(32)][crc7(7)][end=1]
        let response_type = self.get_cmd_response();
        let mut arg: u32 = 0;

        match self {
            SdCmd::GoIdleState | SdCmd::AllSendCid | SdCmd::SendRelativeAddr => {}

            SdCmd::SelectDeselectCard(rca) => {
                arg = (*rca as u32) << 16;
            }
            SdCmd::SendIfCond(test_pattern) => {
                // Voltage supplied: 2.7-3.6V (0b0001), plus check pattern in low byte.
                arg |= 0x1 << 8; // VHS = 0b0001 at bits 11..8
                arg |= *test_pattern as u32;
            }
            SdCmd::SendCsd(rca) => {
                arg = (*rca as u32) << 16;
            }
            SdCmd::SetBlockLen(len) => arg = *len,

            SdCmd::ReadSingleBlock(addr) | SdCmd::WriteBlock(addr) => arg = *addr,

            SdCmd::AppCmd(rca) => arg = (*rca as u32) << 16,

            SdCmd::SetBusWidth(wide) => {
                arg = if *wide { 2 } else { 0 };
            }
            SdCmd::SdAppOpCond(hcs, xpc, s18r, voltage_window) => {
                if *hcs {
                    arg |= 1 << 30;
                }
                if *xpc {
                    arg |= 1 << 28;
                }
                if *s18r {
                    arg |= 1 << 24;
                }
                arg |= voltage_window & 0xFFFF_FF;
            }
        }

        let cmd_idx = self.get_cmd_index() as u8 & 0x3F;
        let mut crc: u8 = 0;
        let crc_bytes = [
            0x40 | cmd_idx,           // start=0, transmit=1, command index
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
        ];
        for b in crc_bytes {
            crc = CRC7_TABLE[(crc ^ b) as usize];
        }
        crc &= 0x7F;

        let mut packet: u64 = 0;
        packet |= (crc_bytes[0] as u64) << 40;
        packet |= (crc_bytes[1] as u64) << 32;
        packet |= (crc_bytes[2] as u64) << 24;
        packet |= (crc_bytes[3] as u64) << 16;
        packet |= (crc_bytes[4] as u64) << 8;
        packet |= (crc as u64) << 1;
        packet |= 1; // end bit

        let mut data = [0u32; 2];
        // Word0: bits31-24 = number of bits minus 1 (47), bits23-0 = top 24 bits of packet.
        data[0] = (47u32 << 24) | ((packet >> 24) as u32 & 0x00FF_FFFF);
        // Word1: bits31-8 = lower 24 bits of packet, bits7-0 = response length minus 1 (or 0 if none).
        data[1] = ((packet & 0x00FF_FFFF) as u32) << 8;
        let response_len = response_type.get_response_len() as u32;
        if response_len > 0 {
            data[1] |= response_len - 1;
        }

        data
    }

    pub fn is_acmd(&self) -> bool {
        self.flags() & FLAG_ACMD > 0
    }

    pub fn with_arg(self, arg: u32) -> Self {
        match self {
            SdCmd::ReadSingleBlock(_) => SdCmd::ReadSingleBlock(arg),
            SdCmd::WriteBlock(_) => SdCmd::WriteBlock(arg),
            _ => self,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SdCmdResponseType {
    R0,
    R1,
    R1b,
    R2,
    R3,
    R6,
    R7,
}

impl SdCmdResponseType {
    pub fn get_response_len(&self) -> u8 {
        match self {
            SdCmdResponseType::R0 => 0,
            SdCmdResponseType::R1
            | SdCmdResponseType::R1b
            | SdCmdResponseType::R3
            | SdCmdResponseType::R6
            | SdCmdResponseType::R7 => 48,
            SdCmdResponseType::R2 => 136,
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
pub enum SdCmdResponse {
    R0,
    R1(u32),
    R1b(u32),
    R2([u32; 4]),
    R3(u32),
    R6(u32),
    R7(u32),
}

impl SdCmdResponse {
    pub fn get_cmd_index(&self) -> u8 {
        match self {
            SdCmdResponse::R0 => 0,
            SdCmdResponse::R1(_) => 1,
            SdCmdResponse::R1b(_) => 1,
            SdCmdResponse::R2(_) => 2,
            SdCmdResponse::R3(_) => 3,
            SdCmdResponse::R6(_) => 6,
            SdCmdResponse::R7(_) => 7,
        }
    }
}

pub const FLAG_ACMD: u16 = 0x40;
pub const FLAG_R0: u16 = 0x00;
pub const FLAG_R1: u16 = 0x80;
pub const FLAG_R1B: u16 = 0x100;
pub const FLAG_R2: u16 = 0x180;
pub const FLAG_R3: u16 = 0x200;
pub const FLAG_R6: u16 = 0x280;
pub const FLAG_R7: u16 = 0x300;
pub const FLAGS_R: u16 = 0x380;

pub fn calculate_crc7_from_words(words: &[u32], skip: usize, len: usize) -> u8 {
    let mut crc: u8 = 0;

    let start_word = skip / 4; // Words are 32 bits long
    let start_shift = 24 - ((skip % 4) * 8);

    let mut word = start_word;
    let mut shift = start_shift;

    for _ in 0..len {
        // Calculate crc
        let byte = ((words[word] >> shift) & 0xFF) as u8;
        let index = byte ^ crc;
        crc = CRC7_TABLE[index as usize];

        // Advance the shift and/or the word

        if shift == 0 {
            shift = 24;
            word += 1;
        } else {
            shift -= 8;
        }
    }

    crc
}

pub fn crc16_4bit(data: &[u32]) -> u64 {
    let mut crc = 0;
    for word in data {
        let data_in = word.swap_bytes();
        let mut data_out: u32 = (crc >> 32) as u32;
        crc <<= 32;
        data_out ^= (data_out ^ data_in) >> 16;
        let xorred: u64 = (data_out ^ data_in) as u64;
        crc ^= xorred;
        crc ^= xorred << (5 * 4);
        crc ^= xorred << (12 * 4);
    }
    crc
}
