use crate::constants::*;
use crate::type_defs::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

#[derive(Debug, Clone, Copy)]
pub enum State {
    Receiver,
    Transmitter,
    Standby,
}

pub struct NRF24L01 {
    spi: Spi,
    ce: CePin, // Chip enable
    csn: CsnPin, // Chip select
    state: State
}

impl NRF24L01 {
    pub fn new(spi: Spi, ce: CePin, csn: CsnPin) -> Self {
        NRF24L01 { spi, ce, csn, state: State::Standby }
    }

    pub fn init(&mut self) {
        // Start with CE low and CSN high
        self.ce.set_low().ok();
        self.csn.set_high().ok();

        // Wait for nRF24 to power up (100ms recommended)
        cortex_m::asm::delay(12_500_000); // ~100ms at 125MHz

        // Configure nRF24L01
        // 1. Power down first
        self.write_register(REG_CONFIG, CONFIG_PWR_DOWN);

        // 2. Enable auto-ack on pipe 0
        self.write_register(REG_EN_AA, EN_AA_PIPE0);

        // 3. Enable RX address on pipe 0
        self.write_register(REG_EN_RXADDR, EN_RXADDR_PIPE0);

        // 4. Set address width to 5 bytes
        self.write_register(REG_SETUP_AW, SETUP_AW_5BYTES);

        // 5. Setup auto retransmission (ARD=500µs, ARC=15)
        self.write_register(REG_SETUP_RETR, SETUP_RETR_500US_15);

        // 6. Set RF channel (2.402 GHz)
        self.write_register(REG_RF_CH, RF_CH_2402MHZ);

        // 7. Set data rate (1Mbps) and power (0dBm)
        self.write_register(REG_RF_SETUP, RF_SETUP_1MBPS_0DBM);

        // 8. Clear status flags
        self.write_register(REG_STATUS, STATUS_CLEAR_IRQ);

        // 9. Set RX address for pipe 0
        self.write_register_multi(REG_RX_ADDR_P0, &DEFAULT_ADDRESS);

        // 10. Set TX address (must match RX_ADDR_P0 for auto-ack)
        self.write_register_multi(REG_TX_ADDR, &DEFAULT_ADDRESS);

        // 11. Set payload width for pipe 0 (32 bytes)
        self.write_register(REG_RX_PW_P0, RX_PW_P0_32BYTES);

        // 12. Flush FIFOs
        self.send_command(CMD_FLUSH_TX);
        self.send_command(CMD_FLUSH_RX);

        // 13. Power up in RX mode
        self.write_register(REG_CONFIG, CONFIG_RX_MODE);

        // Wait 1.5ms for power up
        cortex_m::asm::delay(187_500); // ~1.5ms at 125MHz

        // 14. Enable receiver by setting CE high
        self.ce.set_high().ok();

        self.state = State::Receiver;
    }

    // Write to a single-byte register
    fn write_register(&mut self, reg: u8, value: u8) {
        self.csn.set_low().ok();
        self.spi.write(&[CMD_W_REGISTER | reg, value]).ok();
        self.csn.set_high().ok();
    }

    // Write to a multi-byte register
    fn write_register_multi(&mut self, reg: u8, data: &[u8]) {
        self.csn.set_low().ok();
        self.spi.write(&[CMD_W_REGISTER | reg]).ok();
        self.spi.write(data).ok();
        self.csn.set_high().ok();
    }

    pub fn read_payload(&mut self) -> [u8; 32] {
        let cmd_buf = {
            let mut b = [0u8; 33];
            b[0] = CMD_R_RX_PAYLOAD;
            b
        };
        let mut transfer_buf = [0u8; 33];
        self.csn.set_low().ok();
        self.spi.transfer(&mut transfer_buf, &cmd_buf).ok();
        self.csn.set_high().ok();
        let mut payload = [0u8; 32];
        payload.copy_from_slice(&transfer_buf[1..]);
        payload
    }

    pub fn set_receiver_mode(&mut self) {
        self.ce.set_low().ok();
        self.write_register(REG_CONFIG, CONFIG_RX_MODE);
        self.ce.set_high().ok();
        self.state = State::Receiver;
    }

    pub fn transmit(&mut self, payload: &[u8; 32]) {
        self.ce.set_low().ok();
        self.write_register(REG_CONFIG, CONFIG_TX_MODE);
        self.csn.set_low().ok();
        self.spi.write(&[CMD_W_TX_PAYLOAD]).ok();
        self.spi.write(payload).ok();
        self.csn.set_high().ok();
        // Pulse CE ≥10µs to trigger transmission
        self.ce.set_high().ok();
        cortex_m::asm::delay(1_250); // ~10µs at 125MHz
        self.ce.set_low().ok();
        self.state = State::Transmitter;
    }

    pub fn read_status(&mut self) -> u8 {
        self.send_command(CMD_NOP)
    }

    pub fn clear_interrupts(&mut self) {
        self.write_register(REG_STATUS, STATUS_CLEAR_IRQ);
    }

    // Send command
    pub fn send_command(&mut self, cmd: u8) -> u8 {
        let mut status = [0u8];
        self.csn.set_low().ok();
        self.spi.transfer(&mut status, &[cmd]).ok();
        self.csn.set_high().ok();
        status[0]
    }

    pub fn get_state(&self) -> State {
        self.state
    }
}
