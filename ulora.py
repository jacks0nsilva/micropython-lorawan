# ulora.py adaptado para Raspberry Pi Pico + SX127x com SPIConfig e ModemConfig

from machine import SPI, Pin
import time

# --- SPIConfig Enum Simples ---
class SPIConfig:
    rp2_0 = {
        "spi_id": 0,
        "sck": 18,
        "mosi": 19,
        "miso": 16
    }
    rp2_1 = {
        "spi_id": 1,
        "sck": 10,
        "mosi": 11,
        "miso": 12
    }

# --- ModemConfig Enum Simples (placeholder) ---
class ModemConfig:
    Bw125Cr45Sf128 = 0  # Exemplo (não é usado diretamente nesse código)
    Bw500Cr45Sf128 = 1  # Exemplo

# --- Constantes dos registradores ---
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_RSSI_VALUE = 0x1A
REG_PKT_SNR_VALUE = 0x19
REG_MODEM_CONFIG_3 = 0x26
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_PAYLOAD_LENGTH = 0x22
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42
REG_PA_DAC = 0x4D

# --- Modos ---
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_SINGLE = 0x06

# --- IRQ ---
IRQ_TX_DONE_MASK = 0x08
IRQ_RX_DONE_MASK = 0x40
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20

# --- PA config ---
PA_BOOST = 0x80

class LoRa:
    def __init__(self, spi_config, interrupt, address, cs_pin, reset_pin=None, freq=868.0, tx_power=14, acks=False):
        self.interrupt = Pin(interrupt, Pin.IN)
        self.cs = Pin(cs_pin, Pin.OUT)
        self.reset = Pin(reset_pin, Pin.OUT) if reset_pin is not None else None
        self.address = address
        self.freq = freq
        self.acks = acks
        self.on_recv = None

        spi_params = spi_config
        self.spi = SPI(spi_params["spi_id"], baudrate=5000000, polarity=0, phase=0,
                       sck=Pin(spi_params["sck"]), mosi=Pin(spi_params["mosi"]), miso=Pin(spi_params["miso"]))

        self._reset()
        self._check_version()
        self._setup_radio(tx_power)
        self.set_mode_rx()

    def _reset(self):
        if self.reset:
            self.reset.value(0)
            time.sleep(0.01)
            self.reset.value(1)
            time.sleep(0.01)

    def _check_version(self):
        version = self.read_register(REG_VERSION)
        if version != 0x12:
            raise Exception("LoRa radio not found (invalid version: 0x{:02x})".format(version))

    def _setup_radio(self, tx_power):
        self.sleep()
        self.write_register(REG_FIFO_RX_BASE_ADDR, 0)
        self.write_register(REG_FIFO_TX_BASE_ADDR, 0)
        self.set_frequency(self.freq)
        self.set_tx_power(tx_power)
        self.write_register(REG_MODEM_CONFIG_3, 0x04)  # LowDataRateOptimize
        self.idle()

    def set_frequency(self, frequency):
        frf = int((frequency / 32e6) * (1 << 19))
        self.write_register(REG_FRF_MSB, (frf >> 16) & 0xff)
        self.write_register(REG_FRF_MID, (frf >> 8) & 0xff)
        self.write_register(REG_FRF_LSB, frf & 0xff)

    def set_tx_power(self, level):
        level = max(2, min(level, 17))
        self.write_register(REG_PA_CONFIG, PA_BOOST | (level - 2))

    def sleep(self):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    def idle(self):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

    def set_mode_rx(self):
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)

    def send(self, data):
        self.idle()
        self.write_register(REG_FIFO_ADDR_PTR, 0)
        for b in data:
            self.write_register(REG_FIFO, b)
        self.write_register(REG_PAYLOAD_LENGTH, len(data))
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)
        while (self.read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
            pass
        self.write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)
        self.set_mode_rx()

    def check_recv(self):
        irq_flags = self.read_register(REG_IRQ_FLAGS)
        if irq_flags & IRQ_RX_DONE_MASK:
            if irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK:
                self.write_register(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR_MASK)
                return
            payload = self._read_payload()
            if self.on_recv:
                self.on_recv(payload)
        self.write_register(REG_IRQ_FLAGS, irq_flags)

    def _read_payload(self):
        current_addr = self.read_register(REG_FIFO_RX_CURRENT_ADDR)
        self.write_register(REG_FIFO_ADDR_PTR, current_addr)
        length = self.read_register(REG_RX_NB_BYTES)
        payload = bytearray()
        for _ in range(length):
            payload.append(self.read_register(REG_FIFO))
        class Packet:
            header_from = 0
            message = bytes(payload)
            rssi = self.read_register(REG_PKT_RSSI_VALUE)
            snr = self.read_register(REG_PKT_SNR_VALUE) / 4.0
        return Packet()

    def read_register(self, address):
        self.cs.value(0)
        self.spi.write(bytearray([address & 0x7F]))
        result = self.spi.read(1)[0]
        self.cs.value(1)
        return result

    def write_register(self, address, value):
        self.cs.value(0)
        self.spi.write(bytearray([address | 0x80, value]))
        self.cs.value(1)
