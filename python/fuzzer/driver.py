import usb.core
import struct
import numpy as np

class Fuzzer:
    VID = 0xffff
    PID = 0x5878

    CONTROL_IN  = 0xC1  # Host to device
    CONTROL_OUT = 0x41  # Device to host

    REQUEST_STREAM_INPUT   = 0x20
    REQUEST_STREAM_OUTPUT  = 0x21
    REQUEST_CONFIGURE_PIN  = 0x22

    CONF_OUTPUT = 1
    CONF_PU     = 2
    CONF_PD     = 4
    CONF_OD     = 8

    N_PINS = 8

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
        self.dev.set_configuration()

    def stream_input_enable(self, state):
        state = int(bool(state))
        self.dev.ctrl_transfer(self.CONTROL_OUT, self.REQUEST_STREAM_INPUT, state, 0, b"")

    def stream_output_enable(self, state):
        state = int(bool(state))
        self.dev.ctrl_transfer(self.CONTROL_OUT, self.REQUEST_STREAM_OUTPUT, state, 0, b"")

    def stream_input(self, count):
        result = self.dev.read(0x81, count, timeout=int(count / 125 * 1.2 + 1000))
        values = struct.unpack("<{}B".format(len(result)), result)
        values = np.array(values, dtype=np.float)
        return values

    def stream_output(self, data):
        data = struct.pack("<{}B".format(len(data)), *list(data))
        result = self.dev.write(0x02, data, timeout=int(len(data) / 125 * 1.2 + 1000))
        assert result == len(data)

    def configure_pin(self, pin, config):
        assert isinstance(pin, int), ValueError("pin must be integer")
        assert pin >= 0, ValueError("pin must be positive")
        assert pin < self.N_PINS, ValueError("pin must be less than {}".format(self.N_PINS))
        data = struct.pack("<B", config)
        self.dev.ctrl_transfer(self.CONTROL_OUT, self.REQUEST_CONFIGURE_PIN, pin, 0, data)

    # Higher-level functions

    def raw_input(self, n_samples):
        try:
            self.stream_input_enable(True)
            return np.array(self.stream_input(n_samples), dtype=np.uint8)
        finally:
            self.stream_input_enable(False)

    def raw_output(self, samples):
        try:
            self.stream_output_enable(True)
            self.stream_output(samples)
        finally:
            self.stream_input_enable(False)

