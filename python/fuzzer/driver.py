import usb.core
import struct
import numpy as np

class Fuzzer:
    VID = 0xffff
    PID = 0x5878

    CONTROL_IN  = 0xC1  # Host to device
    CONTROL_OUT = 0x41  # Device to host

    REQUEST_STREAM              = 0x20
    REQUEST_STREAM_LOAD_PROGRAM = 0x21

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VID, idProduct=self.PID)
        self.dev.set_configuration()

    def stream_enable(self, state):
        state = int(bool(state))
        self.dev.ctrl_transfer(self.CONTROL_OUT, self.REQUEST_STREAM, state, 0, b"")

    def load_program(self, program):
        self.dev.ctrl_transfer(self.CONTROL_OUT, self.REQUEST_LOAD_PROGRAM, 0, 0, program)

    # Higher-level functions

    def raw_input(self, n_samples):
        try:
            self.stream_enable(True)
            return np.array(self.stream_input(n_samples), dtype=np.uint8)
        finally:
            self.stream_enable(False)

    def raw_output(self, samples):
        try:
            self.stream_enable(True)
            self.stream_output(samples)
        finally:
            self.stream_enable(False)

