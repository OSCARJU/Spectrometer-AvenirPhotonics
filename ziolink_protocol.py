import struct

from libusb_interface import LibusbInterface


class ZioLinkProtocol:

    def __init__(self, usb_device):
        self._usb_device = LibusbInterface(usb_device)

    def open(self):
        self._usb_device.open()

    def close(self):
        self._usb_device.close()

    @property
    def isopen(self):
        return self._usb_device.is_open

    _max_rx_length = 16492

    # Send a command code and a parameter to the device, evaluate the return code and return any other received data
    def send_receive_message(self, command_code, parameter=None):
        if parameter is None:
            tx_data = struct.pack("<I", command_code)
        else:
            tx_data = struct.pack("<Ii", command_code, parameter)  # parameter as signed integer
        return self.send_bulk_receive_message(tx_data)

    # Send a byte array to the device, evaluate the return code and return any other received data
    def send_bulk_receive_message(self, tx_data):  # returns received message without return code
        self._usb_device.write(tx_data)
        rx_data = self._usb_device.read(self._max_rx_length)
        if len(rx_data) == 0:
            raise IOError("No reply from device.")
        if len(rx_data) < 4:
            raise IOError("Not enough bytes received from device.")

        if rx_data[1] == 0:
            return rx_data[4:]
        elif 1 <= rx_data[1] <= 10:
            error_messages = ["", "Unknown command code", "Invalid parameter sent to device",
                              "This operation is currently not allowed", "The device does not support this operation",
                              "Invalid passcode for this operation", "A communication error occurred",
                              "Internal error received from device", "Cannot read from device memory"]
            raise Exception(error_messages[rx_data[1]])
        else:
            raise Exception("Error code 0x" + hex(rx_data[1]) + hex(rx_data[0]) + " received from device");
