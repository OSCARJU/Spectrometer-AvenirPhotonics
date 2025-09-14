import usb.core
import usb.util
import usb.backend.libusb1 as libusb1
import platform


class LibusbInterface:

    @staticmethod
    def find_interfaces():  # searches for any USB device
        libusb1backend = libusb1.get_backend()
        if libusb1backend is None:
            print('No libusb1 backend found.')
        devs_gen = usb.core.find(find_all=True, backend=libusb1backend)

        devs = list(devs_gen)  # get list from returned generator object

        for d in devs:
            usb.util.dispose_resources(d)

        return devs

    def __init__(self, usb_device):
        self._usb_device = usb_device
        self._is_open = False
        return

    def open(self):
        if platform.system() != 'Windows':
            if self._usb_device.is_kernel_driver_active(0):
                self._usb_device.detach_kernel_driver(0)
        cfg = self._usb_device.get_active_configuration()
        config_number = 1  # USB device configuration index
        if cfg is None or cfg.bConfigurationValue != config_number:
            self._usb_device.set_configuration(config_number)

        usb.util.claim_interface(self._usb_device, 0)

        self._is_open = True

    def close(self):
        usb.util.dispose_resources(self._usb_device)
        self._is_open = False

    @property
    def is_open(self):
        return self._is_open

    @property
    def device_path(self):
        return ""

    @property
    def vid(self):
        return 0

    @property
    def pid(self):
        return 0

    _timeout = 1000
    _readPipe = 0x81  # IN endpoint
    _writePipe = 0x01  # OUT endpoint

    def write(self, tx_data):
        if not self._is_open:
            raise IOError("USB connection is closed.")
        count = self._usb_device.write(self._writePipe, tx_data, self._timeout)
        if count != len(tx_data):
            raise IOError("Device write failed")

    def read(self, size):
        if not self._is_open:
            raise IOError("USB connection is closed.")
        rx_data = self._usb_device.read(self._readPipe, size, self._timeout)
        return rx_data
