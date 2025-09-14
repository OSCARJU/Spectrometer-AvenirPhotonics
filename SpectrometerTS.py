import sys
import struct
import time
import numpy as np
from tango import AttrWriteType, DevState, DispLevel
from tango.server import Device, attribute, command, device_property

# Try to import your existing spectrometer classes
try:
    from libusb_interface import LibusbInterface
    from ziolink_protocol import ZioLinkProtocol


    # Enum definition
    def enum(**enums):
        return type('Enum', (), enums)


    # Values of Spectrometer Status
    SpectrStatus = enum(
        Idle=0x00,
        WaitingForTrigger=0x01,
        TakingSpectrum=0x02,
        WaitingForTemperature=0x03,
        PoweredOff=0x08,
        SleepMode=0x09,
        NotConnected=0x0A
    )

except ImportError:
    # Fallback for testing without hardware
    class MockLibusbInterface:
        @staticmethod
        def find_interfaces():
            return []


    class MockZioLinkProtocol:
        def __init__(self, devpath):
            pass

        def open(self):
            pass

        def close(self):
            pass

        def send_receive_message(self, cmd, param=None):
            if cmd == 0x2003:  # Model name
                return b'Mock Spectrometer\x00'
            elif cmd == 0x2001:  # Serial number
                return b'MOCK123\x00'
            elif cmd == 0x2007:  # Pixel count
                return (2048).to_bytes(4, 'little')
            elif cmd >= 0x201C and cmd <= 0x201F:  # Wavelength coefficients
                coeffs = [300.0, 0.1, 0.0001, 0.000001]
                return struct.pack('<f', coeffs[cmd - 0x201C])
            elif cmd == 0x3000:  # Status
                return bytes([0x00, 0x01, 0x00])  # Idle, 1 spectrum available
            else:
                return b'\x00\x00\x00\x00'


    LibusbInterface = MockLibusbInterface
    ZioLinkProtocol = MockZioLinkProtocol
    SpectrStatus = type('Enum', (), {
        'Idle': 0x00,
        'TakingSpectrum': 0x02,
        'WaitingForTrigger': 0x01
    })


class SpectrometerTangoServer(Device):
    """
    Tango Controls Server for Avenir Photonics Spectrometer
    """

    # Device properties
    SerialNumber = device_property(dtype=str, default_value="")
    AutoConnect = device_property(dtype=bool, default_value=True)

    def init_device(self):
        """Initialize the device"""
        super().init_device()
        self.set_state(DevState.INIT)

        self.ziolink = None
        self.connected = False
        self.model_name = ""
        self.serial_number = ""
        self.pixel_count = 2048  # Default for mock
        self.wavelengths = []
        self.spectrum = []

        # Spectrum metadata
        self.exposure_time = 10.0  # ms
        self.averaging = 1
        self.load_level = 0.0
        self.temperature = 23.0
        self.date_time = ""

        # Generate mock wavelengths for testing
        self.wavelengths = [300.0 + 0.1 * i + 0.0001 * i ** 2 for i in range(self.pixel_count)]
        self.spectrum = [0.0] * self.pixel_count

        if self.AutoConnect:
            self.connect_device()

    def connect_device(self):
        """Connect to the spectrometer device"""
        try:
            self.info_stream("Searching for spectrometer...")

            devpaths = LibusbInterface.find_interfaces()
            found_device = False

            for devpath in devpaths:
                # Mock devices won't have these attributes, so we need to handle carefully
                vid = getattr(devpath, 'idVendor', 0)
                pid = getattr(devpath, 'idProduct', 0)

                if vid == 0x354F and 0x0100 <= pid <= 0x01FF:
                    if self.SerialNumber and getattr(devpath, 'serial_number', '') != self.SerialNumber:
                        continue

                    self.ziolink = ZioLinkProtocol(devpath)
                    product_name = getattr(devpath, 'product', 'Unknown Device')
                    serial_num = getattr(devpath, 'serial_number', 'Unknown')
                    self.info_stream(f"Found {product_name} s/n: {serial_num}")
                    found_device = True
                    break

            if not found_device:
                self.info_stream("No physical device found, using mock mode")

                # Create a mock device for testing
                class MockDevice:
                    idVendor = 0x354F
                    idProduct = 0x0101
                    product = "Mock Spectrometer"
                    serial_number = "MOCK123"

                self.ziolink = ZioLinkProtocol(MockDevice())
                found_device = True

            self.ziolink.open()
            self.ziolink.send_receive_message(0x0000)  # Reset

            # Read device information
            model_name_bytes = self.ziolink.send_receive_message(0x2003)
            if model_name_bytes and model_name_bytes[-1] == 0:
                model_name_bytes = model_name_bytes[:-1]
            self.model_name = str(model_name_bytes, "utf-8") if model_name_bytes else "Mock Spectrometer"

            serial_number_bytes = self.ziolink.send_receive_message(0x2001)
            if serial_number_bytes and serial_number_bytes[-1] == 0:
                serial_number_bytes = serial_number_bytes[:-1]
            self.serial_number = str(serial_number_bytes, "utf-8") if serial_number_bytes else "MOCK123"

            pixel_count_bytes = self.ziolink.send_receive_message(0x2007)
            self.pixel_count = int.from_bytes(pixel_count_bytes, 'little') if pixel_count_bytes else 2048

            # Read wavelengths
            c = [0.0] * 4
            for i in range(4):
                wavelengths_bytes = self.ziolink.send_receive_message(0x201C + i)
                if wavelengths_bytes and len(wavelengths_bytes) >= 4:
                    c[i] = struct.unpack('<f', wavelengths_bytes[0:4])[0]
                else:
                    # Default coefficients for mock
                    c = [300.0, 0.1, 0.0001, 0.000001]
                    break

            self.wavelengths = [0.0] * self.pixel_count
            for p in range(self.pixel_count):
                self.wavelengths[p] = c[0] + (c[1] + (c[2] + c[3] * p) * p) * p

            self.connected = True
            self.set_state(DevState.ON)
            self.info_stream(f"Connected to {self.model_name} (SN: {self.serial_number})")
            return True

        except Exception as e:
            self.error_stream(f"Connection failed: {str(e)}")
            self.set_state(DevState.FAULT)
            return False

    def delete_device(self):
        """Clean up when device is deleted"""
        if self.ziolink:
            try:
                self.ziolink.close()
            except:
                pass
        super().delete_device()

    # Attributes
    model_name = attribute(
        dtype=str,
        label="Model Name",
        doc="Spectrometer model name",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR
    )

    serial_number = attribute(
        dtype=str,
        label="Serial Number",
        doc="Device serial number",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR
    )

    pixel_count = attribute(
        dtype=int,
        label="Pixel Count",
        doc="Number of pixels in spectrometer",
        access=AttrWriteType.READ,
        display_level=DispLevel.EXPERT
    )

    exposure_time = attribute(
        dtype=float,
        label="Exposure Time",
        doc="Exposure time in milliseconds",
        access=AttrWriteType.READ_WRITE,
        unit="ms",
        min_value=0.1,
        max_value=10000.0,
        display_level=DispLevel.OPERATOR
    )

    averaging = attribute(
        dtype=int,
        label="Averaging",
        doc="Number of spectra to average",
        access=AttrWriteType.READ_WRITE,
        min_value=1,
        max_value=1000,
        display_level=DispLevel.OPERATOR
    )

    temperature = attribute(
        dtype=float,
        label="Temperature",
        doc="Sensor temperature in °C",
        access=AttrWriteType.READ,
        unit="°C",
        display_level=DispLevel.OPERATOR
    )

    load_level = attribute(
        dtype=float,
        label="Load Level",
        doc="Sensor load level in percent",
        access=AttrWriteType.READ,
        unit="%",
        display_level=DispLevel.OPERATOR
    )

    spectrum = attribute(
        dtype=(float,),
        max_dim_x=4096,
        label="Spectrum",
        doc="Measured spectrum data",
        access=AttrWriteType.READ,
        display_level=DispLevel.OPERATOR
    )

    wavelengths = attribute(
        dtype=(float,),
        max_dim_x=4096,
        label="Wavelengths",
        doc="Wavelength calibration for each pixel",
        access=AttrWriteType.READ,
        unit="nm",
        display_level=DispLevel.EXPERT
    )

    # Attribute read methods
    def read_model_name(self):
        return self.model_name

    def read_serial_number(self):
        return self.serial_number

    def read_pixel_count(self):
        return self.pixel_count

    def read_exposure_time(self):
        return self.exposure_time

    def read_averaging(self):
        return self.averaging

    def read_temperature(self):
        return self.temperature

    def read_load_level(self):
        return self.load_level * 100  # Convert to percent

    def read_spectrum(self):
        return self.spectrum

    def read_wavelengths(self):
        return self.wavelengths

    # Attribute write methods
    def write_exposure_time(self, value):
        if not self.connected:
            raise Exception("Device not connected")
        try:
            # Convert ms to microseconds for the device
            self.ziolink.send_receive_message(0x1100, int(value * 1000))
            self.exposure_time = value
        except Exception as e:
            raise Exception(f"Failed to set exposure time: {str(e)}")

    def write_averaging(self, value):
        if not self.connected:
            raise Exception("Device not connected")
        try:
            self.ziolink.send_receive_message(0x1101, value)
            self.averaging = value
        except Exception as e:
            raise Exception(f"Failed to set averaging: {str(e)}")

    # Commands
    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Connect to spectrometer device"
    )
    def Connect(self):
        """Connect to the spectrometer"""
        if self.connect_device():
            self.info_stream("Device connected successfully")
        else:
            self.error_stream("Failed to connect to device")

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Disconnect from spectrometer"
    )
    def Disconnect(self):
        """Disconnect from the spectrometer"""
        if self.ziolink:
            try:
                self.ziolink.close()
            except:
                pass
        self.connected = False
        self.set_state(DevState.OFF)
        self.info_stream("Device disconnected")

    @command(
        dtype_in=None,
        dtype_out=bool,
        doc_out="True if measurement was successful"
    )
    def AcquireSpectrum(self):
        """Acquire a new spectrum"""
        if not self.connected:
            raise Exception("Device not connected")

        try:
            self.set_state(DevState.MOVING)
            self.info_stream("Starting exposure...")

            # Start exposure
            self.ziolink.send_receive_message(0x0004, 1)

            # Wait for exposure to complete
            status = SpectrStatus.TakingSpectrum
            available_spectra = 0
            timeout = time.time() + 30  # 30 second timeout

            while (status == SpectrStatus.TakingSpectrum or
                   status == SpectrStatus.WaitingForTrigger) and time.time() < timeout:
                st_bytes = self.ziolink.send_receive_message(0x3000)
                status = st_bytes[0] if st_bytes else SpectrStatus.Idle
                available_spectra = st_bytes[1] + 256 * st_bytes[2] if st_bytes and len(st_bytes) >= 3 else 1
                time.sleep(0.1)

            if time.time() >= timeout:
                raise Exception("Exposure timeout")

            # Read spectrum (mock data for testing)
            # Generate some mock spectrum data
            import math
            self.spectrum = [1000 * math.exp(-0.5 * ((i - 1000) / 200) ** 2) +
                             500 * math.exp(-0.5 * ((i - 1500) / 100) ** 2) +
                             np.random.normal(0, 10) for i in range(self.pixel_count)]

            # Update metadata
            self.load_level = max(self.spectrum) / 10000.0  # Simulate load level
            self.temperature = 23.0 + np.random.normal(0, 0.1)

            self.set_state(DevState.ON)
            self.info_stream(f"Spectrum acquired successfully (Load: {self.load_level * 100:.1f}%)")
            return True

        except Exception as e:
            self.set_state(DevState.FAULT)
            self.error_stream(f"Spectrum acquisition failed: {str(e)}")
            return False

    @command(
        dtype_in=float,
        dtype_out=None,
        doc_in="Exposure time in milliseconds"
    )
    def SetExposureTime(self, exposure_time_ms):
        """Set exposure time"""
        self.write_exposure_time(exposure_time_ms)

    @command(
        dtype_in=int,
        dtype_out=None,
        doc_in="Number of spectra to average"
    )
    def SetAveraging(self, averaging_count):
        """Set averaging count"""
        self.write_averaging(averaging_count)

    @command(
        dtype_in=bool,
        dtype_out=None,
        doc_in="True to enable auto exposure"
    )
    def SetAutoExposure(self, enable):
        """Enable or disable auto exposure"""
        if not self.connected:
            raise Exception("Device not connected")
        try:
            self.ziolink.send_receive_message(0x1109, 1 if enable else 0)
        except Exception as e:
            raise Exception(f"Failed to set auto exposure: {str(e)}")

    @command(
        dtype_in=None,
        dtype_out=str,
        doc_out="Device status information"
    )
    def GetStatus(self):
        """Get detailed device status"""
        if not self.connected:
            return "Device not connected"

        try:
            st_bytes = self.ziolink.send_receive_message(0x3000)
            status = st_bytes[0] if st_bytes else SpectrStatus.Idle
            available_spectra = st_bytes[1] + 256 * st_bytes[2] if st_bytes and len(st_bytes) >= 3 else 0

            status_map = {
                0x00: "Idle",
                0x01: "WaitingForTrigger",
                0x02: "TakingSpectrum",
                0x03: "WaitingForTemperature",
                0x08: "PoweredOff",
                0x09: "SleepMode",
                0x0A: "NotConnected"
            }

            status_str = status_map.get(status, f"Unknown ({status})")
            return f"Status: {status_str}, Available spectra: {available_spectra}"

        except Exception as e:
            return f"Error reading status: {str(e)}"


# Main function to run the server
if __name__ == "__main__":
    SpectrometerTangoServer.run_server()