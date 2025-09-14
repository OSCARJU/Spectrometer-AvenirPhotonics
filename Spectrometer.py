# -*- coding: utf-8 -*-
#
# This file is part of the SpectrometerCtrl project
#
#
#
# Distributed under the terms of the none license.
# See LICENSE.txt for more info.

"""
Avenir Photonics Spectrometer Controller

Implements the command control for an Avenir Photonics spectrometer device.
The device communicates via USB using the ZioLink protocol.
"""

# PROTECTED REGION ID(SpectrometerCtrl.system_imports) ENABLED START #
# PROTECTED REGION END #    //  SpectrometerCtrl.system_imports

# PyTango imports
import tango
from tango import DebugIt
from tango.server import run
from tango.server import Device
from tango.server import attribute, command
from tango.server import device_property
from tango import AttrQuality, DispLevel, DevState
from tango import AttrWriteType, PipeWriteType
# Additional import
# PROTECTED REGION ID(SpectrometerCtrl.additional_import) ENABLED START #

import sys
import struct
import time
import traceback
import threading
from enum import Enum
from json import dumps

# Import your existing spectrometer classes
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


# Trigger modes according to ZioLink protocol
class TriggerMode(Enum):
    FREE_RUNNING_END = 0  # Trigger on end of exposure
    FREE_RUNNING_START = 1  # Trigger on start of exposure
    HARDWARE_TRIGGER = 2  # Hardware trigger


# Trigger edge types
class TriggerEdge(Enum):
    RISING_EDGE = 0
    FALLING_EDGE = 1


# Acquisition modes
class AcquisitionMode(Enum):
    SINGLE = 0
    CONTINUOUS = 1


# PROTECTED REGION END #    //  SpectrometerCtrl.additional_import

__all__ = ["SpectrometerCtrl", "main"]


class SpectrometerCtrl(Device):
    """
    Implements the command control for an Avenir Photonics spectrometer device.
    The device communicates via USB using the ZioLink protocol.

    **Properties:**

    - Device Property
        SerialNumber
            - Serial number of the spectrometer to connect to (e.g., "591")
            - Type:'str'
        AutoConnect
            - Automatically connect on startup
            - Type:'bool'
    """

    # PROTECTED REGION ID(SpectrometerCtrl.class_variable) ENABLED START #
    def _probeconnection(self):
        """Try to ping spectrometer, so that either after this command a valid
        connection is possible or else, an exception is thrown."""
        try:
            if not self._connected:
                self._connect()

            # Test connection by reading model name
            model_name_bytes = self._ziolink.send_receive_message(0x2003)
            if not model_name_bytes:
                raise Exception("No response from device")

        except Exception as ex:
            # First fail, try immediate reconnect
            self.warn_stream('Connection to spectrometer not possible, immediate retry')
            self.debug_stream('Exception: {}'.format(traceback.format_exc()))
            try:
                self._disconnect()
                self._connect()
                # Test again
                model_name_bytes = self._ziolink.send_receive_message(0x2003)
                if not model_name_bytes:
                    raise Exception("No response after reconnect")
            except Exception:
                self.error_stream('Unable to connect to spectrometer.')
                self.debug_stream('Secondary Exception: {}'.format(traceback.format_exc()))
                raise ex

    def _connect(self):
        """Connect to the spectrometer device via USB"""
        try:
            self.info_stream("Searching for USB spectrometer...")

            devpaths = LibusbInterface.find_interfaces()
            found_device = False

            for devpath in devpaths:
                if devpath.idVendor == 0x354F and 0x0100 <= devpath.idProduct <= 0x01FF:
                    # Check if this matches our serial number filter
                    device_serial = str(devpath.serial_number)
                    if self.SerialNumber and device_serial != self.SerialNumber:
                        self.debug_stream(
                            f"Skipping device with serial {device_serial} (looking for {self.SerialNumber})")
                        continue

                    self._ziolink = ZioLinkProtocol(devpath)
                    self.info_stream(f"Found {devpath.product} s/n: {device_serial}")
                    found_device = True
                    break

            if not found_device:
                raise Exception(f"No USB spectrometer device found. SerialNumber filter: '{self.SerialNumber}'")

            self._ziolink.open()  # Open USB interface
            self._ziolink.send_receive_message(0x0000)  # Reset

            # Read device information
            model_name_bytes = self._ziolink.send_receive_message(0x2003)  # Get ModelName
            if model_name_bytes[-1] == 0:
                model_name_bytes = model_name_bytes[:-1]  # strip trailing null terminator
            self._model_name = str(model_name_bytes, "utf-8")

            serial_number_bytes = self._ziolink.send_receive_message(0x2001)  # Get SerialNumber
            if serial_number_bytes[-1] == 0:
                serial_number_bytes = serial_number_bytes[:-1]  # strip trailing null terminator
            self._serial_number = str(serial_number_bytes, "utf-8")

            pixel_count_bytes = self._ziolink.send_receive_message(0x2007)  # Get PixelCount
            self._pixel_count = int.from_bytes(pixel_count_bytes, 'little')

            # Read wavelengths
            c = [0] * 4
            for i in range(0, 4):
                wavelengths_bytes = self._ziolink.send_receive_message(0x201C + i)  # Get WavelengthCoeff
                c[i] = struct.unpack('<f', wavelengths_bytes[0:4])[0]

            self._wavelengths = [0] * self._pixel_count
            for p in range(0, self._pixel_count):
                self._wavelengths[p] = c[0] + (c[1] + (c[2] + c[3] * p) * p) * p

            # Initialize exposure parameters
            self._exposure_time = 10.0  # ms
            self._averaging = 1
            self._auto_exposure_enabled = False
            self._auto_exposure_time = 200  # ms
            self._trigger_mode = TriggerMode.FREE_RUNNING_START
            self._trigger_edge = TriggerEdge.RISING_EDGE
            self._trigger_delay = 0.0  # ms
            self._trigger_enabled = False
            self._acquisition_mode = AcquisitionMode.SINGLE
            self._acquisition_running = False
            self._acquisition_thread = None

            # Set default trigger configuration
            self._configure_trigger()

            self._connected = True
            self.set_state(DevState.ON)
            self.set_status('Connected')
            self.info_stream(f"Connected to {self._model_name} (SN: {self._serial_number})")
            self.info_stream(f"Wavelength range: {self._wavelengths[0]:.2f} to {self._wavelengths[-1]:.2f} nm")

        except Exception as ex:
            self.error_stream(f"Connection failed: {str(ex)}")
            raise ex

    def _disconnect(self):
        """Disconnect from the spectrometer"""
        self._stop_acquisition()  # Stop any running acquisition

        if self._ziolink:
            try:
                self._ziolink.close()
            except:
                pass
        self._connected = False
        self.set_state(DevState.OFF)
        self.set_status('Disconnected')
        self.info_stream("Device disconnected")

    def _configure_trigger(self):
        """Configure trigger settings using ZioLink protocol"""
        if not self._connected:
            return

        try:
            # Build TriggerInConfiguration value according to ZioLink protocol
            # Byte 0: Trigger Mode, Byte 1: Trigger Edge, Bytes 2-3: reserved
            trigger_config = (self._trigger_mode.value & 0xFF) | ((self._trigger_edge.value & 0xFF) << 8)

            # Set TriggerInConfiguration parameter (0x1104)
            self._ziolink.send_receive_message(0x1104, trigger_config)

            # Set TriggerInDelay parameter (0x1105) in microseconds
            self._ziolink.send_receive_message(0x1105, int(self._trigger_delay * 1000))

            # Set TriggerInEnable parameter (0x1106)
            self._ziolink.send_receive_message(0x1106, 1 if self._trigger_enabled else 0)

            self.info_stream(
                f"Trigger configured: Mode={self._trigger_mode.name}, Edge={self._trigger_edge.name}, Delay={self._trigger_delay}ms, Enabled={self._trigger_enabled}")

        except Exception as e:
            self.error_stream(f"Failed to configure trigger: {str(e)}")

    def _start_acquisition(self):
        """Start acquisition based on current mode"""
        if self._acquisition_running:
            return

        if self._acquisition_mode == AcquisitionMode.CONTINUOUS:
            self._start_continuous_acquisition()
        else:
            self._acquire_single_spectrum()

    def _start_continuous_acquisition(self):
        """Start continuous acquisition in a separate thread"""
        self._acquisition_running = True
        self._acquisition_thread = threading.Thread(target=self._acquisition_loop)
        self._acquisition_thread.daemon = True
        self._acquisition_thread.start()
        self.info_stream("Continuous acquisition started")

    def _stop_acquisition(self):
        """Stop continuous acquisition"""
        self._acquisition_running = False
        if self._acquisition_thread and self._acquisition_thread.is_alive():
            self._acquisition_thread.join(timeout=2.0)
        self.info_stream("Acquisition stopped")

    def _acquisition_loop(self):
        """Continuous acquisition loop"""
        while self._acquisition_running and self._connected:
            try:
                self._acquire_single_spectrum()
                # Small delay to prevent overwhelming the device
                time.sleep(0.1)
            except Exception as e:
                self.error_stream(f"Acquisition error: {str(e)}")
                time.sleep(1.0)  # Wait before retrying

    def _acquire_single_spectrum(self):
        """Acquire a single spectrum"""
        if not self._connected:
            return False

        try:
            # Set exposure parameters
            self._ziolink.send_receive_message(0x1100, int(self._exposure_time * 1000))  # ExposureTime in microseconds
            self._ziolink.send_receive_message(0x1101, self._averaging)  # Averaging
            self._ziolink.send_receive_message(0x1109, 1 if self._auto_exposure_enabled else 0)  # AutoExposureEnabled

            if self._auto_exposure_enabled:
                self._ziolink.send_receive_message(0x110A,
                                                   int(self._auto_exposure_time * 1000))  # AutoExposureTime in microseconds

            # Configure trigger
            self._configure_trigger()

            # Start exposure
            self._ziolink.send_receive_message(0x0004, 1)  # Start exposure with 1 spectrum

            # Wait for exposure to complete
            status = SpectrStatus.TakingSpectrum
            available_spectra = 0
            timeout = time.time() + 30  # 30 second timeout

            while (status == SpectrStatus.TakingSpectrum or
                   status == SpectrStatus.WaitingForTrigger) and time.time() < timeout:
                st_bytes = self._ziolink.send_receive_message(0x3000)
                status = st_bytes[0]
                available_spectra = st_bytes[1] + 256 * st_bytes[2]
                time.sleep(0.1)

            if time.time() >= timeout:
                raise Exception("Exposure timeout")

            # Read spectrum from device
            rawdata = self._ziolink.send_receive_message(0x4000)

            if len(rawdata) != 64 + self._pixel_count * 4:
                raise Exception("Unexpected number of bytes in received spectrum")

            # Extract metadata from spectrum header
            self._exposure_time = struct.unpack("<I", rawdata[0:4])[0] / 1000.0  # us to ms
            self._averaging = struct.unpack("<I", rawdata[4:8])[0]
            self._load_level = struct.unpack("<f", rawdata[16:20])[0]
            self._temperature = struct.unpack("<f", rawdata[20:24])[0]

            # Extract spectrum data
            self._spectrum = [0.0] * self._pixel_count
            for p in range(self._pixel_count):
                self._spectrum[p] = struct.unpack("<f", rawdata[p * 4 + 64:p * 4 + 68])[0]

            return True

        except Exception as e:
            self.error_stream(f"Spectrum acquisition failed: {str(e)}")
            return False

    # PROTECTED REGION END #    //  SpectrometerCtrl.class_variable

    # -----------------
    # Device Properties
    # -----------------

    SerialNumber = device_property(
        dtype='str',
        default_value="",
        doc="Serial number of the spectrometer to connect to (e.g., '591')"
    )

    AutoConnect = device_property(
        dtype='bool',
        default_value=True,
        doc="Automatically connect on startup"
    )

    # ----------
    # Attributes
    # ----------

    ModelName = attribute(
        dtype='str',
        access=AttrWriteType.READ,
        doc="Spectrometer model name",
        display_level=DispLevel.OPERATOR
    )

    SerialNumberAttr = attribute(
        dtype='str',
        access=AttrWriteType.READ,
        doc="Device serial number",
        display_level=DispLevel.OPERATOR
    )

    PixelCount = attribute(
        dtype='int',
        access=AttrWriteType.READ,
        doc="Number of pixels in spectrometer",
        display_level=DispLevel.EXPERT
    )

    ExposureTime = attribute(
        dtype='float',
        access=AttrWriteType.READ_WRITE,
        unit="ms",
        min_value=0.1,
        max_value=10000.0,
        doc="Exposure time in milliseconds",
        display_level=DispLevel.OPERATOR
    )

    Averaging = attribute(
        dtype='int',
        access=AttrWriteType.READ_WRITE,
        min_value=1,
        max_value=1000,
        doc="Number of spectra to average",
        display_level=DispLevel.OPERATOR
    )

    AutoExposureEnabled = attribute(
        dtype='bool',
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable auto exposure",
        display_level=DispLevel.OPERATOR
    )

    AutoExposureTime = attribute(
        dtype='float',
        access=AttrWriteType.READ_WRITE,
        unit="ms",
        min_value=0.1,
        max_value=10000.0,
        doc="Auto exposure time in milliseconds",
        display_level=DispLevel.OPERATOR
    )

    TriggerMode = attribute(
        dtype='int',
        access=AttrWriteType.READ_WRITE,
        doc="Trigger mode (0=FreeRunningEnd, 1=FreeRunningStart, 2=HardwareTrigger)",
        display_level=DispLevel.OPERATOR
    )

    TriggerEdge = attribute(
        dtype='int',
        access=AttrWriteType.READ_WRITE,
        doc="Trigger edge (0=RisingEdge, 1=FallingEdge)",
        display_level=DispLevel.OPERATOR
    )

    TriggerDelay = attribute(
        dtype='float',
        access=AttrWriteType.READ_WRITE,
        unit="ms",
        min_value=0.0,
        max_value=1000.0,
        doc="Trigger delay in milliseconds",
        display_level=DispLevel.OPERATOR
    )

    TriggerEnabled = attribute(
        dtype='bool',
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable external triggering",
        display_level=DispLevel.OPERATOR
    )

    AcquisitionMode = attribute(
        dtype='int',
        access=AttrWriteType.READ_WRITE,
        doc="Acquisition mode (0=Single, 1=Continuous)",
        display_level=DispLevel.OPERATOR
    )

    Temperature = attribute(
        dtype='float',
        access=AttrWriteType.READ,
        unit="°C",
        doc="Sensor temperature in °C",
        display_level=DispLevel.OPERATOR
    )

    LoadLevel = attribute(
        dtype='float',
        access=AttrWriteType.READ,
        unit="%",
        doc="Sensor load level in percent",
        display_level=DispLevel.OPERATOR
    )

    Spectrum = attribute(
        dtype=('float',),
        max_dim_x=4096,
        access=AttrWriteType.READ,
        doc="Measured spectrum data",
        display_level=DispLevel.OPERATOR
    )

    Wavelengths = attribute(
        dtype=('float',),
        max_dim_x=4096,
        access=AttrWriteType.READ,
        unit="nm",
        doc="Wavelength calibration for each pixel",
        display_level=DispLevel.EXPERT
    )

    Status = attribute(
        dtype='str',
        access=AttrWriteType.READ,
        doc="Detailed device status information",
        display_level=DispLevel.OPERATOR
    )

    AcquisitionRunning = attribute(
        dtype='bool',
        access=AttrWriteType.READ,
        doc="True if acquisition is running",
        display_level=DispLevel.OPERATOR
    )

    # ---------------
    # General methods
    # ---------------

    def init_device(self):
        """Initializes the attributes and properties of the SpectrometerCtrl."""
        Device.init_device(self)
        # PROTECTED REGION ID(SpectrometerCtrl.init_device) ENABLED START #

        self._ziolink = None
        self._connected = False
        self._model_name = ""
        self._serial_number = ""
        self._pixel_count = 0
        self._wavelengths = []
        self._spectrum = []
        self._exposure_time = 10.0
        self._averaging = 1
        self._auto_exposure_enabled = False
        self._auto_exposure_time = 200.0
        self._trigger_mode = TriggerMode.FREE_RUNNING_START
        self._trigger_edge = TriggerEdge.RISING_EDGE
        self._trigger_delay = 0.0
        self._trigger_enabled = False
        self._acquisition_mode = AcquisitionMode.SINGLE
        self._load_level = 0.0
        self._temperature = 23.0
        self._acquisition_running = False
        self._acquisition_thread = None

        try:
            if self.AutoConnect:
                self.info_stream('Attempting auto-connect to spectrometer')
                self._connect()
            else:
                self.set_state(DevState.OFF)
                self.set_status('AutoConnect disabled - use Connect command')

        except Exception as ex:
            """Device Server coding guidelines: Init() must never fail, but
            rather go into FAULT state and give user a chance to see error in
            Status()"""
            self.set_state(DevState.FAULT)
            self.set_status('Unexpected error during init: ' + str(ex))
            self.error_stream('Unexpected error during init, try again. Message:' + str(ex))

        # PROTECTED REGION END #    //  SpectrometerCtrl.init_device

    def always_executed_hook(self):
        """Method always executed before any TANGO command is executed."""
        # PROTECTED REGION ID(SpectrometerCtrl.always_executed_hook) ENABLED START #
        if self._connected:
            try:
                self._probeconnection()
            except:
                self.set_state(DevState.FAULT)
                self.set_status('Connection lost')
        # PROTECTED REGION END #    //  SpectrometerCtrl.always_executed_hook

    def delete_device(self):
        """Hook to delete resources allocated in init_device.

        This method allows for any memory or other resources allocated in the
        init_device method to be released.  This method is called by the device
        destructor and by the device Init command.
        """
        # PROTECTED REGION ID(SpectrometerCtrl.delete_device) ENABLED START #
        self._stop_acquisition()
        self._disconnect()
        # PROTECTED REGION END #    //  SpectrometerCtrl.delete_device

    # ------------------
    # Attributes methods
    # ------------------

    def read_ModelName(self):
        return self._model_name

    def read_SerialNumberAttr(self):
        return self._serial_number

    def read_PixelCount(self):
        return self._pixel_count

    def read_ExposureTime(self):
        return self._exposure_time

    def write_ExposureTime(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._exposure_time = value
        self.info_stream(f"Exposure time set to {value} ms")

    def read_Averaging(self):
        return self._averaging

    def write_Averaging(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._averaging = value
        self.info_stream(f"Averaging set to {value}")

    def read_AutoExposureEnabled(self):
        return self._auto_exposure_enabled

    def write_AutoExposureEnabled(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._auto_exposure_enabled = value
        status = "enabled" if value else "disabled"
        self.info_stream(f"Auto exposure {status}")

    def read_AutoExposureTime(self):
        return self._auto_exposure_time

    def write_AutoExposureTime(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._auto_exposure_time = value
        self.info_stream(f"Auto exposure time set to {value} ms")

    def read_TriggerMode(self):
        return self._trigger_mode.value

    def write_TriggerMode(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        try:
            self._trigger_mode = TriggerMode(value)
            self._configure_trigger()
        except ValueError:
            raise Exception(f"Invalid trigger mode: {value}")

    def read_TriggerEdge(self):
        return self._trigger_edge.value

    def write_TriggerEdge(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        try:
            self._trigger_edge = TriggerEdge(value)
            self._configure_trigger()
        except ValueError:
            raise Exception(f"Invalid trigger edge: {value}")

    def read_TriggerDelay(self):
        return self._trigger_delay

    def write_TriggerDelay(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._trigger_delay = value
        self._configure_trigger()
        self.info_stream(f"Trigger delay set to {value} ms")

    def read_TriggerEnabled(self):
        return self._trigger_enabled

    def write_TriggerEnabled(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        self._trigger_enabled = value
        self._configure_trigger()
        status = "enabled" if value else "disabled"
        self.info_stream(f"Trigger {status}")

    def read_AcquisitionMode(self):
        return self._acquisition_mode.value

    def write_AcquisitionMode(self, value):
        if not self._connected:
            raise Exception("Device not connected")
        try:
            self._acquisition_mode = AcquisitionMode(value)
            mode = "Single" if value == 0 else "Continuous"
            self.info_stream(f"Acquisition mode set to {mode}")
        except ValueError:
            raise Exception(f"Invalid acquisition mode: {value}")

    def read_Temperature(self):
        return self._temperature

    def read_LoadLevel(self):
        return self._load_level * 100  # Convert to percent

    def read_Spectrum(self):
        return self._spectrum

    def read_Wavelengths(self):
        return self._wavelengths

    def read_Status(self):
        if not self._connected:
            return "Device not connected"

        try:
            st_bytes = self._ziolink.send_receive_message(0x3000)
            status = st_bytes[0]
            available_spectra = st_bytes[1] + 256 * st_bytes[2]

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
            acquisition_status = "Running" if self._acquisition_running else "Stopped"
            trigger_mode = self._trigger_mode.name
            trigger_status = "Enabled" if self._trigger_enabled else "Disabled"
            return f"Status: {status_str}, Available spectra: {available_spectra}, Acquisition: {acquisition_status}, Trigger: {trigger_mode} ({trigger_status})"

        except Exception as e:
            return f"Error reading status: {str(e)}"

    def read_AcquisitionRunning(self):
        return self._acquisition_running

    # --------
    # Commands
    # --------

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Connect to spectrometer device"
    )
    @DebugIt()
    def Connect(self):
        self._connect()

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Disconnect from spectrometer"
    )
    @DebugIt()
    def Disconnect(self):
        self._disconnect()

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Reinitialize device connection"
    )
    @DebugIt()
    def Reinitialize(self):
        """Reinitialize the device connection"""
        self.info_stream("Reinitializing device connection...")
        try:
            self._disconnect()
            time.sleep(1.0)  # Brief delay
            self._connect()
            self.info_stream("Device reinitialized successfully")
        except Exception as e:
            self.error_stream(f"Reinitialization failed: {str(e)}")
            raise

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Reset device to default settings"
    )
    @DebugIt()
    def ResetDevice(self):
        """Reset device to factory defaults"""
        if not self._connected:
            raise Exception("Device not connected")

        try:
            self._ziolink.send_receive_message(0x0000)  # Reset command
            self.info_stream("Device reset to factory defaults")
            time.sleep(2.0)  # Wait for reset to complete
            self.Reinitialize()  # Reinitialize connection
        except Exception as e:
            self.error_stream(f"Device reset failed: {str(e)}")
            raise

    @command(
        dtype_in=None,
        dtype_out='bool',
        doc_out="True if measurement was successful"
    )
    @DebugIt()
    def AcquireSpectrum(self):
        """Acquire a single spectrum"""
        if not self._connected:
            raise Exception("Device not connected")

        try:
            self.set_state(DevState.MOVING)
            success = self._acquire_single_spectrum()
            self.set_state(DevState.ON)
            if success:
                self.info_stream(f"Spectrum acquired successfully (Load: {self._load_level * 100:.1f}%)")
            return success

        except Exception as e:
            self.set_state(DevState.FAULT)
            self.error_stream(f"Spectrum acquisition failed: {str(e)}")
            return False

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Start acquisition (single or continuous based on mode)"
    )
    @DebugIt()
    def StartAcquisition(self):
        """Start acquisition based on current mode"""
        if not self._connected:
            raise Exception("Device not connected")
        self._start_acquisition()

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Stop acquisition"
    )
    @DebugIt()
    def StopAcquisition(self):
        """Stop acquisition"""
        self._stop_acquisition()

    @command(
        dtype_in=None,
        dtype_out=None,
        doc_in="Send software trigger"
    )
    @DebugIt()
    def SoftwareTrigger(self):
        """Send software trigger command"""
        if not self._connected:
            raise Exception("Device not connected")

        try:
            # For software trigger, we use the standard exposure start command
            # since software triggering is handled by starting the exposure
            self._ziolink.send_receive_message(0x0004, 1)  # Start exposure with 1 spectrum
            self.info_stream("Software trigger sent - starting exposure")
        except Exception as e:
            self.error_stream(f"Software trigger failed: {str(e)}")
            raise

    @command(
        dtype_in='float',
        dtype_out=None,
        doc_in="Exposure time in milliseconds"
    )
    @DebugIt()
    def SetExposureTime(self, exposure_time_ms):
        self.write_ExposureTime(exposure_time_ms)

    @command(
        dtype_in='int',
        dtype_out=None,
        doc_in="Number of spectra to average"
    )
    @DebugIt()
    def SetAveraging(self, averaging_count):
        self.write_Averaging(averaging_count)

    @command(
        dtype_in='bool',
        dtype_out=None,
        doc_in="True to enable auto exposure"
    )
    @DebugIt()
    def SetAutoExposure(self, enable):
        self.write_AutoExposureEnabled(enable)

    @command(
        dtype_in='float',
        dtype_out=None,
        doc_in="Auto exposure time in milliseconds"
    )
    @DebugIt()
    def SetAutoExposureTime(self, exposure_time_ms):
        self.write_AutoExposureTime(exposure_time_ms)

    @command(
        dtype_in='int',
        dtype_out=None,
        doc_in="Trigger mode (0-2)"
    )
    @DebugIt()
    def SetTriggerMode(self, trigger_mode):
        self.write_TriggerMode(trigger_mode)

    @command(
        dtype_in='int',
        dtype_out=None,
        doc_in="Trigger edge (0-1)"
    )
    @DebugIt()
    def SetTriggerEdge(self, trigger_edge):
        self.write_TriggerEdge(trigger_edge)

    @command(
        dtype_in='float',
        dtype_out=None,
        doc_in="Trigger delay in milliseconds"
    )
    @DebugIt()
    def SetTriggerDelay(self, delay_ms):
        self.write_TriggerDelay(delay_ms)

    @command(
        dtype_in='bool',
        dtype_out=None,
        doc_in="Enable or disable external triggering"
    )
    @DebugIt()
    def SetTriggerEnabled(self, enable):
        self.write_TriggerEnabled(enable)

    @command(
        dtype_in='int',
        dtype_out=None,
        doc_in="Acquisition mode (0=Single, 1=Continuous)"
    )
    @DebugIt()
    def SetAcquisitionMode(self, acquisition_mode):
        self.write_AcquisitionMode(acquisition_mode)

    @command(
        dtype_in='DevString',
        dtype_out='DevString',
        doc_in="ZioLink command to send",
        doc_out="Response from device"
    )
    @DebugIt()
    def SendZioLinkCommand(self, argin):
        if not self._connected:
            raise Exception("Device not connected")

        try:
            if '=' in argin:
                cmd_str, param_str = argin.split('=', 1)
                cmd = int(cmd_str, 0)
                param = int(param_str)
                response = self._ziolink.send_receive_message(cmd, param)
            else:
                cmd = int(argin, 0)
                response = self._ziolink.send_receive_message(cmd)

            return f"Command 0x{cmd:04X}: {response.hex()}"

        except Exception as e:
            raise Exception(f"Failed to send ZioLink command: {str(e)}")


# ----------
# Run server
# ----------

def main(args=None, **kwargs):
    return run((SpectrometerCtrl,), args=args, **kwargs)


if __name__ == '__main__':
    main()