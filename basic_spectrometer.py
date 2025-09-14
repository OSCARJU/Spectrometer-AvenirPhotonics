"""
BasicSpectrometer (Python)
-----------------
Copyright 2023 by Avenir Photonics GmbH & Co. KG

This is a very simple demo to show the essential steps for measuring a spectrum.
The spectrometer is controlled via USB.

Besides the main program, there are two more classes:
  - WinUsbInterface is a wrapper for WinUSB, the generic USB driver provided by Microsoft
    (this class was reduced to the minimum features required for this demo)
  - ZioLinkProtocol is a very simple implementaion of the ZioLink protocol.
    It uses WinUsbInterface to communicate with the device.

This demo is intentionally kept as simple as possible in order to show the basic concepts.
Any real application should probably include more error checking and a timeout.

Requires package pyusb.
"""

import sys
import struct

from libusb_interface import LibusbInterface
from ziolink_protocol import ZioLinkProtocol


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


print()
print("BasicSpectrometer")
print("-----------------")

# If you use Windows and your libusb-1.0.dll file is in the same folder as this file, uncomment the following:
# import os
# import pathlib
# libsearch = str(pathlib.Path(__file__).parent.absolute())
# os.environ['PATH'] = libsearch + os.pathsep + os.environ['PATH']

try:

    # ******************** Open USB connection to spectrometer ********************

    print("Searching for spectrometer:")

    ziolink = None
    devpaths = LibusbInterface.find_interfaces()

    for devpath in devpaths:
        # print('USB device found VID=' + hex(devpath.idVendor) + ' PID=' + hex(devpath.idProduct))

        if devpath.idVendor == 0x354F and 0x0100 <= devpath.idProduct <= 0x01FF:
            ziolink = ZioLinkProtocol(devpath)
            print("Found " + str(devpath.product) + " s/n:" + str(devpath.serial_number))
            break  # if several spectrometers ar present: only use the first one

    if ziolink is None:
        print("No device found.")
        sys.exit()
    
    ziolink.open()  # Open USB interface

    ziolink.send_receive_message(0x0000)  # 0x0000 = Send Command: Reset

    # ******************** Read device information ********************

    model_name_bytes = ziolink.send_receive_message(0x2003)  # 0x2003 = Get Device Property: ModelName
    if model_name_bytes[-1] == 0:
        model_name_bytes = model_name_bytes[:-1]  # strip trailing null terminator
    print("Model name: " + str(model_name_bytes, "utf-8"))

    serial_number_bytes = ziolink.send_receive_message(0x2001)  # 0x2001 = Get Device Property: SerialNumber
    if serial_number_bytes[-1] == 0:
        serial_number_bytes = serial_number_bytes[:-1]  # strip trailing null terminator
    print("Serial number: " + str(serial_number_bytes, "utf-8"))

    pixel_count_bytes = ziolink.send_receive_message(0x2007)  # 0x2007 = Get Device Property: PixelCount
    pixel_count = int.from_bytes(pixel_count_bytes, 'little')
    print("Number of pixels: " + str(pixel_count))

    # ******************** Read wavelengths ********************

    c = [0]*4
    for i in range(0, 4):
        wavelengths_bytes = ziolink.send_receive_message(0x201C + i)  # 0x201C = Get Device Property: WavelengthCoeff0
        c[i] = struct.unpack('<f', wavelengths_bytes[0:4])[0]

    wavelengths = [0]*pixel_count
    for p in range(0, pixel_count):
        wavelengths[p] = c[0] + (c[1] + (c[2] + c[3] * p) * p) * p
    print("Wavelength range: " + "{:.2f}".format(wavelengths[0]) + " to " +
          "{:.2f}".format(wavelengths[pixel_count - 1]) + " nm")

    # ******************** Set exposure parameters ********************

    print("Setting exposure parameters to: 20 ms, 10x averaging, no auto exposure")
    ziolink.send_receive_message(0x1100, 20000)  # 0x1100 = Set Parameter: ExposureTime (in microseconds)
    ziolink.send_receive_message(0x1101, 10)     # 0x1101 = Set Parameter: Averaging
    ziolink.send_receive_message(0x1109, 0)      # 0x1109 = Set Parameter: AutoExposureEnabled

    # ******************** Start Exposure ********************

    print("Starting exposure ...")
    ziolink.send_receive_message(0x0004, 1)  # 0x0004 = Send command: StartExposure (with parameter number of spectra)

    # ******************** Wait for exposure to be finished ********************

    status = SpectrStatus.TakingSpectrum
    available_spectra = 0
    while status == SpectrStatus.TakingSpectrum or status == SpectrStatus.WaitingForTrigger:
        st_bytes = ziolink.send_receive_message(0x3000)  # 0x3000 = Get Measured Value: Status
        status = st_bytes[0]  # byte 0 of returned value
        available_spectra = st_bytes[1] + 256 * st_bytes[2]  # //byte 1 and 2 of returned value
    print("Exposure finished. " + str(available_spectra) + " spectrum available in device.")

    # ******************** Read spectrum from device ********************

    print("Reading spectrum from device:")
    rawdata = ziolink.send_receive_message(0x4000)  # 0x4000 = Get Bulk Data: Spectrum

    if len(rawdata) != 64 + pixel_count * 4:  # should never happen
        raise Exception("Unexpected number of bytes in received spectrum.")
    # rawdata now contains the spectrum metadata (the "spectrum header") and the actual spectrum
    # see table below for the layout of this data structure

    # ******************** Display values ********************

    # Now we can read all values transmitted in the spectrum header, for example:
    load_level = struct.unpack("<f", rawdata[16:20])[0]
    print("Load level: " + "{:.2f}".format(load_level * 100) + "%")

    spectrum = [0.0] * pixel_count
    for p in range(pixel_count):
        spectrum[p] = struct.unpack("<f", rawdata[p * 4 + 64:p * 4 + 68])[0]  # Spectrum starts at byte 64

    print("First 10 spectrum values:")
    for p in range(0, 10):
        print("{:.2f}".format(wavelengths[p]) + " nm: " + str(spectrum[p]))

    print("Done.")

except Exception as e:
    print("An error occurred: " + str(e))

"""
***************** Structure of returned spectrum data ********************

Position   Data type   Description
0          uint        ExposureTime //in us (0: unknown)
4          uint        Averaging //(0: unknown)
8          uint        Time //elapsed 0.1 ms since midnight (0xFFFFFFFF = unknown)
12         uint        Date //elapsed days (0xFFFFFFFF = unknown)
16         float       LoadLevel //1.0: Saturation Level (< 0: unknown or n/a)
20         float       SensorTemperature //in °C (< 274: unknown)
24         ushort      PixelCount
26         byte        PixelFormat //0x00 (reserved for future use)
27         byte        VersionNumber //0x00 (reserved for future use)
28         ushort      ExposureSettings 
30         ushort      AppliedProcessing 
32         string      Name //up to 3 null-terminated strings: Spectrum name, intensity unit, sample name
56         float       DarkAvg
60         float       ReadoutNoise
64         float[]     Spectrum
"""
