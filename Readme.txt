                        BasicSpectrometer Python
                        ------------------------
            Copyright (C) 2024 Avenir Photonics GmbH & Co. KG

This is a simple demo code showing the basic steps to control a spectrometer
in Python.

It runs under Linux, Mac and Windows and was tested with Python 3.8 and
Python 3.9.

For the USB communication the 'pyusb' package needs to be installed. In Linux
you can install this package by simply entering 'pip install pyusb' in the
terminal.

Alternatively, if you are using the PyCharm IDE (available as a free download
from https://www.jetbrains.com/pycharm), you may follow these steps to get
started:

1. Choose "New Project..." from the File menu in PyCharm
2. Select the folder "BasicSpectrometer Python"
3. Uncheck "Create a main.py welcome script"
4. Click on Create, then choose "Create from Existing Sources"
5. Click on "Python Packages" at the bottom of the main window
6. Search for "pyusb", then select "pyusb" and click on "Install package"
7. Open the file "basic_spectrometer.py"
8. Click on the Run or Debug button


REQUIREMENTS FOR WINDOWS

In Windows the library file 'libusb-1.0.dll' also needs to be present
somewhere in the library search path. To install this file in Windows:
1. Download the package from:
   https://libusb.info -> Downloads -> Latest Windows binaries
2. Unpack the downloaded file
3. Locate the file VS2015-x64\dll\libusb-1.0.dll
4. Copy this file into C:\Windows\system32\

Alternatively, you can also download the library file 'libusb-1.0.dll' in
Python using "pip install libusb1".

Instead of copying this file into the Windows folder, you can also copy it
into the program folder with the demo code and uncomment four lines in the
basic_spectrometer.py file as indicated.


REQUIREMENTS FOR LINUX

Before you can use the device in Linux, you need to grant permission for the
software to access the device. This permission can be given on a per-user
basis by creating a "udev rule" file.

To access the device, simply copy the file '52-avenirphotonics.rules' into
the udev rules directory by entering the following command in the terminal:

$ sudo cp 52-avenirphotonics.rules /etc/udev/rules.d/

Then either restart your computer or enter the following commands to apply the
changes immediately:

$ sudo udevadm control --reload
$ sudo udevadm trigger

Further information on udev rules can be found at:
http://www.reactivated.net/writing_udev_rules.html


TROUBLESHOOTING

Error message "No libusb1 backend found.":
This usually means one of the following:
- The libusb library file was not installed
- This file is not in your library search path (see above)
- Only an older version of libusb (not 1.0) was found

Error message "No device found.":
- Check if you can connect to your spectrometer using the Inspective software

Error message "The device has no langid" or "Access denied" or any other error
that may indicate a problem with permissions:
- Follow the steps described above to create a udev rule file.
- After connecting the spectrometer, check the folder '/dev/' if there is a
  symlink created there with the name of your device followed by some numbers.
  If not, the udev rules were probably not correctly applied.
