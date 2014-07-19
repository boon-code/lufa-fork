
                   _   _ _ ___ _
                  | | | | | __/ \
                  | |_| U | _| o | - The Lightweight USB
                  |___|___|_||_n_|    Framework for AVRs
                =========================================
                          Written by Dean Camera
                  dean [at] fourwalledcubicle [dot] com

                         http://www.lufa-lib.org
                =========================================

               LUFA is donation supported. To support LUFA,
             please donate at http://www.lufa-lib.org/donate

               Released under a modified MIT license - see
                  LUFA/License.txt for license details.

                For Commercial Licensing information, see
                     http://www.lufa-lib.org/license


This package contains the complete LUFA library, demos, user-submitted
projects and bootloaders for use with compatible microcontroller models.
LUFA is a simple to use, lightweight framework which sits atop the hardware
USB controller in specific AVR microcontroller models, and allows for the
quick and easy creation of complex USB devices and hosts.

To get started, you will need to install the "Doxygen" documentation
generation tool. If you use Linux, this can be installed via the "doxygen"
package in your chosen package management tool - under Ubuntu, this can be
achieved by running the following command in the terminal:

   sudo apt-get install doxygen

Other package managers and distributions will have similar methods to
install Doxygen. In Windows, you can download a prebuilt installer for
Doxygen from its website, www.doxygen.org.

Once installed, you can then use the Doxygen tool to generate the library
documentation from the command line or terminal of your operating system. To
do this, open your terminal or command line to the root directory of the
LUFA package, and type the following command:

   make doxygen

Which will recursively generate documentation for all elements in the
library - the core, plus all demos, projects and bootloaders. Generated
documentation will then be available by opening the file "index.html" of the
created Documentation/html/ subdirectories inside each project folder.

The documentation for the library itself (but not the documentation for the
individual demos, projects or bootloaders) is also available as a separate
package from the project webpage for convenience if Doxygen cannot be
installed.


My Enhancements
===============

I added a 'monitor mode' to the USBtoSerial Project. As long as you
don't connect to the device with 1200 baud, the device will behave
like a simple USB to RS232 converter. If you change the baud-rate to
1200 baud, a textual menu (monitor) will be presented which offers
some nice features. Currently, following modes are available:

-  Slave reset: Simple digital IO pin (A0 on the Arduino Micro) will be
   toggled by pressing a. This is useful if you want to program another
   device which needs a reset.
-  Enter bootloader: Can be used to reprogram the device itself (using
   standard Arduino bootloader).
-  Measure mode: Measures pin level (on A1 on the Arduino Micro board)
   and continuously outputs measured voltage. Note that the software
   timeout is really bad, but I didn't want to use an hardware timer
   for that (therefore timing is extremely imprecise).

I use following udev rules to create symlinks for device nodes used in
program.sh:

   # My Adruino Micro
   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0037", SYMLINK+="ttyARDUINO"

   # Enhanced USBtoSerial device
   SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="204b", ATTRS{product}=="Enhanced LUFA USB-RS232 Adapter", SYMLINK+="ttyEUSB"
