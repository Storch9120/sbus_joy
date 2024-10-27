Joystick based on Arduino Nano with two axes, 1 dpad and 1 encoder based axis.

SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 0: channel 17 (0x01)
      * Bit 1: channel 18 (0x02)
      * Bit 2: frame lost (0x04)
      * Bit 3: failsafe activated (0x08)
   * Byte[24]: SBUS footer

 pMDDL data links do not support 100k baud rate. So this has been modified to work at 115200 baud rate. Along with the necessary firmware changes in ArduPilot firmware to support SBUS as that baud rate.

 Suggestions for future development:
 => Remove the middleman device to render sbus packets. 
   * Write an HID Driver
   * Generate the SBUS packets directly from the joy board outputs.
   * Better Joy controls