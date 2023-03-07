# Serial communication

## Baudrate

The baudrate is `115200` and the communication is operating over usb between arduino and Raspberry Pi.

## Commands from raspberry

### Command frame
Commands from raspberry are sent using bytes. The line is finished by a `LF` (`\n`).

### Frame content
Bytes are hex values between 1000 and 2000 (coded on two consecutive bytes) which are direct us request on the output channel.
Channel order is :

- Thruster channel
- D fin channel
- P fin channel
- S fin channel

### Example

All to neutral:

`\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x0A`

Thruster full forward, fins full backward:

`\0x07\0xD0\0x03\0xE8\0x03\0xE8\0x53\0xE8\0x0A`

## Commands from arduino

### Command frame
Commands from raspberry are sent using bytes. The line is finished by a `LF` (`\n`).

### Frame content
Bytes are hex values between 1000 and 2000 (on three hexadecimal digits) which are direct us request from the RC captured by arduino and real generated PWM to actuators.
Channel order is :

- Thruster channel
- D fin channel
- P fin channel
- S fin channel
- RC channel 1 (Manual button)
- RC channel 2 (Timer button)
- RC channel 3 (Thruster)
- RC channel 4 (D fin)
- RC channel 5 (P fin)
- RC channel 6 (S fin)

### Example

All to neutral:

`\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x05\0xDC\0x0A`

Thruster full forward, fins full backward, in manual mode, with timer at the minimum, so the RC captured commands are commands applied to the actuators:

`\0x07\0xD0\0x03\0xE8\0x03\0xE8\0x53\0xE8\0x07\0xD0\0x03\0xE8\0x07\0xD0\0x03\0xE8\0x03\0xE8\0x53\0xE8\0x0A`

## Picocom debug

To debug the serial communication, picocom can be used with the following configuration to correctly print hex characters:

```bash
picocom --imap spchex,nrmhex,8bithex,lfcrlf /dev/ttyUSB0 -b 115200
```