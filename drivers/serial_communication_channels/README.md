# serial_communication_channels

**Version 1.0**

This folder contains a ROS package that defines a library to communicate with
another devices such as Arduinos (Uno, Mega, etc). The communication protocol is
based on the _channels_ library developed by professor Paulo Costa. The current
version is an improvement over the one developed by Héber Sobreira
(https://gitlab.inesctec.pt/heber.m.sobreira/serial_communication_channels.git)
in the way that allows channels between 'g'-'z' and 'G'-'Z'. An important note
is that you must check if on the Arduino side the hexdecimals characters
correspond to the ones consider in this package: '0'..'9','A'..'F'.

Lastly, it is possible to use the same library on the Arduino (example provided
by Héber Sobreira - please contact him for further informations on this matter).
However, this implementation uses reallocs of memory. Even though it is only 
when the Arduino turns on, I did not think that this approach would be the
correct one due to Arduinos do not have a proper memory management system. So,
the one used on the Arduino is the same one developed by professor Paulo Costa.
**Main disadvantage:** the Arduino only interprets fixed-size packets
(specifically, 4 bytes of data: `float`, `uint32_t` or `int32_t`).

**With this version, it is possible to:**

- Send data of the following types: `float`, `uint32_t` or `int32_t`
  (limited because of the implementation on the Arduino side)
- 40 different channels with different functions associated with each one
  ('g'-'z' and 'G'-'Z')

**The next version will add these features:**

- It is not expected any further developments on this library

## Communication Protocol

- `[CHANNEL]` + `[DATA]` + `\0`
- `[CHANNEL]`: 'g'-'z', 'G'-'Z'
- `[DATA]`: characters that represent a value in hexadecimal

## Contacts

If you have any questions or you want to know more about this work, please contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
