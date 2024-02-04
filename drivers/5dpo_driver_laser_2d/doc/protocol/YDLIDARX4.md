# YDLIDARX4

## Working Modes

- **Idle mode:**
  - Default mode
  - No output of ranging data
- **Scan mode:**
  - Continuous output of ranging data
- **Stop mode:**
  - Active when error occurs (when turning on the scanner, e.g., motor does not
    rotate, laser off, etc.)
  - Automatic turns off the output of ranging data
  - Feedbacks the error code

## System Communication

### Communication mechanism

1. **Device > X4:** System command
2. **X4:** Parsing command
3. **X4:** Switch mode
4. **X4 > Device:** Reply message
5. **Device:** Parsing command
6. **Device:** Retrieve data

### System command

| System Command | Description | Mode Switch | Response Mode |
| :---: | :--- | :---: | :---: |
| 0xA5 + 0x60 | Start scanning and export point cloud data | Scan | Continuous |
| 0xA5 + 0x65 | Stop and stop scanning | Stop | None |
| 0xA5 + 0x90 | Get device information | No | Single |
| 0xA5 + 0x91 | Get device health status | No | Single |
| 0xA5 + 0x80 | Soft restart | / | No response |

### Response messages

**Types of responses**

- No response
- Single response
- Continuous response

**Response message**

| Start Sign | Response Length | Response Mode | Type Code | Content |
| :---: | :---: | :---: | :---: | :---: |
| 16 bits | 30 bits | 2 bits | 8 bits | / |

| 0 | | 2 | | 4 | | 6 | 7 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| LSB | MSB | LSB | MSB | LSB | [Mode] MSB | LSB | ... |

- **Start sign:** 0xA55A
- **Response length:** length infinite when answer mode is continuous
- **Response mode:** 2 bits
  - 0x0: single response
  - 0x1: continuous response
  - 0x2 and 0x3: undefined
- **Type code:** different system commands correspond to different types codes
- **Content:** data content (each system command has its own communication
  protocol)

## Data Protocol

**Note:** X4 data communication adopts the little-endian mode and the low-order
mode.

### Scan command (0xA5 + 0x60)

| 0 | | 2 | | 4 | | 6 | 7 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| A5 | 5A | 05 | 00 | 00 | [Mode] 40 | 81 | ... |

- **Length:** ignore!
- **Response mode:** 0x1 continuous (= high part of the 6th bit)
- **Type code:** 0x81

**Content (variable)**

| 0 | | 2 | | 4 | | 6 | | 8 | | 10 | | 12 | | .. | .. |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| PH | | CT | LSN | FSA | | LSA | | CS | | S1 | | S2 | | .. | .. |
| LSB | MSB | LSB | MSB | LSB | MSB | LSB | MSB | LSB | MSB | LSB | MSB | LSB | MSB | .. | .. |

| Content | Name | Description |
| :---: | :---: | :--- |
| PH(2B) | Package header | Fixed at 0x55AA (low in front, high in back) |
| CT(1B) | Package type | CT[bit(0)] = 1 start of a round of data; CT[bit(0)] = 0 indicates the point cloud data package. CT[bit(7:1)] are reserved bits |
| LSN(1B) | Sample quantity | Number of sampling points in current package. Only one initial point in the zero package (value = 1). |
| FSA(2B) | Start angle | Angle of the first sample point. |
| LSA(2B) | End angle | Angle of the last sample point. |
| CS(2B) | Check code | 2B exclusive OR to check the current data package. |
| Si(2B) | Sample data | Distance data of the sampling point. |

- Start bit & scan frequency analysis
  - CT[bit(0)] = 0: point cloud package
  - CT[bit(0)] = 1: start package
    - LSN = 1 (#Si = 1)
    - CT[bit(7:1)]: scan frequency information
      - F = CT[bit(7:1)]/10
- Distance analysis:
  - Di = Si / 4
- Angle analysis:
  - Data structure:
    - LSB: [Ang_q2[6:0] | C], where C is fixed at 1 (check bit)
    - MSB: [Ang_q2[14:7]]
  - 1st level: obtain initial value of the angle
    - Angle FSA = (FSA >> 1) / 64
    - Angle LSA = (LSA >> 1) / 64
    - Angle i = diff(Angle) / (LSN - 1) * (i - 1) + Angle FSA, i = 2, 3, ...,
      LSN - 1
      - diff(Angle) is clockwise angle difference from the starting angle
        (uncorrected value) to the ending angle (uncorrected value)
      - LSN is the number of package samples in the current frame
  - 2nd level: correct initial value of the angle
    - Angle i = Angle i + AngCorrect i
    - AngCorrect i:
      - (Distance i == 0) AngCorrect i = 0
      - (ELSE) AngCorrect i = tand^(-1) (21.8 * (155.3 - Distance i) /
        (155.3 * Distance i))
- Check code parsing:
  - CS = XOR 1..end (Ci)
    1. PH(2B)
    2. FSA(2B)
    3. S1(2B)
    4. ...
    5. [ CT(2B) | LSN(2B) ]
    6. LSA(2B)


### Device information (0xA5 + 0x90)

| 0 | | 2 | | 4 | | 6 | 7 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| A5 | 5A | 14 | 00 | 00 | [Mode] 00 | 04 | ... |

- **Length:** 0x00000014
- **Response mode:** 0x0 single
- **Type code:** 0x04

**Content (20B)**

| 0 | 1 | 2 | 3 | 4 | 5 | .. | 19 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Model | FW version | FW version | HW version | Serial number | Serial number |  .. | Serial number |

- **Model number (1B):** YLIDAR X4 = 0x06
- **Firmware version (2B):**
  - LSB: major version number
  - MSB: minor version number
- **Hardware version (1B)**
- **Serial number (16B):** the only factory serial number

### Health status (0xA5 + 0x91)

| 0 | | 2 | | 4 | | 6 | 7 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| A5 | 5A | 03 | 00 | 00 | [Mode] 00 | 06 | ... |

- **Length:** 0x00000003
- **Response mode:** 0x0 single
- **Type code:** 0x06

**Content (3B)**

| 0 | 1 | 2 |
| :--- | :--- | :--- |
| Status | Error code | Error code |

- **Status code (1B):**
  - 0x0: runs normally
  - 0x1: warning
  - 0x2: incorrect behavior
- **Error code (2B):**
  - 0x00: runs wo/ any error
  - Specific error code
