# RPLIDAR

Interface protocal applicable to RPLIDAR A and S series.

## Working States

- **Idle state:**
  - Default state (enter automatically after powering up / reset)
  - No output of ranging data
- **Request processing state:**
  - Enters state after receiving a request packet from host system
  - No scan operation and no ranging data
  - Only send out response data for request needing a response
- **Protection stop state:**
  - Device hardware error
  - Host system may query the laser's working status
  - No scan operations unless host system sends a reset request to reboot the
    lidar
- **Scanning state:**
  - Only when motor rotation speed is stable the RPLIDAR starts taking distance
    measurement and sending out the result data

### Scan mode and measurement frequency (>= FWv1.24)

| Scan Mode Name | Description | Max Sample Rate (sps) |
| :---: | :--- | :---: |
| Standard | Traditional mode | 16000 (S2) |
| Express | Traditional express mode |
| Boost | Performance priority mode (sample rate) |
| Sensitivity | Sensitivity priority mode (longer range, better sensitivity) |
| Stability | Stability priority mode (environment light elimination performance) |
| DenseBoost | Dense mode | 32000 (S2) |

## Packet Format

### Request packet

| Start Flag | Command | Payload Size | Payload Data | Checksum |
| :---: | :---: | :---: | :---: | :---: |
| 1B (0xA5) | 1B | 1B (optional) | 0-255B (optional) | 1B (optional) |

- **Start flag:** 0xA5
- **Command:** type of request
- **Payload size:** required if the current request carries extra payload data
- **Checksum:** after the payload data has been transmitted, compute checksum
  field from the previous sent data
  - `checksum = 0 (+) 0xA5 (+) CmdType (+) PayloadSize (+) Payload[0] (+) ... (+) Payload[n]`,
  where `(+)` represents OR-exclusive
- **Timing consideration:** all bytes within a request packet must be
  transmitted to RPLIDAR within 5 seconds

### Response packet

- If current request requires a response, always send a response descriptor
- Only send one descriptor packet during a request/response session

**Response descriptors**

| Start Flag (1) | Start Flag (2) | Data Response Length | Send Mode | Data Type |
| :---: | :---: | :---: | :---: | :---: |
| 1B (0xA5) | 1B (0x5A) | 30bits | 2bits | 1B |

- **Start flag:** 0xA5 + 0x5A to identify the start of a response descriptor
- **Data response length:** size of a **single** incoming data response packet
  in bytes
- **Send mode:** request/response mode of the current session
  - 0x0: single request - single response (only 1 data response packet)
  - 0x1: single request - multiple response (continuous data transmission)
  - 0x2 and 0x3: reserved for future use
- **Data type:** type of the incoming data
  - Possibility of choosing different data receiving and handling policy based
    on this field
  - Each type of response data has its own data format and packet length based
    on its type

## System Communication

### System command

| Request Name | Command | Payload | Response Mode | Description |
| :--- | :---: | :---: | :---: | :--- |
| Stop | 0x25 | N/A | No response | current state > idle |
| Reset | 0x40 | N/A | No response | reset / reboot RPLIDAR |
| Scan | 0x20 | N/A | Multiple response | enter scan state |
| Express scan | 0x82 | YES | Multiple response | enter scan state + highest speed |
| Force scan | 0x21 | N/A | Multiple response | enter scan state + no check rotation stability |
| Get info | 0x50 | N/A | Single response | send device info |
| Get health | 0x52 | N/A | Single response | send device health info |
| Get sample rate | 0x59 | N/A | Single response | send single sampling time |
| Get lidar conf | 0x84 | YES | Single response | get lidar configuration |

**Stop: 0xA5 + 0x25**

- Enter idle state
- Wait at least 1ms before sending another request

**Reset: 0xA5 + 0x40**

- Make RPLIDAR core to reset / reboot itself
- Useful whel RPLIDAR enters Protection Stop state
- After reset, return to idle state
- Wait at least 2ms before sending another request

**Scan: 0xA5 + 0x20**

- Lower sampling rate
  - **Note:** use Express scan for the best performance
- If already in scan state, stop current measurement and start new round of
  scanning
- Ignored when RPLIDAR in Protection Stop state

## Data Protocol

**Note:** RPLIDAR data communication adopts the little-endian mode and the
low-order mode in values with 2B.

### Scan command (0xA5 + 0x20)

**Response Descriptor**

| 0 | 1 | 2 | 3 | 4 | 5 | 6 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| A5 | 5A | 05 | 00 | 00 | 40 | 81 |

- **Response mode:** 0x1 (single request - multiple response)
- **Data type:** 0x81

**Data Response Packet**

| 0 | 1 | 2 | 3 | 4 |
| :---: | :---: | :---: | :---: | :---: |
| QL | ANG | ANG | DIST | DIST |
| [Quality][~S][S] | [Angle LSB][C] | [Angle MSB] | [Dist LSB] | [Dist MSB] |

- Quality (6bits): quality of the current measurement sample related to the
  reflected laser pulse strength
- S: start flag bit of a new scan
  - QL[bit(0)] = 1: current and incomming packets belong to a new 360deg scan
  - Set to 1 for the first scan point of each scan frame
  - Inversed start flag bit may be used as a data check bit
- C: check bit
  - Constantly set to 1
  - May be used as a data check bit
- Angle: heading angle (deg) between 0deg and 360deg
  - Actual angle = angle / 64.0 deg
  - Clockwise (CW)
- Distance: measured object distance (mm)
  - Actual distance = distance / 4.0
  - Set to 0 when measurement invalid
