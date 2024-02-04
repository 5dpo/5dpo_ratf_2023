# Camera Intrinsic Calibration

## Setup

### OpenCV-Python

Follow the installation instruction available in
https://github.com/5dpo/5dpo_ratf_ros/blob/main/doc/sbc/rpi4b.md#opencv.

The Python script requires appending the following lines to indicate to consider
the newer version of OpenCV:
```py
import sys
sys.path.append('/usr/local/lib/python3.8/site-packages')
```

## Usage

```py
# Change the directory of the images

# And then, execute the command:
python3 5dpo_calibration.py
```
