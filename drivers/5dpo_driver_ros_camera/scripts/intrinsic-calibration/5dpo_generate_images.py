# -*- coding: utf-8 -*-
import sys
sys.path.append('/usr/local/lib/python3.8/site-packages')

import numpy as np
import cv2
import glob

cap = cv2.VideoCapture(0)

if not cap.isOpened():
  print("Cannot open camera")
  exit()

i = 0

while True:
  # Capture frame-by-frame
  ret, frame = cap.read()

  # if frame is read correctly ret is True
  if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break

  # Display the resulting frame
  print("Display image "+ str(i) + "... Press 's' save current image, " \
        "'q' to quit")
  cv2.imshow('frame', frame)

  key = cv2.waitKey()

  if key == ord('q'):
    break
  elif key == ord('s'):
    print("Saving image " + str(i) + "...")
    filename = "image" + str(i) + ".jpg"
    cv2.imwrite(filename, frame)
    i = i + 1

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
