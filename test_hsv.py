#!/usr/bin/env python3
import cv2
import numpy as np

# Test HSV conversion for our colors
colors_bgr = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'orange': (0, 165, 255)
}

print("BGR to HSV conversion:")
for name, bgr in colors_bgr.items():
    # Convert single pixel to HSV
    pixel = np.uint8([[bgr]])
    hsv = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[0][0]
    print(f"{name}: BGR{bgr} -> HSV({h}, {s}, {v})")