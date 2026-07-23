import cv2
import time

print("Opening stream...")
cap = cv2.VideoCapture("udp://@0.0.0.0:11111", cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Failed to open stream")
else:
    print("Stream opened!")

cap.release()
