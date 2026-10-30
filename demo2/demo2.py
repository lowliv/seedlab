"""
This is the RaspberryPi component of Demo1.
The purpose is to ensure our camera is calibrated and we can effectively measure angles and distances of aruco markers to pass information to the arduino.

To use, ensure the lcd and camera are properly connected to the pi. If no .pkl files are present, run getImages.py followed by calibration.py to generate them.
Place the camera on a flat surface aiming level with the surface. place an aruco marker within detectable distance and field of view, and measure distance and angle from the camera using a method of your choice.
Run the program and compare output to measurements.
"""
import math
import struct
import pickle

import cv2
import numpy as np
from cv2 import aruco
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import busio
import board
import time
import threading
import queue

# constants for the program
ARDUINO_ADDR = 8
MARKER_FT = 1.935 / 12
FOV_DEG = 53.5

# lcd instantiation
lcdI2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcdI2c, 16, 2)
lcd.color = [255, 255, 255]

# lcd queue on separate thread
lcd_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

# gets msg from lcd queue
def lcd_worker():
    last = None

    while not stop_event.is_set():
        try:
            text = lcd_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if last == text:
            continue
        else:
            lcd.clear()
            lcd.message = text
            last = text

            time.sleep(0.5)

# writes to lcd from msg pulled from queue
def send_lcd(out):
    try:
        lcd_queue.put_nowait(out)
    except queue.Full:
        try:
            lcd_queue.get_nowait()
        except queue.Empty:
            pass

        try:
            lcd_queue.put_nowait(out)
        except queue.Full:
            pass

# starts lcd thread
lcd_thread = threading.Thread(target=lcd_worker, daemon=True)
lcd_thread.start()

# Load camera calibration
with open("calibration.pkl", "rb") as f:
    cameraMatrix, distCoeffs = pickle.load(f)

cameraMatrix = cameraMatrix.copy()

i2c = SMBus(1)
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

params = aruco.DetectorParameters()
params.minMarkerPerimeterRate = 0.01
detector = aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_6X6_50),
    params
)

# main loop
try:
    while True:
        ok, frame = cam.read()
        if not ok:
            continue

        # Correct lens distortion
        frame = cv2.undistort(frame, cameraMatrix, distCoeffs)

        # get frame dimensions and find focal length
        h, w = frame.shape[:2]
        fx_angle = (w / 2) / math.tan(math.radians(FOV_DEG / 2))
        fx_dist = cameraMatrix[0, 0]
        cx = cameraMatrix[0, 2]

        corners, ids, _ = detector.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        )

        # check for markers
        found = ids is not None
        angle = dist = 0.0
        text = "NO MARKER"

        direction = None

        # if markers find angle and distance
        if found:
            c = corners[0].reshape(4, 2)
            u = c[:, 0].mean()
            side = np.mean([
                np.linalg.norm(c[k] - c[k - 1])
                for k in range(4)
            ])

            #angle = -math.degrees(math.atan((u - w / 2) / fx_angle))
            angle = math.degrees(math.atan((cx - u) / fx_dist))
            dist = fx_angle * MARKER_FT / side
            #dist2 = fx_angle * MARKER_FT / side
            #dist = (dist1 + dist2) / 2
            text = f"{angle:+.1f} deg  {dist:.2f} ft"

            xA, yA, wA, hA = cv2.boundingRect(corners[0].reshape(4, 2))
            y1 = yA + hA // 4
            y2 = yA + 3 * hA // 4

            left_reg = frame[y1:y2, max(0, xA - wA):xA]
            right_reg = frame[y1:y2, xA + wA:min(frame.shape[1], xA + 2 * wA)]
            green_hsv = cv2.cvtColor(left_reg, cv2.COLOR_BGR2HSV)
            red_hsv = cv2.cvtColor(right_reg, cv2.COLOR_BGR2HSV)

            green = cv2.inRange(green_hsv, (40, 65, 35), (90, 255, 255))
            red = cv2.inRange(red_hsv, (0, 65, 35), (12, 255, 255))

            if cv2.countNonZero(green) / green.size > 0.2:
                direction = "left"
            elif cv2.countNonZero(red) / red.size > 0.2:
                direction = "right"
            
            print(direction)

            aruco.drawDetectedMarkers(frame, corners, ids)


        # send data to arduino
        try:
            i2c.write_i2c_block_data(
                ARDUINO_ADDR,
                0,
                list(struct.pack(
                    ">BhH",
                    found,
                    int(angle * 100),
                    int(dist * 100)
                ))
            )
        except OSError:
            pass

        # send data to lcd
        send_lcd(text)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 255), 1)
        cv2.putText(
            frame, text, (4, 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
        )
        cv2.imshow("pi only", cv2.resize(frame, (640, 480)))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# close everything on termination
finally:
    stop_event.set()
    lcd_thread.join(timeout=1)
    lcd.clear()
    cam.release()
    cv2.destroyAllWindows()
