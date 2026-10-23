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

ARDUINO_ADDR = 8
MARKER_FT = 1.935 / 12
FOV_DEG = 53.5

lcdI2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcdI2c, 16, 2)
lcd.color = [255, 255, 255]

lcd_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

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


lcd_thread = threading.Thread(target=lcd_worker, daemon=True)
lcd_thread.start()

# Load camera calibration
with open("calibration.pkl", "rb") as f:
    cameraMatrix, distCoeffs = pickle.load(f)

# Scale calibration matrix from 640x480 to 320x240
cameraMatrix = cameraMatrix.copy()
cameraMatrix[:2, :] *= 0.5

i2c = SMBus(1)
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

params = aruco.DetectorParameters()
params.minMarkerPerimeterRate = 0.01
detector = aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_6X6_50),
    params
)

try:
    while True:
        ok, frame = cam.read()
        if not ok:
            continue

        # Correct lens distortion
        frame = cv2.undistort(frame, cameraMatrix, distCoeffs)

        h, w = frame.shape[:2]
        fx_angle = (w / 2) / math.tan(math.radians(FOV_DEG / 2))
        fx_dist = cameraMatrix[0, 0]
        cx = cameraMatrix[0, 2]

        corners, ids, _ = detector.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        )

        found = ids is not None
        angle = dist = 0.0
        text = "NO MARKER"

        if found:
            c = corners[0].reshape(4, 2)
            u = c[:, 0].mean()
            side = np.mean([
                np.linalg.norm(c[k] - c[k - 1])
                for k in range(4)
            ])

            angle = -math.degrees(math.atan((u - w / 2) / fx_angle))
            # angle = math.degrees(math.atan((cx - u) / fx_angle))
            dist = fx_dist * MARKER_FT / side
            text = f"{angle:+.1f} deg  {dist:.2f} ft"

            aruco.drawDetectedMarkers(frame, corners, ids)

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

        send_lcd(text)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 255), 1)
        cv2.putText(
            frame, text, (4, 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
        )
        cv2.imshow("pi only", cv2.resize(frame, (640, 480)))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    stop_event.set()
    lcd_thread.join(timeout=1)
    lcd.clear()
    cam.release()
    cv2.destroyAllWindows()
