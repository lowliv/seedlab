import math
import struct
import pickle

import cv2
import numpy as np
from cv2 import aruco
from smbus2 import SMBus

ARDUINO_ADDR = 8
MARKER_FT = 1.935 / 12
FOV_DEG = 53.5

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

while True:
    ok, frame = cam.read()
    if not ok:
        continue

    # Correct lens distortion
    frame = cv2.undistort(frame, cameraMatrix, distCoeffs)

    h, w = frame.shape[:2]
    fx = (w / 2) / math.tan(math.radians(FOV_DEG / 2))

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

        angle = -math.degrees(math.atan((u - w / 2) / fx))
        dist = fx * MARKER_FT / side
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

    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 255), 1)
    cv2.putText(
        frame, text, (4, 14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
    )
    cv2.imshow("pi only", cv2.resize(frame, (640, 480)))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()
