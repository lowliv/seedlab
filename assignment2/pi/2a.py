# Name: Vlad Kazakin
# Title: Assignment 2 2a CV
# Purpose: detect aruco ids and display on lcd


# Import necessary modules
import cv2
from cv2 import aruco
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


# Setup lcd
lcdI2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcdI2c, 16, 2)
lcd.color = [255, 255, 255]


# Setup aruco
arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

parameters = aruco.DetectorParameters()
parameters.minMarkerPerimeterRate = 0.01

detector = aruco.ArucoDetector(arucoDict, parameters)


# Setup camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

shown = ""


# Loop camera processing
while True:
    ret, frame = camera.read()

    if not ret:
        continue

    # Grayscale conversion
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Display if id detected
    if ids is not None:
        ids = ids.flatten()
        message = " ".join(str(i) for i in sorted(ids))

        aruco.drawDetectedMarkers(overlay, corners, ids)

        for corner, markerId in zip(corners, ids):
            x, y = corner[0][0].astype(int)

            cv2.putText(
                overlay,
                str(markerId),
                (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1
            )
    else:
        message = "No markers found"

    # Display on lcd id or none
    if message != shown:
        lcd.clear()
        lcd.message = message
        shown = message

    # Preview render
    cv2.imshow("E2a", cv2.resize(overlay, (960, 720)))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


# Close everything
camera.release()
cv2.destroyAllWindows()
