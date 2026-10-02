import queue
import threading
import time
 
import cv2
from cv2 import aruco
from smbus2 import SMBus
 
CAMERA_INDEX = 0
FRAME_W, FRAME_H = 320, 240
PREVIEW_SIZE = (640, 480)
HYSTERESIS_PX = 6
LCD_COLS, LCD_ROWS = 16, 2
 
QUADRANT_GOALS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}
 
FONT = cv2.FONT_HERSHEY_SIMPLEX
 
i2cAr = SMBus(1)
ARDUINO_ADDR = 8
 
def open_camera():
    cam = cv2.VideoCapture(CAMERA_INDEX)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cam.isOpened():
        raise RuntimeError("camera did not open")
    return cam
 
 
class MarkerFinder:
    def __init__(self):
        params = aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_6X6_50), params)
 
    def find(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None, None, None
        # largest marker wins if more than one is visible
        best, best_area = None, 0.0
        for c, i in zip(corners, ids.flatten()):
            area = cv2.contourArea(c.reshape(4, 2))
            if area > best_area:
                best, best_area = c, area
        cx, cy = best.reshape(4, 2).mean(axis=0)
        return (float(cx), float(cy)), corners, ids
 
 
class QuadrantTracker:
    # tracks N/S and E/W separately with hysteresis so the marker sitting on a centre line doesn't flicker the quadrant back and forth
    def __init__(self, hysteresis=HYSTERESIS_PX):
        self.hyst = hysteresis
        self._east = None
        self._north = None
 
    def update(self, x, y, w, h):
        cx, cy = w / 2, h / 2
        if self._east is None:
            self._east = x >= cx
        elif self._east and x < cx - self.hyst:
            self._east = False
        elif not self._east and x > cx + self.hyst:
            self._east = True
 
        if self._north is None:
            self._north = y < cy
        elif self._north and y > cy + self.hyst:
            self._north = False
        elif not self._north and y < cy - self.hyst:
            self._north = True
 
        return ("N" if self._north else "S") + ("E" if self._east else "W")
 
 
class LcdDisplay:
    # writes on its own thread
    HEADER = "goal:"
 
    def __init__(self):
        import board
        import busio
        import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
 
        i2c = busio.I2C(board.SCL, board.SDA)
        self._lcd = character_lcd.Character_LCD_RGB_I2C(i2c, LCD_COLS, LCD_ROWS)
        self._lcd.color = [255, 255, 255]
        self._lcd.clear()
        self._lcd.message = self.HEADER
 
        self._queue = queue.Queue()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
 
    def show_goal(self, left, right):
        self._queue.put(f"{left} {right}")
 
    def close(self):
        self._queue.put(None)
        self._thread.join(timeout=2)
 
    def _worker(self):
        shown = None
        while True:
            text = self._queue.get()
            try:  # skip to newest msg in queue if there is a backlog
                while True:
                    text = self._queue.get_nowait()
            except queue.Empty:
                pass
            if text is None:
                break
            if text == shown:
                continue
            try:
                self._lcd.cursor_position(0, 1)
                self._lcd.message = text.ljust(LCD_COLS)
                shown = text
            except OSError as e:
                print(f"lcd write failed: {e}")
        try:
            self._lcd.clear()
            self._lcd.color = [0, 0, 0]
        except OSError:
            pass
 
 
def draw_overlay(img, quadrant, center, corners, ids, status):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
 
    if quadrant is not None:
        x0 = cx if quadrant[1] == "E" else 0
        y0 = 0 if quadrant[0] == "N" else cy
        region = img[y0:y0 + cy, x0:x0 + cx]
        tint = region.copy()
        tint[:] = (0, 165, 255)
        img[y0:y0 + cy, x0:x0 + cx] = cv2.addWeighted(tint, 0.25, region, 0.75, 0)
 
    cv2.line(img, (cx, 0), (cx, h), (255, 255, 255), 1)
    cv2.line(img, (0, cy), (w, cy), (255, 255, 255), 1)
    for label, pos in (("NW", (4, 14)), ("NE", (w - 26, 14)), ("SW", (4, h - 6)), ("SE", (w - 26, h - 6))):
        cv2.putText(img, label, pos, FONT, 0.4, (255, 255, 255), 1)
 
    if ids is not None:
        aruco.drawDetectedMarkers(img, corners, ids)
    if center is not None:
        px, py = int(center[0]), int(center[1])
        cv2.circle(img, (px, py), 4, (0, 0, 255), -1)
        cv2.putText(img, f"({px},{py})", (px + 6, max(py - 6, 12)), FONT, 0.4, (0, 0, 255), 1)
 
    (tw, _), _ = cv2.getTextSize(status, FONT, 0.45, 1)
    cv2.putText(img, status, (max((w - tw) // 2, 0), h - 6), FONT, 0.45, (0, 255, 255), 1)
 
 
def main():
    camera = open_camera()
    finder = MarkerFinder()
    tracker = QuadrantTracker()
    lcd = LcdDisplay()
 
    goal = (0, 0)
    quadrant = None
    lcd.show_goal(*goal)
 
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                continue
 
            center, corners, ids = finder.find(frame)
            if center is not None:
                h, w = frame.shape[:2]
                quadrant = tracker.update(center[0], center[1], w, h)
                new_goal = QUADRANT_GOALS[quadrant]
                if new_goal != goal:
                    goal = new_goal
                    lcd.show_goal(*goal)
                    print(f"{quadrant}, pos: {goal[0]} {goal[1]}") # instead of printing send to arduino for motor control
                    try:
                        i2cAr.write_i2c_block_data(ARDUINO_ADDR, 0, [each for each in goal])
                    except:
                        print("can't find arduino")
 
            status = f"{quadrant}: {goal[0]} {goal[1]}" if center is not None else f"NONE: {goal[0]} {goal[1]}"
            draw_overlay(frame, quadrant, center, corners, ids, status)
            cv2.imshow("pi only", cv2.resize(frame, PREVIEW_SIZE))
 
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        cv2.destroyAllWindows()
        lcd.close()
 
 
if __name__ == "__main__":
    main()
 
