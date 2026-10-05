from smbus2 import SMBus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

lcd_i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, 16, 2)
lcd.color = [255, 255, 255]

with SMBus(1) as i2c:
    while True:
        try:
            value = int(input("> "))
        except ValueError:
            continue

        if 0 <= value <= 100:
            i2c.write_byte_data(8, 0, value)
            time.sleep(0.1)
            result = i2c.read_byte_data(8, 0)

            lcd.clear()
            lcd.message = str(result)
