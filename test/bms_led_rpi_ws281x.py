import time
from rpi_ws281x import ws, PixelStrip, Adafruit_NeoPixel, Color


class BMSLedStrip(): 
    def __init__(self):
        ### SETUP OF LED STRIP
        LED_2_COUNT = 54        # Number of LED pixels.
        #LED_2_PIN = 13          # GPIO pin connected to the pixels (must support PWM! GPIO 13 or 18 on RPi 3).
        LED_2_PIN = 10          # GPIO 10 uses SPI this can run without sudo 
        LED_2_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
        LED_2_DMA = 10          # DMA channel to use for generating signal (Between 1 and 14)
        LED_2_BRIGHTNESS = 30  # Set to 0 for darkest and 255 for brightest
        LED_2_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
        LED_2_CHANNEL = 1       # 0 or 1
        LED_2_STRIP = ws.WS2811_STRIP_GBR

        self.strip = PixelStrip(LED_2_COUNT, LED_2_PIN, LED_2_FREQ_HZ,
                                LED_2_DMA, LED_2_INVERT, LED_2_BRIGHTNESS,
                                LED_2_CHANNEL, LED_2_STRIP)
        self.strip.begin()
        self.update(9)
        #self.led_test()
        time.sleep(2)
        self.colorWipe()
        self.config()

    def config(self):
        pass


    def led_test(self):
        self.strip.setPixelColor(0, Color(255,0,0))
        time.sleep(1 / 1000.0)
        self.strip.show()
        self.strip.setPixelColor(1, Color(255,0,0))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(2, Color(255,0,0))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(3, Color(255,0,0))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(4, Color(255,0,0))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(5, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(6, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(7, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(8, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(9, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(10, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.setPixelColor(11, Color(0,0,255))
        time.sleep(1 / 1000.0)
        self.strip.show()

    def colorWipe(self, color= Color(0, 0, 0), wait_ms=5):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def update(self, strip_part_On):
        try:
            for i in range(self.strip.numPixels()):
                if i <= strip_part_On:
                    shade_g = int(255*strip_part_On/9)
                    shade_r = int(-255*strip_part_On/9 + 255)
                    self.strip.setPixelColor(i, Color(shade_g,shade_r,0))
                    #self.strip.setPixelColor(i, Color(255,0,0))
                    time.sleep(1 / 1000.0)
                else:
                    self.strip.setPixelColor(i, Color(0,0,0))
                    time.sleep(1 / 1000.0)
            self.strip.show()
        except KeyboardInterrupt:
            self.colorWipe()

if __name__ == '__main__':
    strip = BMSLedStrip()
    BMS_Percentage = 0
    while True: 
        try:
            BMS_Percentage += 10 
            BMS_Percentage = BMS_Percentage % 100
            strip.update( int( BMS_Percentage / 100 * 9) ) 
            time.sleep(2)
        except KeyboardInterrupt:
            strip.colorWipe()

