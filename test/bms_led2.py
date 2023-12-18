import time
import neopixel
import board


def update(strip, strip_part_On):
    try:
        for i in range(12):
            if i <= strip_part_On:
                shade_g = int(255*strip_part_On/12)
                shade_r = int(-255*strip_part_On/12 + 255)
                strip[-i-1] = (0,0.8*shade_r+20,shade_g*0.8+20) #BRG
                #self.strip.setPixelColor(i, Color(255,0,0))
            else:
                strip[-i-1] = (0,0,0)
    except KeyboardInterrupt:
        strip.fill((0,0,0))
        quit()

if __name__ == '__main__':
    strip = neopixel.NeoPixel(board.D21, 12)
    BMS_Percentage = 0
    while True: 
        try: 
            if (BMS_Percentage > 100): BMS_Percentage = 0
            update(strip, int( BMS_Percentage / 100 * 12))
            print(BMS_Percentage / 100 * 12,  int( BMS_Percentage / 100 * 12) )
            BMS_Percentage += 5
            time.sleep(0.5)
        except KeyboardInterrupt:
            strip.fill((0,0,0))
            break

