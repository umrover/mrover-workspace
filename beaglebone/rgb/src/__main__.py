#main function for rgb sensor and i2c multiplexer
from i2c_multiplexer import tca_select
import rgb_sensor
import time

def main():
    #Write to all channels to enable
    tca_select(0xff)
    rgb_sensor.enable()
    
    while(True):
        #Read data from channel 1
        tca_select(0x02)
        r, g, b = rgb_sensor.getData()
        print("----------------")
        print("Red 1: ", r)
        print("Green 1: ", g)
        print("Blue 1: ", b)
        
        time.sleep(2)
        
        #Read data from channel 2
        tca_select(0x04)
        r, g, b = rgb_sensor.getData()
        print("----------------")
        print("Red 2: ", r)
        print("Green 2: ", g)
        print("Blue 2: ", b)
        
        #Flash lights
        rgb_sensor.light(False)
        time.sleep(.5)
        rgb_sensor.light(True)
        
        tca_select(0x02)
        rgb_sensor.light(False)
        time.sleep(.5)
        rgb_sensor.light(True)
        
        time.sleep(2)

if __name__ == "__main__":
    main()

