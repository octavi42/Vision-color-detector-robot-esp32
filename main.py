from machine import Pin, I2C
from WonderCam import *
import time


i2c = I2C(0, scl=Pin(16), sda=Pin(17), freq=400000)
cam = WonderCam(i2c)

cam.set_func(WONDERCAM_FUNC_COLOR_DETECT) 

#These are the colors corresponding to the color IDs
color_dict = {
    1: "Red",
    2: "Green",
    3: "Blue",
    4: "Yellow",
    5: "Black",
    6: "Purple"
}

if __name__ == '__main__':
    while True:
        cam.update_result()  
        color_data = None

        # Iterate through the color IDs and check if the color is detected
        for color_id, color_name in color_dict.items():

            # Returnes true if the color id matches the detected color
            color_data = cam.get_color_blob(color_id)
            if color_data:
                print(color_name) 
                break
        else:
            print("No color detected")

        if color_data:
            center_x = color_data[0]
            center_y = color_data[1]
            print(f"Position - x: {center_x}, y: {center_y}")

        time.sleep(1)