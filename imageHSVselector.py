import cv2
import imutils
import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
from primesense import openni2

class hsvColorSelectorApp(object):
    def __init__(self):
        #vars and constants
        self.DEPTH_RES_KINECT = (512, 424)
        self.COLOR_RES_KINECT = (640, 480)
        self.colorLabel = None

        #init kinect and start color
        openni2.initialize()     # can also accept the path of the OpenNI redistribution
        dev = openni2.Device.open_any()
        #print(dev.get_sensor_info(1))
        self.colorStream = dev.create_color_stream()
        self.colorStream.start()

        #GUI
        self.root = tk.Tk()
        self.root.title('Kinect Ball Tracker')
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", lambda: self.closeApp())

        self.v1 = tk.DoubleVar()
        self.v2 = tk.DoubleVar()
        self.v3 = tk.DoubleVar()
        self.v4 = tk.DoubleVar()
        self.v5 = tk.DoubleVar()
        self.v6 = tk.DoubleVar()

        self.vLowSlider = tk.Scale(self.root, from_ = 1, to=255, orient='horizontal', length=300,label='V Lower',
            variable=self.v5)
        self.vLowSlider.pack(side='bottom', pady=5, padx=5)
        self.vHighSlider = tk.Scale(self.root, from_ = 1, to=255, orient='horizontal', length=300,label='V Upper',
            variable=self.v6)
        self.vHighSlider.pack(side='bottom', pady=5, padx=5)
        
        self.sLowSlider = tk.Scale(self.root, from_ = 1, to=255, orient='horizontal', length=300,label='S Lower',
           variable=self.v3)
        self.sLowSlider.pack(side='bottom', pady=5, padx=5)
        self.sHighSlider = tk.Scale(self.root, from_ = 1, to=255, orient='horizontal', length=300,label='S Upper',
            variable=self.v4)
        self.sHighSlider.pack(side='bottom', pady=5, padx=5)

        self.hLowSlider = tk.Scale(self.root, from_ = 1, to=180, orient='horizontal', length=300,label='H Lower', 
            variable=self.v1)
        self.hLowSlider.pack(side='bottom', pady=5, padx=5)
        self.hHighSlider = tk.Scale(self.root, from_ = 1, to=180, orient='horizontal', length=300,label='H Upper',
            variable=self.v2)
        self.hHighSlider.pack(side='bottom', pady=5, padx=5)

        self.updateHsvValue()

    def read_color_image(self):
        colorFrame = self.colorStream.read_frame()
        frameData = colorFrame.get_buffer_as_triplet()
        colorImg = np.ndarray((colorFrame.height, colorFrame.width, 3), dtype=np.uint8,
                                                        buffer=frameData).astype(np.uint8)
        return colorImg

    def updateHsvValue(self):
        upper = np.array([self.v2.get(),self.v4.get(),self.v6.get()])
        lower = np.array([self.v1.get(),self.v3.get(),self.v5.get()])
  
        cv2Img = self.read_color_image()
        # Convert the Image object into a TkPhoto object
        hsv = cv2.cvtColor(cv2Img, cv2.COLOR_BGR2HSV)

        blurred = cv2.GaussianBlur(hsv,(9,9), 0)
        mask = cv2.inRange(blurred, lower, upper)
        mask = cv2.erode(mask, None, iterations= 3)
        mask = cv2.dilate(mask, None, iterations= 2)
        maskImage = cv2.bitwise_and(blurred, blurred, mask=mask)

        maskedImage = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(maskImage, cv2.COLOR_HSV2BGR)))

        if self.colorLabel is None:
            # Put it in the display window
            self.colorLabel = tk.Label(self.root, image=maskedImage)
            self.colorLabel.image = maskedImage
            self.colorLabel.pack(side='left',padx=10, pady=10)
        else:
            self.colorLabel.configure(image=maskedImage)
            self.colorLabel.image = maskedImage
        self.root.after(200, self.updateHsvValue)


    def closeApp(self):
        print("upper = np.array([" +str(self.v2.get())+","+str(self.v4.get())+","+str(self.v6.get())+"])")
        print("lower = np.array([" + str(self.v1.get())+","+str(self.v3.get())+","+str(self.v5.get())+"])")
        self.root.quit()
        self.colorStream.stop()
        openni2.unload()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    #create gui to update photo
    #create event
    app = hsvColorSelectorApp()
    app.root.mainloop()