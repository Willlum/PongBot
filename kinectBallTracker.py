import cv2
import imutils
import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
from primesense import openni2
from collections import deque
import serial
import time

class kinectBallTrackerApp(object):
    def __init__(self):
        #GUI
        self.root = tk.Tk()
        self.root.geometry('1280x720') #Set GUI screen size
        self.root.title('Kinect Ball Tracker')
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", lambda: self.closeApp())
        self.toggleButton = tk.Button(self.root, text = "Toggle Camera",font=('Helvetica',12), command = lambda: self.toggleCamera())
        self.toggleButton.pack(side='bottom', pady=5, padx=5)
        
        #vars and constants
        self.DEPTH_RES_KINECT = (512, 424)
        self.COLOR_RES_KINECT = (640, 480)
        self.maskOn = False
        self.colorLabel = None
        self.depthLabel = None
        self.xBallPrev = 1
        self.pts = deque(maxlen=16)

        #serial interface to arduino
        self.arduino = serial.Serial(port='COM8', baudrate=19200, timeout=.1)
        
        #init kinect and start color and depth streams
        openni2.initialize()     # can also accept the path of the OpenNI redistribution
        dev = openni2.Device.open_any()
        #print(dev.get_sensor_info(1))
        self.depth_stream = dev.create_depth_stream()
        self.depth_stream.start()
        self.colorStream = dev.create_color_stream()
        self.colorStream.start()
        self.eventLoop()

    def uart_write(self, x):
        xBallToStr = "<" + str(x) + ">"
        self.arduino.write(bytes(xBallToStr, 'utf-8'))
        #time.sleep(0.01)
    
    def read_depth_image(self):
        depth_frame = self.depth_stream.read_frame()
        frame_data = depth_frame.get_buffer_as_uint16()

        (img_w, img_h) = self.DEPTH_RES_KINECT
        depthImg = [[0]*img_h for i in range(img_w)]
        depthImg = np.ndarray((depth_frame.height, depth_frame.width), dtype=np.uint16,
                                buffer=frame_data).astype(np.float32)
        #Normalize image values
        (min_val, max_val, min_loc, max_loc) = cv2.minMaxLoc(depthImg)
        if (min_val < max_val):
            depthImg = (depthImg - min_val) / (max_val - min_val)
        
        return cv2.cvtColor(depthImg, cv2.COLOR_GRAY2RGB)
        #cv2.imshow("a",cv2.cvtColor(depthImg, cv2.COLOR_GRAY2RGB))

    def read_color_image(self):
        colorFrame = self.colorStream.read_frame()
        frameData = colorFrame.get_buffer_as_triplet()
        colorImg = np.ndarray((colorFrame.height, colorFrame.width, 3), dtype=np.uint8, 
                                                        buffer=frameData).astype(np.uint8)
        return colorImg

    def toggleCamera(self):
        self.maskOn = not self.maskOn

    def findBall(self, cv2Image):
        hsv = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv,(9,9),0)
        upper = np.array([103.0,154.0,255.0])
        lower = np.array([55.0,42.0,126.0])
        mask = cv2.inRange(blurred, lower, upper)
        mask = cv2.erode(mask, None, iterations= 3)
        mask = cv2.dilate(mask, None, iterations= 2)
        maskedImage = cv2.bitwise_and(blurred, blurred, mask=mask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # only proceed if tcv2Img = cv2.cvtColor(cv2Img, cv2.COLOR_RGB2he radius meets a minimum size
            if radius > 8:
                M = cv2.moments(c)
                if(M["m00"] > 0):   
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # draw the circle and centroid on the maskedImage,
                # then update the list of tracked points
                cv2.circle(maskedImage, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(maskedImage, center, 5, (0, 0, 255), -1)
                # update the points queue
                self.pts.appendleft(center)
                for i in range(1, len(self.pts)):
                    # if either of the tracked points are None, ignore
                    # them
                    if self.pts[i - 1] is None or self.pts[i] is None:
                        continue
                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(32 / float(i + 1)) * 2)
                    cv2.line(maskedImage, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)
        
        return maskedImage, center
            
    def closeApp(self):
            self.arduino.reset_output_buffer()
            self.uart_write("-1") #home motor
            time.sleep(1)
            self.arduino.close()
            self.root.quit()
            self.depth_stream.stop()
            self.colorStream.stop()
            openni2.unload()              
            cv2.destroyAllWindows()

    def eventLoop(self):
        cv2img = self.read_color_image()
        cv2color = cv2img
        cv2img, ballLoc = self.findBall(cv2img)
        cv2img = cv2.cvtColor(cv2img, cv2.COLOR_HSV2BGR)
        # Convert the Image object into a TkPhoto object
        maskedImage = ImageTk.PhotoImage(Image.fromarray(cv2img)) 
        colorImage = ImageTk.PhotoImage(Image.fromarray(cv2color))
        
        if self.colorLabel is None:
            self.colorLabel = tk.Label(self.root, image=colorImage)
            self.colorLabel.image = colorImage
            self.colorLabel.pack(side='left',padx=10, pady=10)          
        else:
            if self.maskOn == True:
                self.colorLabel.configure(image=maskedImage)
                self.colorLabel.image = maskedImage        
            else:
                self.colorLabel.configure(image=colorImage)
                self.colorLabel.image = colorImage
        
        cv2depth = self.read_depth_image() * 255
        if ballLoc is not None:
            (xBall, yBall) = ballLoc
            if xBall < (self.xBallPrev-20) or xBall > (self.xBallPrev+20):
                pError = np.abs((xBall-self.xBallPrev) / self.xBallPrev)
                #print(pError)
                if(pError < 0.4):
                    self.uart_write(xBall)
                    cv2.circle(cv2depth,[xBall,yBall],5,(0,0,255),-1)
                self.xBallPrev = xBall
        
        depthImage = ImageTk.PhotoImage(Image.fromarray(cv2depth.astype(np.uint8))) 
        if self.depthLabel is None:
            self.depthLabel = tk.Label(self.root, image=depthImage)
            self.depthLabel.image = depthImage
            self.depthLabel.pack(side='left', padx=10,pady=10)

        else:    
            self.depthLabel.configure(image=depthImage)
            self.depthLabel.image = depthImage
        #30 fps for kinect -> 32ms per event
        self.root.after(32, self.eventLoop)

if __name__ == "__main__":
    app = kinectBallTrackerApp()
    app.root.mainloop()