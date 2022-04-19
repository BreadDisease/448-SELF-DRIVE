import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import cv2
from scipy import signal
from scipy.signal import find_peaks
from sklearn import linear_model
from sklearn.linear_model import LinearRegression, RANSACRegressor
from object_detection import ObjectDetector


class RobotVision:

    def __init__(self, detection=False, debugMode=False):
        super().__init__()
        self.debugMode = debugMode
        self.detection = detection
        if detection:
            self.detector = ObjectDetector(gpu=False)

    def detect(self, frame):
        class_ids, scores, boxes = self.detector.detect(frame)
        image = self.detector.drawbox(frame, class_ids, scores, boxes)
        
        return image

    def perceive(self, frame, scale_percent = 20):
        rgb_image = self.getResizedRGBImage(frame, scale_percent)
        
        scale_rate = int(scale_percent / 20.0)
        startPoint = scale_rate * 20
        endPoint = startPoint + 166 * scale_rate
        
        rgb_image = rgb_image[startPoint:endPoint, :]

        temp = self.preprocessImage(rgb_image)

        ### Detect data points
        Y_coor, lefts, rights = self.detectDataPoint(temp)
        if (len(Y_coor) < 3):
            return 0, 0, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        else:
            try:
                ### Fit Ransac
                ransacLeft, ransacRight =  self.fitRANSAC(Y_coor, lefts, rights)

                ### Output Frame
                isValid, intercept, outframe, theme =  self.outputFrame(rgb_image, ransacLeft, ransacRight)

                if isValid == False:
                    return 0, 0, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                if self.detection == True:
                    outframe2 = self.detect(outframe)
                    rtimg = self.plotFrame(intercept, outframe2, theme)
                else:
                    rtimg = self.plotFrame(intercept, outframe, theme)
                
                ### Answer
                suggested_adjustment = self.response(intercept, temp.shape)

                return 1, suggested_adjustment, cv2.cvtColor(rtimg, cv2.COLOR_RGB2BGR)
            except:
                return 0, 0, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

    def resizeImage(self, img, scale_percent):
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)

        # resize image
        resized_image = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

        return resized_image


    def getResizedRGBImage(self, img, scale_percent):
        # resize image
        resized_image = self.resizeImage(img, scale_percent)

        # Change COLOR mode
        rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
        
        return rgb_image

    def preprocessImage(self, image, low_threshold = 100, high_threshold = 250):
        kernel = np.ones((3,3),np.float32)/9
        dst = cv2.filter2D(image,-1,kernel)
        
        img_dilation = cv2.dilate(dst, kernel, iterations=3)
        img_erosion2 = cv2.erode(img_dilation, kernel, iterations=3)
        
        edges = cv2.Canny(img_erosion2, low_threshold, high_threshold)
        
        sobelx = cv2.Sobel(src=edges, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
        sobely = cv2.Sobel(src=edges, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
        
        temp = cv2.bitwise_and(sobelx, sobely)
        
        return temp

    def detectDataPoint(self, img):
        Y_coor = []
        lefts = []
        rights = []

        y = img.shape[0] - 1
        while y >= 20:
            if (len(lefts) >= 20):
                break
            horizontal_pixels = img[y,:]
            if (horizontal_pixels.min() >=0):
                y -= 1
                continue
            else:
                leftPos = np.argmin(horizontal_pixels)
                if (horizontal_pixels[leftPos:].max() <= 0):
                    y -= 1
                    continue
                else:
                    rightPos = leftPos + np.argmax(horizontal_pixels[leftPos:])
                    if rightPos - leftPos > 100:
                        # Add pairs
                        Y_coor.append(y)
                        lefts.append(leftPos)
                        rights.append(rightPos)
            ### update 
            y -= 3
        
        return Y_coor, lefts, rights

    def fitRANSAC(self, Y_coor, lefts, rights): 
        # Y data
        Y_data = np.array(Y_coor).reshape(-1, 1)

        # Left line
        X_left = np.array(lefts).reshape(-1, 1)

        # Right line
        X_right = np.array(rights).reshape(-1, 1)

        # Robustly fit linear model with RANSAC algorithm
        ransacLeft = linear_model.RANSACRegressor(base_estimator=LinearRegression())
        ransacLeft.fit(Y_data, X_left)
        ransacRight = linear_model.RANSACRegressor(base_estimator=LinearRegression())
        ransacRight.fit(Y_data, X_right)

        return ransacLeft, ransacRight

    def outputFrame(self, image, ransacLeft, ransacRight):
        # Calculate interception
        a1 = ransacLeft.estimator_.coef_
        b1 = ransacLeft.estimator_.intercept_

        a2 = ransacRight.estimator_.coef_
        b2 = ransacRight.estimator_.intercept_

        if a1 == a2:
            return False, None, None, None

        Y_intercept = (b2-b1) / (a1 - a2)
        X_intercept = a1 * Y_intercept + b1 

        copy3 = image.copy()
        theme = 255 * np.ones_like(image)

        for i in range(copy3.shape[0]):
            if i < int(Y_intercept):
                continue
            leftDot  = int(ransacLeft.predict(np.array([[i]])))
            rightDot = int(ransacRight.predict(np.array([[i]])))
            if  leftDot <  copy3.shape[1] and leftDot >= 0:
                copy3[i, leftDot] = [255, 0, 0]
                theme[i, leftDot] = [0, 0, 0]
            if  rightDot <  copy3.shape[1] and rightDot >= 0:
                copy3[i, rightDot] = [255, 0, 0]
                theme[i, rightDot] = [0, 0, 0]
            # if np.abs(rightDot - leftDot) < 1:
            #     print(f"Intercept at [{leftDot}/{rightDot}, {i}]")
        
        return True, [X_intercept, Y_intercept], copy3, theme

    def plotFrame(self, intercept, outFrame, theme):
        try:
            return cv2.circle(outFrame, (int(intercept[0]), int(intercept[1])), radius=4, color=(0, 0, 255), thickness=20)
        except:
            return outFrame
            
    def response(self, intercept, imageSize):
        standardVector = [0, imageSize[0]]
        origin_point = [imageSize[1] / 2, 0]
        
        intercept_point = [intercept[0][0][0], intercept[1][0][0]]
        z_vector = [intercept_point[0] - origin_point[0], intercept_point[1] - origin_point[1]]
        
        ab = (z_vector[1]*standardVector[1])
        absZ = np.sqrt(z_vector[0]**2 + z_vector[1]**2)
        absVector = np.sqrt(standardVector[0]**2 + standardVector[1]**2)

        cosineAngle = ab / (absZ * absVector)

        angle = np.arccos(cosineAngle) * (180 / np.pi)

        # Do some condition here
        # We will not trust any suggested angle which is more than 45
        # Angle magnitude [0, 45] 
        if angle > 45:
            angle = 45

        if self.debugMode:
            print("-------------------------")
            print(imageSize)
            print(standardVector)
            print(origin_point)
            print(intercept_point)
            if (z_vector[0] < 0):
                print(-angle)
            else:
                print(+angle)
            print("-------------------------")

        if (z_vector[0] < 0):
            return -angle 
        else:
            return angle 
