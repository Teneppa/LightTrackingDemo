import cv2 as cv2
import math
import time


"""
 > Coded by Teemu w/ bugs
"""

# >> CAMERA PARAMETERS ARE HERE <<
focal_length = 1430                 # Usually in the datasheet
real_size_in_mm = 4.5               # This is the real size of the light
camera_fov = 60                     # This is in the camera's datasheet too

video_capture = cv2.VideoCapture(0)

# > Set the exposure to -3 (a really dark image)
# > -4 results in a better tracking distance, but also
#   increases the likelyhood of misdetections.
#
# > NOTE: This might not work with your camera!
video_capture.set(cv2.CAP_PROP_EXPOSURE, -10)

maxLatency = 0

while(1):

    gotImage,img = video_capture.read()
    starttime = time.time()

    if gotImage:

        # Flip image
        # NOTE: Sometimes Windows flips the image automatically,
        #       **** Windows
        img = cv2.flip(img, 1)

        # Convert to grayscale and do a treshold
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,240,255,cv2.THRESH_BINARY)

        # Erode and dilate to get rid of small dots
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)

        # Find contours (bright spots)
        contours,hierarchy = cv2.findContours(thresh, 1, 2)

        # Find the average for the contours
        cntAverageX = 0
        cntAverageY = 0
        cntAverageW = 0
        cntAverageH = 0
        cntCount = 0

        # Go thorugh all of the contours
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            # Draw all of the contours to the img
            cv2.rectangle(img,(x,y),(x+w, y+h),(0,255,0),2)

            cntAverageX += x
            cntAverageY += y
            cntAverageW += w
            cntAverageH += h
            cntCount += 1

        # If the image contained contours, create the average
        if cntCount != 0:
            cAvgX = int(cntAverageX / cntCount)
            cAvgY = int(cntAverageY / cntCount)
            cAvgW = int(cntAverageW / cntCount)
            cAvgH = int(cntAverageH / cntCount)

            capWidth = video_capture.get(3)
            capHeight = video_capture.get(4)
            
            #===================================================
            # Calculate the distance to the camera
            #
            # The formula is yeeted from this blogpost:
            # http://emaraic.com/blog/distance-measurement
            #===================================================
            size_in_pixels = (cAvgW+cAvgH)/2
            
            # Calculate the distance here
            dist_from_cam = focal_length*(real_size_in_mm/size_in_pixels)

            #===================================================
            # Convert the x- and y-position in pixels to real
            # world coordinates
            #===================================================
            xPos = round(cAvgX/capWidth, 2)
            yPos = round(cAvgY/capHeight, 2)

            # Get the angle and convert it to radians
            xAngle = math.radians((xPos-0.5)*camera_fov)
            yAngle = math.radians((yPos-0.5)*camera_fov)

            # Calculate the x- and y-coordinates
            realXPos = math.tan(xAngle)*dist_from_cam
            realYPos = -1*math.tan(yAngle)*dist_from_cam

            #print(round(realXPos, 2), round(realYPos, 2), round(dist_from_cam, 2), "| UNIT = mm")

            cv2.rectangle(img,(cAvgX, cAvgY),(cAvgX+cAvgW, cAvgY+cAvgH),(255,0,0),2)

        cv2.putText(img,
                    f'{(time.time() - starttime)*1000} ms',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255,255,255),
                    2
                    )

        if (time.time() - starttime)*1000 > maxLatency:
            maxLatency = (time.time() - starttime)*1000
        
        cv2.putText(img,
                    f'MAX: {maxLatency} ms',
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255,255,255),
                    2
                    )
        
        # Show the images
        cv2.imshow('tresh', thresh)
        cv2.imshow('image', img)

        # Exit if q is pressed
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('r'):
            maxLatency = 0
    else:
        print("No image")

cv2.destroyAllWindows()
video_capture.release()
