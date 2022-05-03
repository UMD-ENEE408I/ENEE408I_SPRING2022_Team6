import cv2
import numpy as np
from matplotlib import pyplot as plt
import PIL
import urllib.request


# match_tests = [('F',304,424,155,480), ('LB',0,320,330,480), ('L',0,320,330,480), ('LF',89,364,155,480),
#             ('FL',144,419,105,480), ('FR',311,586,105,480), ('RF',364,639,155,480), ('R',320,640,330,480),
#             ('RB',320,640,330,480), ('END',0,640,120,480)]

match_tests = [("forward",220,420,60,480), ("forward to left",0,420,140,480), ("forward to right",220,640,140,480),
            ("left",0,280,240,480), ("left to forward",35,400,140,480), ("left to backward",0,320,280,480),
            ("right",360,640,240,480), ("right to forward",240,615,140,480), ("right to backward",320,640,280,480),
            ("end",0,640,120,480)]
titles = []
images = []

video_capture = cv2.VideoCapture(0)

while True:
    
    ret, junction = video_capture.read()

    junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(junction_gray,210,255,cv2.THRESH_BINARY)
    
    for match_test in match_tests:
        match_type, left, right, top, bottom = match_test
        test_im = thresh[top:bottom,left:right]
    
        template = np.array(PIL.Image.open("C://Users//hnrom//ENEE408I//ENEE408I_SPRING2022_Team6-demo_test_sockets//Junction Detection Server//templates\\" + match_type + ".png"))
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        h, w = template_gray.shape
    
        test_im_copy = test_im.copy()
    
        result = cv2.matchTemplate(test_im_copy, template_gray, cv2.TM_CCORR_NORMED)
        min_val, similarity, min_loc, location = cv2.minMaxLoc(result)
    
        if (similarity > 0.78):
    
            bottom_right = (location[0] + w, location[1] + h)
            cv2.rectangle(test_im_copy, location, bottom_right, 125, 2)
                
            print(f"{match_type} ({similarity:.0%})")
        
    # Display the resulting image
    cv2.imshow('Video', thresh)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
       
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
