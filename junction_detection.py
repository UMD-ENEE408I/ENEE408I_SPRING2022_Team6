import cv2
import numpy as np
from matplotlib import pyplot as plt

import PIL
import urllib.request

# junction_tests = ['L - FL - R','R - FR','F - LF - RF','L - R','LB - RB','LF - RF']
match_tests = [('LB',80,140,50,140), ('L',80,140,50,140), ('LF',80,140,50,140),
           ('FL',20,140,80,170), ('F',20,130,80,170), ('FR',20,140,80,170),
           ('RF',80,140,110,205), ('R',80,140,110,205), ('RB',80,140,110,205)]

titles = []
images = []

video_capture = cv2.VideoCapture(0)

while True:
    
    ret, junction = video_capture.read()
    #junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)

    thresh = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
    #ret, thresh = cv2.threshold(junction_gray,127,255,cv2.THRESH_BINARY)

    junction_input = thresh[10:10, 10:10]
# for junction_type in junction_tests:
#    junction = np.array(PIL.Image.open("C://Users//hnrom//ENEE408I//sample_junctions//sample junctions//" + junction_type + ".png"))
#    junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
#    ret,thresh = cv2.threshold(junction_gray,127,255,cv2.THRESH_BINARY)

    for junction_type in junction_input:
    
        for match_test in match_tests:
            match_type, top, bottom, left, right = match_test
            test_im = thresh[top:bottom,left:right]
        
            template = np.array(PIL.Image.open("C:\\Users\\hnrom\\ENEE408I\\templates\\templates\\" + match_type + ".png"))
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            h, w = template_gray.shape
        
            test_im_copy = test_im.copy()
        
            result = cv2.matchTemplate(test_im_copy, template_gray, cv2.TM_CCORR_NORMED)
            min_val, similarity, min_loc, location = cv2.minMaxLoc(result)
        
            if (similarity > 0.8):
        
                bottom_right = (location[0] + w, location[1] + h)
                cv2.rectangle(test_im_copy, location, bottom_right, 125, 2)
        
                titles.append(junction_type)
                titles.append(f"{match_type} ({similarity:.0%})")
                images.append(junction)
                images.append(test_im_copy)
                
                print("its a match")
        
    # Display the resulting image
    cv2.imshow('Video', thresh)


    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
       
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

# for i in range(len(images)):
#    plt.subplot(4,8,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)
#    plt.title(titles[i])
#    plt.xticks([]),plt.yticks([])

# plt.tight_layout()
# plt.show()
