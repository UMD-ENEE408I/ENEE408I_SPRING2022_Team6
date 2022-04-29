import cv2
import numpy as np
import PIL
import http
import socketserver

PORT = 8000

match_tests = [('F',220,420,140,480), ('FL',0,420,140,480), ('FR',220,640,140,480),
            ('L',0,280,340,480), ('LF',35,400,140,480), ('LB',0,320,280,480),
            ('R',360,640,340,480), ('RF',240,615,140,480), ('RB',320,640,280,480)]

def get_junction(mouse):
    video_capture = cv2.VideoCapture(0)

    while True:
        
        ret, junction = video_capture.read()

        junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(junction_gray,200,255,cv2.THRESH_BINARY)
        
        for match_test in match_tests:
            match_type, left, right, top, bottom = match_test
            test_im = thresh[top:bottom,left:right]
        
            template = np.array(PIL.Image.open("C://Users//hnrom//ENEE408I//templates//templates\\" + match_type + ".png"))
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            h, w = template_gray.shape
        
            test_im_copy = test_im.copy()
        
            result = cv2.matchTemplate(test_im_copy, template_gray, cv2.TM_CCORR_NORMED)
            min_val, similarity, min_loc, location = cv2.minMaxLoc(result)
        
            if (similarity > 0.8):
        
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

# Server Functions
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        get_junction(self.path)
        self.send_response(200)


with socketserver.TCPServer(('', PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()
