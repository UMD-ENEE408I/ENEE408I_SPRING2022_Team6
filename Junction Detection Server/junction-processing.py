import face_recognition
import cv2
import numpy as np
import PIL
import http
import socketserver
import json

PORT = 9000

match_tests = [('forward',220,420,140,480), ('forward to left',0,420,140,480), ('forward to right',220,640,140,480),
            ('left',0,280,340,480), ('left to forward',35,400,140,480), ('left to backward',0,320,280,480),
            ('right',360,640,340,480), ('right to forward',240,615,140,480), ('right to backward',320,640,280,480),
            ('end',0,640,120,480)]

# Load a sample picture and learn how to recognize it.
chan_image = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_1.jpg")
chan_face_encoding = face_recognition.face_encodings(chan_image)[0]

# Load a second sample picture and learn how to recognize it.
reeves_image = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_1.jpg")
reeves_face_encoding = face_recognition.face_encodings(reeves_image)[0]

# Load a second sample picture and learn how to recognize it.
rock_image = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_1.jpg")
rock_face_encoding = face_recognition.face_encodings(rock_image)[0]


# Create arrays of known face encodings and their names
known_face_encodings = [
    chan_face_encoding,
    reeves_face_encoding,
    rock_face_encoding,
]
known_face_names = [
    "Jackie Chan",
    "Keanu Reeves",
    "Dwayne Johnson",
]

def get_junction(mouse):
    video_capture = cv2.VideoCapture(0)

    paths = {}
    is_end = False

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

            if (similarity > 0.8) and match_type == 'end':
                is_end = True
                print(f"{match_type} ({similarity:.0%})")
            if (similarity > 0.8):
                paths[match_test] = True
                print(f"{match_type} ({similarity:.0%})")
            
        # Display the resulting image
        cv2.imshow('Video', thresh)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()

    return {
        'paths': paths,
        'is_end': is_end
    }




def get_vip(mouse):

    # Get a reference to webcam #0 (the default one)
    video_capture = cv2.VideoCapture(0)

    name = ''

    while True:
        # Grab a single frame of video
        ret, frame = video_capture.read()

        #small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
        
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        frame2 = frame[:, :, ::-1]
        frame3 = cv2.resize(frame2, (900,900), fx=5, fy=5)
        # frame3 = cv2.cvtColor(frame4, cv2.COLOR_BGR2GRAY)

        # Find all the faces and face enqcodings in the frame of video
        face_locations = face_recognition.face_locations(frame3)
        face_encodings = face_recognition.face_encodings(frame3, face_locations)

        # Loop through each face in this frame of video
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

            # If a match was found in known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
                            
            # Draw a box around the face
            cv2.rectangle(frame3, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame3, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame3, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video', frame3)


        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
    
    return name



# Server Functions
class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        PARAMS = get_junction(self.path)
        PARAMS['vip'] = get_vip(self.path)
        self.send_response(200)
        self.wfile.write(json.dumps(PARAMS))


with socketserver.TCPServer(('', PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()
