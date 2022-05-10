import face_recognition
import cv2
import numpy as np
import PIL
import asyncio
import websockets
import json
import time


match_tests = [("F", "forward",200,440,80,320), ("F-1", "forward",200,440,80,320), ("F-2", "forward",200,440,80,320), ("F-3", "forward",200,440,80,320), ("F-4", "forward",200,440,80,320), 
                ("F-5", "forward",200,440,80,320), ("F-6", "forward",200,440,80,320), ("F-7", "forward",200,440,80,320), ("F-8", "forward",200,440,80,320), ("F-9", "forward",200,440,80,320), 
                ("FL", "forward to left",60,540,30,310), ("FL-1", "forward to left",60,540,30,310), ("FL-2", "forward to left",60,540,30,310), ("FL-3", "forward to left",60,540,30,310), ("FL-4", "forward to left",60,540,30,310), ("FL-5", "forward to left",60,540,30,310), 
                ("FR", "forward to right",100,580,30,310), ("FR-1", "forward to right",100,580,30,310), ("FR-2", "forward to right",100,580,30,310), ("FR-3", "forward to right",100,580,30,310), ("FR-4", "forward to right",100,580,30,310), ("FR-5", "forward to right",100,580,30,310), 
                ("L", "left",10,340,140,380), ("L-1", "left",10,340,140,380), ("L-2", "left",10,340,140,380), ("L-3", "left",10,340,140,380), ("L-4", "left",10,340,140,380), ("L-5", "left",10,340,140,380), 
                ("LF", "left to forward",0,360,80,400), ("LF-1", "left to forward",0,360,80,400), ("LF-2", "left to forward",0,360,80,400), ("LF-3", "left to forward",0,360,80,400), ("LF-4", "left to forward",0,360,80,400), ("LF-5", "left to forward",0,360,80,400), 
                ("LB", "left to backward",0,380,120,480), ("LB-1", "left to backward",0,380,120,480), ("LB-2", "left to backward",0,380,120,480), ("LB-3", "left to backward",0,380,120,480), ("LB-4", "left to backward",0,380,120,480), ("LB-5", "left to backward",0,380,120,480), 
                ("R", "right",300,630,140,380), ("R-1", "right",300,630,140,380), ("R-2", "right",300,630,140,380), ("R-3", "right",300,630,140,380), ("R-4", "right",300,630,140,380), ("R-5", "right",300,630,140,380), 
                ("RF", "right to forward",280,640,80,400), ("RF-1", "right to forward",280,640,80,400), ("RF-2", "right to forward",280,640,80,400), ("RF-3", "right to forward",280,640,80,400), ("RF-4", "right to forward",280,640,80,400), ("RF-5", "right to forward",280,640,80,400), 
                ("RB", "right to backward",260,640,120,480), ("RB-1", "right to backward",260,640,120,480), ("RB-2", "right to backward",260,640,120,480), ("RB-3", "right to backward",260,640,120,480), ("RB-4", "right to backward",260,640,120,480), ("RB-5", "right to backward",260,640,120,480), 
                ("END", "end",0,640,120,480), ("END-1", "end",0,640,120,480), ("END-2", "end",0,640,120,480), ("END-3", "end",0,640,120,480)]


# Facial Detection #
cascPath = "haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
result = False


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
    "jackie",
    "keanu",
    "dwayne",
]

video_capture = cv2.VideoCapture(0)


def get_junction():

    paths = {}
    is_end = False
    
    count = 0
    num_samples = 10
    
    while(count < num_samples):
            
        ret, junction = video_capture.read()
        
        junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(junction_gray,180,255,cv2.THRESH_BINARY)
        
        for match_test in match_tests:
            filename, match_type, left, right, top, bottom = match_test
            test_im = thresh[top:bottom,left:right]
        
            template = np.array(PIL.Image.open("C://Users//hnrom//ENEE408I//ENEE408I_SPRING2022_Team6-demo_test_sockets//Junction Detection Server//templates\\" + filename + ".png"))
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        
            test_im_copy = test_im.copy()
        
            result = cv2.matchTemplate(test_im_copy, template_gray, cv2.TM_CCORR_NORMED)
            min_val, similarity, min_loc, location = cv2.minMaxLoc(result)
        
            if (similarity > 0.8) and match_type == "end":
                is_end = True
            elif (similarity > 0.78):
                paths[match_type] = True
            
        count = count + 1

    print("Paths: ", paths)
    print("END?: ", is_end)

    return {
        "paths": paths,
        "is_end": is_end
    }




def get_vip():
    
    name = ""

    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    frame2 = frame[:, :, ::-1]
    frame3 = cv2.resize(frame2, (900,900), fx=5, fy=5)
    frame3 = frame3[200:900,0:900]

    # Find all the faces and face enqcodings in the frame of video
    face_locations = face_recognition.face_locations(frame3)
    face_encodings = face_recognition.face_encodings(frame3, face_locations)

    # Loop through each face in this frame of video
    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

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
        

    print("VIP: ", name)
    print()
    
    return name

def detect_face():
    
    count = 0
    num_samples = 10
    face_sum = 0
    
    while(count < num_samples):
    
        ret, frame = video_capture.read()
        frame = frame[50:900,0:900]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        faces = []
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        if (len(faces) > 0):
            face_sum = face_sum + 1
            print('Found face D:')
        
        count = count + 1
            
        
    return face_sum > 5


async def on_message(websocket, path):
    print()
    print('[Reading Junction]')
    print()
    params = get_junction()
    is_face = detect_face()
    print('Face?',is_face)
    if is_face:
        params["vip"] = get_vip()
        while params["vip"] == "":
            params["vip"] = get_vip()
    else:
        params["vip"] = ""
        
    await websocket.send(json.dumps(params))

start_server = websockets.serve(on_message, "localhost", 9000)

print("Started Socket Server on port 9000")

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()