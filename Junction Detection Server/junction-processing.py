import face_recognition
import cv2
import numpy as np
import PIL
import asyncio
import websockets
import json
import threading


params = {
    "paths": {
        "forward": False,
        "left": False,
        "right": False,
        "forward to left": False,
        "forward to right": False,
        "left to forward": False,
        "right to forward": False,
        "left to backward": False,
        "right to backward": False
    },
    "is_end": False,
    "vip": ""
}

match_tests = [("forward",220,420,60,480), ("forward to left",0,420,140,480), ("forward to right",220,640,140,480),
            ("left",0,280,240,480), ("left to forward",35,400,140,480), ("left to backward",0,320,280,480),
            ("right",360,640,240,480), ("right to forward",240,615,140,480), ("right to backward",320,640,280,480),
            ("end",0,640,120,480)]

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

video_capture = cv2.VideoCapture(0)


def get_junction():

    paths = {}
    is_end = False
    
    count = 0
    num_samples = 5
    
    while(count < num_samples):
            
        ret, junction = video_capture.read()
        
        junction_gray = cv2.cvtColor(junction, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(junction_gray,215,255,cv2.THRESH_BINARY)
        
        for match_test in match_tests:
            match_type, left, right, top, bottom = match_test
            test_im = thresh[top:bottom,left:right]
        
            template = np.array(PIL.Image.open("C://Users//hnrom//ENEE408I//ENEE408I_SPRING2022_Team6-demo_test_sockets//Junction Detection Server//templates\\" + match_type + ".png"))
            template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        
            test_im_copy = test_im.copy()
        
            result = cv2.matchTemplate(test_im_copy, template_gray, cv2.TM_CCORR_NORMED)
            min_val, similarity, min_loc, location = cv2.minMaxLoc(result)
        
            if (similarity > 0.9) and match_type == "end":
                is_end = True
            elif (similarity > 0.78):
                paths[match_type] = True
            
        count = count + 1

    params["paths"] = paths
    params["is_end"] = is_end

    print("Paths: ", params["paths"])
    print()
    print("END?: ", params["is_end"])
    print()




def get_vip():
    
    name = ""

    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    frame2 = frame[:, :, ::-1]
    frame3 = cv2.resize(frame2, (900,900), fx=5, fy=5)

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

    # return name
    params["vip"] = name

    print("VIP: ", params["vip"])
    print()


def run_vip_processor():
    while(True):
        print("Started VIP Processor")
        print()
        get_vip()

def run_junction_processor():
    while(True):
        print("Started Junction Processor")
        print()
        get_junction()

vip_thread = threading.Thread(target=run_vip_processor)
junction_thread = threading.Thread(target=run_junction_processor)

vip_thread.setDaemon(True)
junction_thread.setDaemon(True)

vip_thread.start()
junction_thread.start()

async def on_message(websocket, path):
    mouse_name = await websocket.recv()
    print()
    print(f"[Sending {mouse_name} junction]")
    print()
    await websocket.send(json.dumps(params))

start_server = websockets.serve(on_message, "localhost", 9000)

print("Started Socket Server on port 9000")

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()