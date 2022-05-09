import face_recognition
import cv2
import numpy as np

        
# def face_rec():
# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)

  
# Load JACKIE CHAN images to train
chan_image_01 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_1.jpg")
chan_face_encoding_01 = face_recognition.face_encodings(chan_image_01)[0]
chan_image_02 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_2.jpg")
chan_face_encoding_02 = face_recognition.face_encodings(chan_image_02)[0]
chan_image_03 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_3.jpg")
chan_face_encoding_03 = face_recognition.face_encodings(chan_image_03)[0]
chan_image_04 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_4.jpg")
chan_face_encoding_04 = face_recognition.face_encodings(chan_image_04)[0]
chan_image_05 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_5.jpg")
chan_face_encoding_05 = face_recognition.face_encodings(chan_image_05)[0]
chan_image_06 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_6.jpg")
chan_face_encoding_06 = face_recognition.face_encodings(chan_image_06)[0]
chan_image_07 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_7.jpg")
chan_face_encoding_07 = face_recognition.face_encodings(chan_image_07)[0]
chan_image_08 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_8.jpg")
chan_face_encoding_08 = face_recognition.face_encodings(chan_image_08)[0]
chan_image_09 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_9.jpg")
chan_face_encoding_09 = face_recognition.face_encodings(chan_image_09)[0]
chan_image_10 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//jackie_chan//jackie_10.jpg")
chan_face_encoding_10 = face_recognition.face_encodings(chan_image_10)[0]


# Load KEANU REEVES images to train
reeves_image_01 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_1.jpg")
reeves_face_encoding_01 = face_recognition.face_encodings(reeves_image_01)[0]
reeves_image_02 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_2.jpg")
reeves_face_encoding_02 = face_recognition.face_encodings(reeves_image_02)[0]
reeves_image_03 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_3.jpg")
reeves_face_encoding_03 = face_recognition.face_encodings(reeves_image_03)[0]
reeves_image_04 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_4.jpg")
reeves_face_encoding_04 = face_recognition.face_encodings(reeves_image_04)[0]
reeves_image_05 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_5.jpg")
reeves_face_encoding_05 = face_recognition.face_encodings(reeves_image_05)[0]
reeves_image_06 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_6.jpg")
reeves_face_encoding_06 = face_recognition.face_encodings(reeves_image_06)[0]
reeves_image_07 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_7.jpg")
reeves_face_encoding_07 = face_recognition.face_encodings(reeves_image_07)[0]
reeves_image_08 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_8.jpg")
reeves_face_encoding_08 = face_recognition.face_encodings(reeves_image_08)[0]
reeves_image_09 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_9.jpg")
reeves_face_encoding_09 = face_recognition.face_encodings(reeves_image_09)[0]
reeves_image_10 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//keanu_reeves//keanu_10.jpg")
reeves_face_encoding_10 = face_recognition.face_encodings(reeves_image_10)[0]


# Load DWAYNE JOHNSON images for training
rock_image_01 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_1.jpg")
rock_face_encoding_01 = face_recognition.face_encodings(rock_image_01)[0]
rock_image_02 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_2.jpg")
rock_face_encoding_02 = face_recognition.face_encodings(rock_image_02)[0]
rock_image_03 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_3.jpg")
rock_face_encoding_03 = face_recognition.face_encodings(rock_image_03)[0]
rock_image_04 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_4.jpg")
rock_face_encoding_04 = face_recognition.face_encodings(rock_image_04)[0]
rock_image_05 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_5.jpg")
rock_face_encoding_05 = face_recognition.face_encodings(rock_image_05)[0]
rock_image_06 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_6.jpg")
rock_face_encoding_06 = face_recognition.face_encodings(rock_image_06)[0]
rock_image_07 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_7.jpg")
rock_face_encoding_07 = face_recognition.face_encodings(rock_image_07)[0]
rock_image_08 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_8.jpg")
rock_face_encoding_08 = face_recognition.face_encodings(rock_image_08)[0]
rock_image_09 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_9.jpg")
rock_face_encoding_09 = face_recognition.face_encodings(rock_image_09)[0]
rock_image_10 = face_recognition.load_image_file("C://Users//hnrom//ENEE408I//dataset//the_rock//rock_10.jpg")
rock_face_encoding_10 = face_recognition.face_encodings(rock_image_10)[0]


# Create arrays of known face encodings and their names
known_face_encodings = [
    chan_face_encoding_01,chan_face_encoding_02,chan_face_encoding_03,chan_face_encoding_04,chan_face_encoding_05,
    chan_face_encoding_06,chan_face_encoding_07,chan_face_encoding_08,chan_face_encoding_09,chan_face_encoding_10,
    
    reeves_face_encoding_01,reeves_face_encoding_02,reeves_face_encoding_03,reeves_face_encoding_04,reeves_face_encoding_05,
    reeves_face_encoding_06,reeves_face_encoding_07,reeves_face_encoding_08,reeves_face_encoding_09,reeves_face_encoding_10,

    rock_face_encoding_01,rock_face_encoding_02,rock_face_encoding_03,rock_face_encoding_04,rock_face_encoding_05,
    rock_face_encoding_06,rock_face_encoding_07,rock_face_encoding_08,rock_face_encoding_09,rock_face_encoding_10,

]
known_face_names = [
    "Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan","Jackie Chan",
    "Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves","Keanu Reeves",
    "Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson","Dwayne Johnson",
]

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

        name = "Unknown"

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

    # return name
        
    
    
    
    
    
    
    
    
    
