# ---------------------------------------------------------------------------
# Copyright (c) Development-of-a-Videography-Assisting-Robot Team
# Electrical Engineering of Mahidol University
# All rights reserved.
# ---------------------------------------------------------------------------

import BlynkLib
import cv2
import mediapipe as mp
import serial
import time
import threading
import matplotlib.pyplot as plt
import numpy as np

auth_token = 'CdSk3z3RlP9IiQhRKYbnG__reL4aC29f'
blynk = BlynkLib.Blynk(auth_token)

#define video stream resolution
cap_width = 480
cap_height = 480

v1_value = None
v2_value = None
v3_value = None
v4_value = None   
v5_value = None
v6_value = None
v7_value = 5   #default time for dolly time
v8_value = None

distance = 0 #Initialize distance variable


# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh()

pwm = 64 #default pwm

video_style_range = 50 #default Video Style to Medium mode

# Define Calibration index for calibrate distance
CalibrationIndex = 0.5

# Define the servo motor control function
def move_servo(servo_num, angle):
    command = f"S{servo_num}:{angle}\n"
    ser.write(command.encode())

def move_motor_A(motor_num ,pwm):
    command = f"MA{motor_num}:{pwm}\n"
    ser.write(command.encode())

def move_motor_B(motor_num ,pwm):
    command = f"MB{motor_num}:{pwm}\n"
    ser.write(command.encode())

def motor_forward():
    print('forward')
    move_motor_A(1 ,pwm)
    move_motor_A(2 ,0)
    move_motor_B(1 ,pwm)
    move_motor_B(2 ,0)

def motor_forward_left():
    print('forward left')
    move_motor_A(1 ,0)
    move_motor_A(2 ,pwm)
    move_motor_B(1 ,pwm)
    move_motor_B(2 ,0)

def motor_forward_right():
    print('forward right')
    move_motor_A(1 ,pwm)
    move_motor_A(2 ,0)
    move_motor_B(1 ,0)
    move_motor_B(2 ,pwm)

def motor_stop():
    print('stop')
    move_motor_A(1 ,0)
    move_motor_A(2 ,0)
    move_motor_B(1 ,0)
    move_motor_B(2 ,0)

def motor_backward():
    print('backward')
    move_motor_A(1 ,0)
    move_motor_A(2 ,pwm)
    move_motor_B(1 ,0)
    move_motor_B(2 ,pwm)

def motor_backward_left():
    print('backward left')
    move_motor_A(1 ,pwm)
    move_motor_A(2 ,0)
    move_motor_B(1 ,0)
    move_motor_B(2 ,pwm)

def motor_backward_right():
    print('backward right')
    move_motor_A(1 ,0)
    move_motor_A(2 ,pwm)
    move_motor_B(1 ,pwm)
    move_motor_B(2 ,0)

def read_count_from_serial():
    while ser.in_waiting > 0:
        line = ser.readline().decode().strip()
        if line.startswith("COUNT:"):
            RPM = int(line.split(":")[1])
            return RPM
    return None

#define PID control
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.min_output = min_output
        self.max_output = max_output
        self.previous_error = 0
        self.integral = 0
        self.time_previous = time.time()

    def update(self, current_value):
        time_now = time.time()
        delta_time = time_now - self.time_previous

        error = self.setpoint - current_value
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.max_output), self.min_output)

        self.previous_error = error
        self.time_previous = time_now

        return output
  
servo1_pid = PID(Kp=0.5, Ki=0.0, Kd=0.0, setpoint=90, min_output=0, max_output=90)
servo2_pid = PID(Kp=0.5, Ki=0.0, Kd=0.0, setpoint=90, min_output=0, max_output=90)
dc_motor_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=64, min_output= 0, max_output=255)

def facetracking():

    # Initialize variables for FPS calculation
    fps_hist = []
    prev_time = 0
    fps = 0

    # Initialize lists to store data for plotting graphs
    x_pan = []
    y_pan = []
    x_tilt = []
    y_tilt = []
    
    # Initialize pose detection
    print('Initialize pose detection')
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)
    
    while cap.isOpened():
        
        global degreeHorizon
        global degreeVertical
        global distance

        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Ignoring empty camera frame.")
            return None  # Add this line

        # Resize the input frame
        frame = cv2.resize(frame, (480, 480))

        # Pass by reference.
        frame.flags.writeable = False
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process frame
        results = face_mesh.process(frame)

        # Draw the pose annotation on the image.
        frame.flags.writeable = True

        # Draw info on frame
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                nose_landmark = face_landmarks.landmark[4]

                degreeHorizon = ((nose_landmark.x) * 90)
                degreeVertical = ((nose_landmark.y) * 90)
                distance = (-1 / (nose_landmark.z)) * 100 * CalibrationIndex

                # Convert the nose landmark from normalized coordinates to pixel coordinates
                x = int(nose_landmark.x * cap_width)
                y = int(nose_landmark.y * cap_height)

                # Draw a circle at the nose landmark
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                
                x_centroid = x-240
                y_centroid = y-240

                # Add data to lists for plotting graphs
                x_pan.append(time.time())
                y_pan.append(x_centroid)
                x_tilt.append(time.time())
                y_tilt.append(y_centroid)


            # Update PID controllers
            servo1_output = servo1_pid.update(degreeHorizon)
            servo2_output = servo2_pid.update(degreeVertical)
            #dc_motor_output = dc_motor_pid.update(RPM)

            # Send the output to the servos and DC motor
            move_servo(1, servo1_output)
            move_servo(2, servo2_output)
        
        curr_time = time.time()
        elapsed_time = curr_time - prev_time
        prev_time = curr_time
        fps = int(1 / elapsed_time)
        fps_hist.append(fps)

        # Display the FPS on the frame
        cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow('Monitoring', frame)

        # Wait for a keypress and break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Calculate the average error from 0 for Pan and Tilt
            pan_avg_err = (np.mean(np.abs(y_pan))/240)*100
            tilt_avg_err = (np.mean(np.abs(y_tilt))/240)*100
            fps_avg = (np.mean(np.abs(fps_hist)))
            # Plot the Pan graph
            plt.figure()
            plt.plot(x_pan, y_pan)
            plt.title('Pan')
            plt.xlabel('Time')
            plt.ylabel('Pan error')
            plt.ylim(-240, 240)
            plt.axhline(y=0, color='r', linestyle='-')
            plt.axhline(y=-48, color='orange', linestyle='--')
            plt.axhline(y=48, color='orange', linestyle='--')
            plt.text(0.60, 0.05, f'%average error: {pan_avg_err:.2f}' ' %', transform=plt.gca().transAxes)

            # Plot the Tilt graph
            plt.figure()
            plt.plot(x_tilt, y_tilt)
            plt.title('Tilt')
            plt.xlabel('Time')
            plt.ylabel('Tilt error')
            plt.ylim(-240, 240)
            plt.axhline(y=0, color='r', linestyle='-')
            plt.axhline(y=-48, color='orange', linestyle='--')
            plt.axhline(y=48, color='orange', linestyle='--')
            plt.text(0.60, 0.05, f'%average error: {tilt_avg_err:.2f}' ' %', transform=plt.gca().transAxes)

            #Plot FPS
            plt.figure()
            plt.plot(np.arange(len(fps_hist)), fps_hist)
            plt.title('fps over Time')
            plt.xlabel('frame')
            plt.ylabel('fps')
            plt.text(0.70, 0.05, f'average: {fps_avg:.0f}' ' fps', transform=plt.gca().transAxes)
            plt.axhline(y = fps_avg, color='red', linestyle='--')

            # Show the plots
            plt.show()
            break

    # Release the video capture and close all windows
    cap.release()
    cv2.destroyAllWindows()

                        
#recieve Max Speed
def on_v0_write(values):
    global pwm
    pwm = int(values[0])
    print(pwm)

#recieve Video Mode
def on_v1_write(values):
    global v1_value
    global v2_value
    v1_value = values[0]
    
#recieve X
def on_v2_write(values):
    global v2_value
    v2_value = values[0]
    
#recieve Y
def on_v3_write(values):
    global v3_value
    v3_value = values[0]
    
#recieve Video Style
def on_v4_write(values):
    global video_style_range
    v4_value = values[0]
    if v4_value is not None and int(v4_value) == 1:
        print('You are in Close-up shot mode')
        video_style_range = 50
    elif v4_value is not None and int(v4_value) == 2:
        print('You are in Medium shot mode')
        video_style_range = 100
    elif v4_value is not None and int(v4_value) == 3:
        print('You are in Long shot mode')
        video_style_range = 200

#recieve Dolly In
def on_v5_write(values):
    global v5_value
    v5_value = values[0]

#recieve Dolly Out
def on_v6_write(values):
    global v6_value
    v6_value = values[0]
    
#recieve Dolll time    
def on_v7_write(values):
    global v7_value 
    v7_value = values[0]

#recieve Emergeny Stop
def on_v8_write(values):
    global v8_value 
    v8_value = values[0]
    if v8_value == '1':
        print('Emergency Stop')
        while v8_value == '1':
            motor_stop()   #Motor Stop
            blynk.run()

blynk.on("V0", on_v0_write)
blynk.on("V1", on_v1_write)
blynk.on("V2", on_v2_write)
blynk.on("V3", on_v3_write)
blynk.on("V4", on_v4_write)
blynk.on("V5", on_v5_write)
blynk.on("V6", on_v6_write)
blynk.on("V7", on_v7_write)
blynk.on("V8", on_v8_write)

facetracking_thread = threading.Thread(target=facetracking,args=())
facetracking_thread.start()

while True:
    
    blynk.run()
    
    # RPM = read_count_from_serial()
    # if RPM is not None:
    #     print(f"RPM: {RPM}")
    # frame = facetracking
    
    #Check Auto Mode?
    if v1_value == '0':
        print('Auto mode')
        

        if distance > video_style_range:
            move_motor_A(1 ,(degreeHorizon/90)*pwm)
            move_motor_A(2 ,0)
            move_motor_B(1 ,((90-degreeHorizon)/90)*pwm)
            move_motor_B(2 ,0)
        elif distance < video_style_range:
            motor_stop()
    
    #Check Manual Mode?
    elif v1_value == '1':
        print('Manual mode')
        if v1_value == '1':
            blynk.run()
            dolly_time = int(v7_value)
            if v3_value is not None and int(v3_value) >= 192:
                if v2_value is not None:
                    v2_int = int(v2_value)
                    if v2_int <= 64:
                        motor_forward_left()      #motorTurnLeft
                    elif v2_int >= 192:
                        motor_forward_right()     #motorTurnRight
                    elif v2_int > 64 and v2_int < 192:
                        motor_forward()      #motorForward
                else:
                    print("v2_value is None")
            elif v3_value is not None and int(v3_value) <= 64:
                if v2_value is not None:
                    v2_int = int(v2_value)
                    if v2_int <= 64:
                        motor_backward_left()     #motorTurnLeft
                    elif v2_int >= 192:
                        motor_backward_right()      #motorTurnRight
                    elif v2_int > 64 and v2_int < 192:
                        motor_backward()      #motorBackward
                else:
                    print("v2_value is None")
                    
            elif v5_value is not None and int(v5_value) == 1:
                while v5_value == '1':
                    print('Fade In!')
                    motor_forward()      #motorForward
                    time.sleep(dolly_time)
                    print('Fade In complete!')
                    blynk.run()
                
            elif v6_value is not None and int(v6_value) == 1:
                while v6_value == '1':  
                    print('Fade Out!')
                    motor_backward()      #motorBackward
                    time.sleep(dolly_time)
                    print('Fade Out complete!')
                    blynk.run()
     
            else:
                motor_stop()      #motorStop
