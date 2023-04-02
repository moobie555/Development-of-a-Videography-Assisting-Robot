import BlynkLib
import cv2
import mediapipe as mp
import serial
import time

auth_token = 'CdSk3z3RlP9IiQhRKYbnG__reL4aC29f'
blynk = BlynkLib.Blynk(auth_token)

#define video stream resolution
cap_width = 320
cap_height = 320

v1_value = None
v2_value = None
v3_value = None
v4_value = None   
v5_value = None
v6_value = None
v7_value = 5   #default time for dolly time


# Set up the serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Define Calibration index for calibrate distance
CalibrationIndex = 0.5


# Define the servo motor control function
def move_servo(servo_num, angle):
    command = f"S{servo_num}:{angle}\n"
    ser.write(command.encode())

def move_motor(motor_num ,pwm):
    command = f"M{motor_num}:{pwm}\n"
    ser.write(command.encode())
    
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
  
servo1_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=90, min_output=0, max_output=90)
servo2_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=90, min_output=0, max_output=90)
dc_motor_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=128, min_output= 0, max_output=255)

def on_v0_write(values):
    global pwm
    pwm = int(values[0])
    print(pwm)
    
def on_v1_write(values):
    global v1_value
    global v2_value
    v1_value = values[0]
    if v1_value == '0':
        print('Auto mode')
        
    elif v1_value == '1':
        print('Manual mode')
        move_motor(0 ,pwm)      #motorStop
        while v1_value == '1':
            blynk.run()
            dolly_time = int(v7_value)
            if v3_value is not None and int(v3_value) >= 192:
                if v2_value is not None:
                    v2_int = int(v2_value)
                    if v2_int <= 64:
                        print('Turn Forward Left')
                        move_motor(2 ,pwm)      #motorTurnLeft
                    elif v2_int >= 192:
                        print('Turn Forward Right')
                        move_motor(3 ,pwm)      #motorTurnRight
                    elif v2_int > 64 and v2_int < 192:
                        print('Forward')
                        move_motor(1 ,pwm)      #motorForward
                else:
                    print("v2_value is None")
            elif v3_value is not None and int(v3_value) <= 64:
                if v2_value is not None:
                    v2_int = int(v2_value)
                    if v2_int <= 64:
                        print('Turn Backward Left')
                        move_motor(5 ,pwm)      #motorTurnLeft
                    elif v2_int >= 192:
                        print('Turn Backward Right')
                        move_motor(6 ,pwm)      #motorTurnRight
                    elif v2_int > 64 and v2_int < 192:
                        print('Backward')
                        move_motor(4 ,pwm)      #motorForward
                else:
                    print("v2_value is None")
                    
            elif v5_value is not None and int(v5_value) == 1:
                while v5_value == '1':
                    print('Fade In!')
                    move_motor(1 ,pwm)      #motorForward
                    time.sleep(dolly_time)
                    print('Fade In complete!')
                    blynk.run()
                
            elif v6_value is not None and int(v6_value) == 1:
                while v6_value == '1':  
                    print('Fade Out!')
                    move_motor(4 ,pwm)      #motorBackward
                    time.sleep(dolly_time)
                    print('Fade Out complete!')
                    blynk.run()
        
            else:
                print('Motor stop')
                move_motor(0 ,pwm)      #motorStop
                
def on_v2_write(values):
    global v2_value
    v2_value = values[0]

def on_v3_write(values):
    global v3_value
    v3_value = values[0]

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
    
def on_v5_write(values):
    global v5_value
    v5_value = values[0]

def on_v6_write(values):
    global v6_value
    v6_value = values[0]
    
def on_v7_write(values):
    global v7_value 
    v7_value = values[0]
    

blynk.on("V0", on_v0_write)
blynk.on("V1", on_v1_write)
blynk.on("V2", on_v2_write)
blynk.on("V3", on_v3_write)
blynk.on("V4", on_v4_write)
blynk.on("V5", on_v5_write)
blynk.on("V6", on_v6_write)
blynk.on("V7", on_v7_write)

while True:
    
    pwm = 64 #default pwm
    video_style_range = 100 #default Video Style to Medium mode
    
    # Initialize pose detection
    print('Initialize pose detection')
    
    with mp_pose.Pose(model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

        
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)


        prev_degreeHorizon = 0
        prev_degreeVertical = 0
        prev_distance = 0

        while cap.isOpened():
            blynk.run()
            
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Add a small delay (e.g., 30 ms) to limit the frame rate
            time.sleep(0.03)

            if not ret:
                print("Ignoring empty camera frame.")
                continue

            # Resize the input frame
            frame = cv2.resize(frame, (480, 480))

            # Pass by reference.
            frame.flags.writeable = False
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process frame
            results = pose.process(frame)

            # Draw the pose annotation on the image.
            frame.flags.writeable = True

            # Draw info on frame
            if results.pose_landmarks:
                degreeHorizon = ((results.pose_landmarks.landmark[0].x) * 90)
                degreeVertical = ((results.pose_landmarks.landmark[0].y) * 90)
                distance = (-1 / (results.pose_landmarks.landmark[0].z)) * 100 * CalibrationIndex
                
                
                # Update PID controllers
                servo1_output = servo1_pid.update(degreeHorizon)
                servo2_output = servo2_pid.update(degreeVertical)
                dc_motor_output = dc_motor_pid.update(distance)

                # Send the output to the servos and DC motor
                move_servo(1, servo1_output)
                move_servo(2, servo2_output)
                
                if distance > video_style_range:
                    if degreeHorizon >= 35 and degreeHorizon <= 55:
                        move_motor(1 ,pwm)
                        print('forward')
                    elif degreeHorizon > 55:
                        move_motor(3 ,pwm)      #motorTurnRight
                        print('Right')
                    elif degreeHorizon < 35:
                        move_motor(2 ,pwm)      #motorTurnLeft
                        print('Left')
                elif distance < video_style_range:
                    move_motor(0 ,pwm)

            else:
                # If no pose landmarks are detected, stop the DC motor
                move_motor(0, 0)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            
            #cv2.imshow('MediaPipe Pose', cv2.flip(frame, 1))
            
            
    # Release the camera and close the OpenCV window when done
    cap.release()
    cv2.destroyAllWindows()