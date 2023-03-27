import cv2
import mediapipe as mp
import serial
import time

# Set PWM
pwm = 128

# Set up the serial connection
ser = serial.Serial('COM4', 115200)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Define Calibration index
CalibrationIndex = 0.5

# Initialize FPS calculation
fps_start_time = 0
fps_end_time = 0
fps_history = []

# Define the servo motor control function
def move_servo(servo_num, angle):
    command = f"S{servo_num}:{angle}\n"
    ser.write(command.encode())

def move_motor(motor_num ,pwm):
    command = f"M{motor_num}:{pwm}\n"
    ser.write(command.encode())

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


# Initialize pose detection
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:

    # Create video capture
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev_degreeHorizon = 0
    prev_degreeVertical = 0
    prev_distance = 0

    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Ignoring empty camera frame.")
            continue

        # Resize the input frame
        frame = cv2.resize(frame, (240, 240))

        # Calculate FPS
        fps_end_time = time.time()
        fps = 1 / (fps_end_time - fps_start_time)
        fps_history.append(fps)
        fps_start_time = fps_end_time

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

            if distance > 50:
                if degreeHorizon >= 35 and degreeHorizon <= 55:
                    move_motor(1 ,pwm)
                    print('forward')
                elif degreeHorizon > 55:
                    move_motor(2 ,pwm)      #motorTurnRight
                    print('Right')
                elif degreeHorizon < 35:
                    move_motor(3 ,pwm)      #motorTurnLeft
                    print('Left')
            elif distance < 50:
                move_motor(0 ,pwm)

        else:
            # If no pose landmarks are detected, stop the DC motor
            move_motor(0, 0)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        print('X : ' ,degreeHorizon,' = ' ,servo1_output,'\nY :',
            degreeVertical,' = ' , servo2_output  ,'\nZ : ',
            distance,'=',dc_motor_output,'\n')
        cv2.imshow('MediaPipe Pose', cv2.flip(frame, 1))
