
#include <ESP32Servo.h>
#define PINA_IN1  26 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PINA_IN2  27 // ESP32 pin GIOP26 connected to the IN2 pin L298N


#define PINB_IN1  4 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PINB_IN2  2 // ESP32 pin GIOP26 connected to the IN2 pin L298N



Servo servohorizon;
Servo servovertical;

void setup() {
  Serial.begin(115200);

  servohorizon.attach(13); // attach servo 1 to pin 13
  servovertical.attach(12); // attach servo 2 to pin 14
   // initialize digital pins as outputs.
  pinMode(PINA_IN1, OUTPUT);
  pinMode(PINA_IN2, OUTPUT);

  pinMode(PINB_IN1, OUTPUT);
  pinMode(PINB_IN2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    //Servo Control
    if (command.startsWith("S1:")) {
      int angle = command.substring(3).toInt();
      servohorizon.write(angle);
    }
    else if (command.startsWith("S2:")) {
      int angle = command.substring(3).toInt();
      servovertical.write(angle);
    }

    //Motor Control
    else if (command.startsWith("M1:")) {
      int PWM = command.substring(3).toInt();
      analogWrite(PINA_IN1, PWM); // control the motor's direction in clockwise
      analogWrite(PINA_IN2, 0);  // control the motor's direction in clockwise
      analogWrite(PINB_IN1, PWM); // control the motor's direction in clockwise
      analogWrite(PINB_IN2, 0);  // control the motor's direction in clockwise
    }
    //Turn Left
    else if (command.startsWith("M2:")) {
      int PWM = command.substring(3).toInt();
      analogWrite(PINA_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINA_IN2, PWM);  // control the motor's direction in clockwise
      analogWrite(PINB_IN1, PWM); // control the motor's direction in clockwise
      analogWrite(PINB_IN2, 0);  // control the motor's direction in clockwise
    }
    //Turn Right
    else if (command.startsWith("M3:")) {
      int PWM = command.substring(3).toInt();
      analogWrite(PINA_IN1, PWM); // control the motor's direction in clockwise
      analogWrite(PINA_IN2, 0);  // control the motor's direction in clockwise
      analogWrite(PINB_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINB_IN2, PWM);  // control the motor's direction in clockwise
    }
    //Stop
    else if (command.startsWith("M0:")) {
      int PWM = 0;
      analogWrite(PINA_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINA_IN2, 0);  // control the motor's direction in clockwise
      analogWrite(PINB_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINB_IN2, 0);  // control the motor's direction in clockwise
    }
    //Backward
    else if (command.startsWith("M4:")) {
      int PWM = command.substring(3).toInt();
      analogWrite(PINA_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINA_IN2, PWM);  // control the motor's direction in clockwise
      analogWrite(PINB_IN1, 0); // control the motor's direction in clockwise
      analogWrite(PINB_IN2, PWM);  // control the motor's direction in clockwise
    }
  }
}
 
