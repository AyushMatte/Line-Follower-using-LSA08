

# Line Follower Robot using LSA08 Sensor Array with PID Control

## Introduction
This project demonstrates how to build a line follower robot using the LSA08 sensor array and an Arduino. The robot follows a black line on a white surface by detecting the line's position with the LSA08 sensor array and controlling the motors accordingly using a PID controller for smoother and more accurate line following.

## Components Required
- LSA08 sensor array
- Arduino Uno
- L298N Motor Driver
- DC Motors with wheels
- Robot chassis
- Power supply (e.g., 7.4V LiPo battery)
- Connecting wires
- Breadboard (optional)

## Circuit Diagram
1. Connect the LSA08 sensor array to the Arduino:
   - VCC to 5V
   - GND to GND
   - D0-D7 to Arduino digital pins 2-9

2. Connect the L298N motor driver to the Arduino:
   - IN1, IN2 to Arduino digital pins 10, 11 (for motor 1)
   - IN3, IN4 to Arduino digital pins 12, 13 (for motor 2)
   - ENA to Arduino PWM pin 5
   - ENB to Arduino PWM pin 6
   - VCC to battery positive
   - GND to battery negative and Arduino GND
   - Motor outputs to DC motors

## Arduino Code

```cpp
#define ENA 5
#define IN1 10
#define IN2 11
#define ENB 6
#define IN3 12
#define IN4 13

#define NUM_SENSORS 8
int sensorPins[NUM_SENSORS] = {2, 3, 4, 5, 6, 7, 8, 9};
int sensorValues[NUM_SENSORS];

int lastError = 0;
int integral = 0;

// PID constants
const float Kp = 0.6;
const float Ki = 0.3;
const float Kd = 0.1;

void setup() {
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Set initial motor speed
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
}

void loop() {
  readSensors();
  int position = getPosition();
  int error = 3500 - position;
  
  integral = integral + error;
  int derivative = error - lastError;
  
  int correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  
  int leftMotorSpeed = 150 - correction;
  int rightMotorSpeed = 150 + correction;

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

int getPosition() {
  int weightSum = 0;
  int sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightSum += sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }
  if (sum == 0) {
    return 3500; // Line is lost, return center position
  } else {
    return weightSum / sum;
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}
```

## Usage
1. Connect the components according to the circuit diagram.
2. Upload the provided code to the Arduino.
3. Place the robot on a track with a black line on a white surface.
4. Power the robot and watch it follow the line.

## Troubleshooting
- Ensure all connections are secure.
- Adjust the PID constants (Kp, Ki, Kd) in the code if the robot is not following the line smoothly.
- Check the sensor values to ensure the LSA08 is correctly detecting the line.

## License
This project is open-source and available under the MIT License.

---

This README and code incorporate a PID controller for more precise line following. You can further fine-tune the PID constants (`Kp`, `Ki`, `Kd`) to match your specific setup and track conditions.
