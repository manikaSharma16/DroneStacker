#include <Servo.h>

// This code controls three servo motors and uses two limit switches.
// It allows for serial communication to control the servos.
//
// Servo 1: lead screw
//   - Can be moved by sending "S1" or "S2" commands.
//   - "S1" moves the servo towards one limit switch (SPINCAC1_PIN).
//   - "S2" moves the servo towards the other limit switch (SPINCAC2_PIN).
//   - When a limit switch is activated (LOW), the servo stops at a predefined "home" position (1799 microseconds).
//   - Sending "STOP" will also stop Servo 1 at the home position.
//
// Servo 2 & Servo 3: brackets
//   - Controlled together to simulate opening/closing a drone mount.
//   - "O" command: Opens the mount (servos move to 160 degrees).
//   - "C" command: Closes the mount (servos move to 90 degrees).
//
// Limit Switches:
//   - SPINCAC1_PIN (Digital Pin 2) and SPINCAC2_PIN (Digital Pin 3) are used.
//   - They are active LOW, meaning they return 0 when pressed and 1 when not pressed.
//
// Serial Commands:
//   - "S1": Move Servo 1 towards limit switch 1.
//   - "S2": Move Servo 1 towards limit switch 2.
//   - "O": Open (release) the drone mount (Servo 2 & 3).
//   - "C": Close (secure) the drone mount (Servo 2 & 3).
//   - "STOP": Stop Servo 1 and return it to its home position.
//   - "END": Print the current status of both limit switches to the serial monitor.

// Define pins for servos
#define SERVO1_PIN 8
#define SERVO2_PIN 9
#define SERVO3_PIN 10

// Define pins for limit switches
#define SPINCAC1_PIN 3 // Switch for one direction of rotation (e.g., right)
#define SPINCAC2_PIN 2 // Switch for the other direction of rotation (e.g., left)

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Variables to store switch states
int spinac1Stav = HIGH;
int spinac2Stav = HIGH;

// Variable for current position of servo 1 (for smooth movement)
int servo1AktualniPozice = 1799; // Zero position in microseconds
int servo1Rychlost = 1;          // Speed of servo 1 movement (higher number = faster movement)
int movement = 1799;


void setup() {
  Serial.begin(9600);
  Serial.println("Seeeduino M0 - Servo Controller Active!");

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  // Set switch pins as inputs
  pinMode(SPINCAC1_PIN, INPUT);
  pinMode(SPINCAC2_PIN, INPUT);

  // Set initial servo positions
  servo1.write(servo1AktualniPozice); // zero position
}


void loop() {
  // Read switch states
  spinac1Stav = digitalRead(SPINCAC1_PIN);
  spinac2Stav = digitalRead(SPINCAC2_PIN);

  if(spinac1Stav == 0 && movement == 2000){
    //Serial.println(spinac1Stav);
    servo1.writeMicroseconds(1799);
    delay(20);
  }
  else if (spinac2Stav == 0 && movement == 1600){
    //Serial.println(spinac2Stav);
    servo1.writeMicroseconds(1799);
    delay(20);
  }
    
// Process commands from serial port
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("Received command: ");
    Serial.println(command);

// Command for Servo 1 (manual control possible even with switches)
    if (command.equals("S1")) {
      // Constrain angle to servo range - 544 = 0°, 2400 = 180°, 1472 = 90°
      if (spinac1Stav == 1){
        movement = 2000;
        servo1.writeMicroseconds(movement);
        Serial.println("Lidar is moving up");    
        }
      else if (spinac1Stav == 0){
        movement = 1799;
        servo1.writeMicroseconds(movement);
        Serial.println("Lidar at position up");
        } 
    }
    else if (command.equals("S2")) {
      // Constrain angle to servo range - 544 = 0°, 2400 = 180°, 1472 = 90°
      if (spinac2Stav == 1){
        movement = 1600;
        servo1.writeMicroseconds(movement);
        Serial.println("Lidar is moving down");
        }
      else if (spinac2Stav == 0){
        movement = 1799;
        servo1.writeMicroseconds(movement);
        Serial.println("Lidar at position down");
        }
      }
    // Command for Servo 2 and Servo 3 (both controlled together)
    else if (command.equals("O")) { // Command for opening, releasing the drone
      servo2.write(160);
      servo3.write(160);
      Serial.println("Drone released!");
      }
    else if (command.equals("C")) { // Command for closing, securing the drone
      servo2.write(90);
      servo3.write(90);
      Serial.println("Drone secured!");
      }
    else if (command.equals("END")){
      Serial.print("S_UP");
      Serial.println(spinac1Stav);
      Serial.print("S_DOWN");
      Serial.println(spinac2Stav);
      }
    
    else if (command.equals("STOP")){
      servo1.writeMicroseconds(1799);
    }
  }
}
