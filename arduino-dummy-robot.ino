#include "PS2X_lib.h"
#include <Servo.h>

// Comment out the bellow line to disable debug logging
#define DEBUG

#define CONTROLLER_DATA_PIN       51  // MOSI
#define CONTROLLER_COMMAND_PIN   50  // MISO
#define CONTROLLER_ATTENTION_PIN  42  // SELECT
#define CONTROLLER_CLOCK_PIN      52  // CLOCK

#define FRONT_LEFT_MOTOR_PWM 2
#define FRONT_RIGHT_MOTOR_PWM 3
#define REAR_LEFT_MOTOR_PWM 4
#define REAR_RIGHT_MOTOR_PWM 5


#define FRONT_LEFT_MOTOR_DIR 32
#define FRONT_RIGHT_MOTOR_DIR 30
#define REAR_LEFT_MOTOR_DIR 28
#define REAR_RIGHT_MOTOR_DIR 26

#define ELEVATE_MOTOR_PWM 6
#define ELEVATE_MOTOR_DIR 24

#define LEFT_FINGER 8
#define RIGHT_FINGER 9

#define LEFT_FAN 10
#define RIGHT_FAN 11

#define RED_LED 40
#define GREEN_LED 38
#define BLUE_LED 36

#define BUZZER 34

#define NO_PRESSURE   false
#define NO_RUMMBLE    false

#define UNKNOW_CONTROLLER      0
#define DUALSHOCK_CONTROLLER   1
#define GUITAR_HERO_CONTROLLER 2
#define WIRELESS_DUALSHOCK_CONTROLLER 3

#define NO_ERROR 0
#define CONTROLLER_NOT_FOUND 1
#define CONTROLLER_NOT_ACCEPTING_COMMANDS 2
#define CONTROLLER_REFUSING_TO_ENTER_PRESSURE_MODE 3

PS2X controller;

Servo leftFinger;
Servo rightFinger;

Servo leftFan;
Servo rightFan;

byte controllerError = 0;
byte controllerType = 0;

int leftJoystickX = 0;
int leftJoystickY = 0;
int rightJoystickX = 0;
int rightJoystickY = 0;
int leftJoystickAngle = 0;
int leftJoystickMagnitude = 0;
int rightJoystickAngle = 0;
int rightJoystickMagnitude = 0;

void setup() {
  setUpPinout();
  Serial.begin(115200);
  configController();
  delay(100);
}

void loop() {
  if ((controllerError == CONTROLLER_NOT_FOUND) || (controllerType == GUITAR_HERO_CONTROLLER))
    return;
  else {
    controller.read_gamepad();
    
    // Serial.print("SELECT:");   Serial.print(controller.Button(PSB_SELECT));   Serial.print(", ");
    // Serial.print("START:");    Serial.print(controller.Button(PSB_START));    Serial.print(", ");
    // Serial.print("UP:");       Serial.print(controller.Button(PSB_PAD_UP));       Serial.print(", ");
    // Serial.print("DOWN:");     Serial.print(controller.Button(PSB_PAD_DOWN));     Serial.print(", ");
    // Serial.print("LEFT:");     Serial.print(controller.Button(PSB_PAD_LEFT));     Serial.print(", ");
    // Serial.print("RIGHT:");    Serial.print(controller.Button(PSB_PAD_RIGHT));    Serial.print(", ");
    // Serial.print("TRIANGLE:"); Serial.print(controller.Button(PSB_TRIANGLE)); Serial.print(", ");
    // Serial.print("CIRCLE:");   Serial.print(controller.Button(PSB_CIRCLE));   Serial.print(", ");
    // Serial.print("CROSS:");    Serial.print(controller.Button(PSB_CROSS));    Serial.print(", ");
    // Serial.print("SQUARE:");   Serial.print(controller.Button(PSB_SQUARE));   Serial.print(", ");
    // Serial.print("L1:");       Serial.print(controller.Button(PSB_L1));       Serial.print(", ");
    // Serial.print("L2:");       Serial.print(controller.Button(PSB_L2));       Serial.print(", ");
    // Serial.print("L3:");       Serial.print(controller.Button(PSB_L3));       Serial.print(", ");
    // Serial.print("R1:");       Serial.print(controller.Button(PSB_R1));       Serial.print(", ");
    // Serial.print("R2:");       Serial.print(controller.Button(PSB_R2));       Serial.print(", ");
    // Serial.print("R3:");       Serial.print(controller.Button(PSB_R3));       Serial.print(", ");
    leftJoystickX = controller.Analog(PSS_LX);
    leftJoystickY = controller.Analog(PSS_LY);
    rightJoystickX = controller.Analog(PSS_RX);
    rightJoystickY = controller.Analog(PSS_RY);
    Serial.print("LX:"); Serial.print(leftJoystickX); Serial.print(", ");
    Serial.print("LY:"); Serial.print(leftJoystickY); Serial.print(", ");
    Serial.print("RX:"); Serial.print(rightJoystickX); Serial.print(", ");
    Serial.print("RY:"); Serial.print(rightJoystickY); Serial.print(" --- ");

    if (((leftJoystickX > 112) && (leftJoystickX < 144) && (leftJoystickY > 112) && (leftJoystickY < 144)) && ((rightJoystickX > 112) && (rightJoystickX < 144))) {
      Serial.print(", ");
      Serial.print("Center/No movement --- ");
      stand();
    }

    else if ((rightJoystickX < 112)) {
      Serial.print((127 - rightJoystickX)*2); Serial.print(" --- ");
      rotateLeft((127 - rightJoystickX)*2);
    }

    else if (rightJoystickX > 144) {
      Serial.print((rightJoystickX - 127)*2); Serial.print(" --- ");
      rotateRight((rightJoystickX - 128)*2);
    }

    else {
      leftJoystickAngle = atan2(-(leftJoystickY - 128), (leftJoystickX - 128)) * 180 / PI;
      leftJoystickMagnitude = constrain(sqrt((leftJoystickX - 128) * (leftJoystickX - 128) + (leftJoystickY - 128) * (leftJoystickY - 128)), 0, 128) * 2 - 1;

      Serial.print(", Left angle: "); Serial.print(leftJoystickAngle);
      Serial.print(", Left magnitude: "); Serial.print(leftJoystickMagnitude); Serial.print(" --- ");

      if ((leftJoystickAngle >= 68) && (leftJoystickAngle < 112)) {
        moveForward(leftJoystickMagnitude);
      }
      else if ((leftJoystickAngle >= 112) && (leftJoystickAngle < 158)) {
        moveLeftForward(leftJoystickMagnitude);
      }
      else if (((leftJoystickAngle >= 158) && (leftJoystickAngle <= 180)) || ((leftJoystickAngle >= -179) && (leftJoystickAngle < -158))) {
        moveLeft(leftJoystickMagnitude);
      }
      else if ((leftJoystickAngle >= -158) && (leftJoystickAngle < -112)) {
        moveLeftBackward(leftJoystickMagnitude);
      }
      else if ((leftJoystickAngle >= -112) && (leftJoystickAngle < -68)) {
        moveBackward(leftJoystickMagnitude);
      }
      else if ((leftJoystickAngle >= -68) && (leftJoystickAngle < -22)) {
        moveRightBackward(leftJoystickMagnitude);
      }
      else if (((leftJoystickAngle >= -22) && (leftJoystickAngle <= 0)) || ((leftJoystickAngle >= 1) && (leftJoystickAngle < 22))) {
        moveRight(leftJoystickMagnitude);
      }
      else if ((leftJoystickAngle >= 22) && (leftJoystickAngle < 68)) {
        moveRightForward(leftJoystickMagnitude);
      }
    }
  }
  delay(50);
}

void moveForward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Forward");
  #endif
}

void moveBackward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, HIGH);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Backward");
  #endif
}

void moveLeft(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Left");
  #endif
}

void moveRight(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, HIGH);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Right");
  #endif
}

void moveLeftForward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, 0);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, 0); 

  #ifdef DEBUG
  Serial.println("Move Left Forward");
  #endif
}

void moveRightForward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, 0);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, 0);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Right Forward");
  #endif
}

void moveLeftBackward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, 0);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, 0);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Move Left Backward");
  #endif
}

void moveRightBackward(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, 0);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, HIGH);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, 0); 

  #ifdef DEBUG
  Serial.println("Move Right Backward");
  #endif
}

void rotateLeft(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, LOW);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, HIGH);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, LOW);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Rotate Left");
  #endif
}

void rotateRight(uint8_t speed) {
  digitalWrite(FRONT_LEFT_MOTOR_DIR, LOW);
  analogWrite(FRONT_LEFT_MOTOR_PWM, speed);
  digitalWrite(FRONT_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, speed);
  digitalWrite(REAR_LEFT_MOTOR_DIR, LOW);
  analogWrite(REAR_LEFT_MOTOR_PWM, speed);
  digitalWrite(REAR_RIGHT_MOTOR_DIR, HIGH);
  analogWrite(REAR_RIGHT_MOTOR_PWM, speed); 

  #ifdef DEBUG
  Serial.println("Rotate Right");
  #endif
}

void up() {
  digitalWrite(ELEVATE_MOTOR_DIR, LOW);
  analogWrite(ELEVATE_MOTOR_PWM, 255);

  #ifdef DEBUG
  Serial.println("Up");
  #endif
}

void down() {
  digitalWrite(ELEVATE_MOTOR_DIR, HIGH);
  analogWrite(ELEVATE_MOTOR_PWM, 255);
  
  #ifdef DEBUG
  Serial.println("Down");
  #endif
}

void grab() {
  leftFinger.write(0);
  rightFinger.write(180);

  #ifdef DEBUG
  Serial.println("Grab");
  #endif
}

void release() {
  leftFinger.write(30);
  rightFinger.write(120);

  #ifdef DEBUG
  Serial.println("Grab");
  #endif
}

void openHand() {
  leftFinger.write(90);
  rightFinger.write(90);

  #ifdef DEBUG
  Serial.println("Open Hand");
  #endif
}

void fanOn() {
  leftFan.write(90);
  rightFan.write(90);

  #ifdef DEBUG
  Serial.println("Fan On");
  #endif
}

void fanOff() {
    
  #ifdef DEBUG
  Serial.println("Fan Off");
  #endif
}

void stand() {
  analogWrite(FRONT_LEFT_MOTOR_PWM, 0);
  analogWrite(FRONT_RIGHT_MOTOR_PWM, 0);
  analogWrite(REAR_LEFT_MOTOR_PWM, 0);
  analogWrite(REAR_RIGHT_MOTOR_PWM, 0);

  // digitalWrite(FRONT_LEFT_MOTOR_DIR, 0);
  // digitalWrite(FRONT_RIGHT_MOTOR_DIR, 0);
  // digitalWrite(REAR_LEFT_MOTOR_DIR, 0);
  // digitalWrite(REAR_RIGHT_MOTOR_DIR, 0);

  digitalWrite(ELEVATE_MOTOR_DIR, LOW);
  analogWrite(ELEVATE_MOTOR_PWM, 0);

  #ifdef DEBUG
  Serial.println("Stand");
  #endif
}

void beep(uint16_t duration, uint16_t repeat) {
  if ((duration < 1) || (repeat < 1))
    return;
  if (repeat == 1) {
    digitalWrite(BUZZER, HIGH);
    delay(duration);
    digitalWrite(BUZZER, LOW);
  }
  else {
    for (uint16_t i = 0; i < repeat - 1; i++) {
      digitalWrite(BUZZER, HIGH);
      delay(duration);
      digitalWrite(BUZZER, LOW);
      delay(duration);
    }
    digitalWrite(BUZZER, HIGH);
    delay(duration);
    digitalWrite(BUZZER, LOW);
  }
}

void setUpPinout() {
  pinMode(FRONT_LEFT_MOTOR_PWM, OUTPUT);
  pinMode(FRONT_RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(REAR_LEFT_MOTOR_PWM, OUTPUT);
  pinMode(REAR_RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(FRONT_LEFT_MOTOR_DIR, OUTPUT);
  pinMode(FRONT_RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(REAR_LEFT_MOTOR_DIR, OUTPUT);
  pinMode(REAR_RIGHT_MOTOR_DIR, OUTPUT);

  pinMode(ELEVATE_MOTOR_PWM, OUTPUT);
  pinMode(ELEVATE_MOTOR_DIR, OUTPUT);

  pinMode(LEFT_FINGER, OUTPUT);
  pinMode(RIGHT_FINGER, OUTPUT);

  pinMode(LEFT_FAN, OUTPUT);
  pinMode(RIGHT_FAN, OUTPUT);

  leftFinger.attach(LEFT_FINGER);
  rightFinger.attach(RIGHT_FINGER);

  leftFan.attach(LEFT_FAN);
  rightFan.attach(RIGHT_FAN);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  pinMode(BUZZER, OUTPUT);
}

void configController() {
  controllerError = controller.config_gamepad(CONTROLLER_CLOCK_PIN, CONTROLLER_COMMAND_PIN, CONTROLLER_ATTENTION_PIN, CONTROLLER_DATA_PIN, NO_PRESSURE, NO_RUMMBLE);
  
  if(controllerError == NO_ERROR) {
    Serial.print("Controller found and configured successfully!");
  }  
  else if(controllerError == CONTROLLER_NOT_FOUND)
    Serial.println("No controller found!");
   
  else if(controllerError == CONTROLLER_NOT_ACCEPTING_COMMANDS)
    Serial.println("Controller found but not accepting commands!");

  else if(controllerError == CONTROLLER_REFUSING_TO_ENTER_PRESSURE_MODE)
    Serial.println("Controller refusing to enter Pressures mode!");
  
  controllerType = controller.readType(); 
  switch(controllerType) {
    case UNKNOW_CONTROLLER:
      Serial.print("Unknown Controller");
      break;
    case DUALSHOCK_CONTROLLER:
      Serial.print("DualShock Controller");
      break;
    case GUITAR_HERO_CONTROLLER:
      Serial.print("GuitarHero Controller");
      break;
	  case WIRELESS_DUALSHOCK_CONTROLLER:
      Serial.print("Wireless Sony DualShock Controller");
      break;
  }
}
