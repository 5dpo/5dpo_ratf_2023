#include <Arduino.h>

#include <channels.h>

#include "Robot.h"

/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

unsigned long current_micros = 0, previous_micros = 0;
unsigned long last_motor_update_millis = 0;
bool timeout = false;
channels_t serial_channels;
//uint8_t builtin_led_state;

Robot robot;
Encoder *encoders = robot.enc;



/******************************************************************************
 * FUNCTIONS HEADERS
 ******************************************************************************/
void processSerialPacket(char channel, uint32_t value, channels_t& obj);
void serialWrite(uint8_t b);
void serialWriteChannel(char channel, int32_t value);
void serialRead();
void checkMotorsTimeout();



/******************************************************************************
 * IMPLEMENTATION
 ******************************************************************************/
void setup() {
  // Built-in LED
  /*builtin_led_state = LOW;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, builtin_led_state);*/

  // Robot
  robot.init(serialWriteChannel);

  // Serial communication
  Serial.begin(115200);
  serial_channels.init(processSerialPacket, serialWrite);

  // Reset signal
  serialWriteChannel('r', 0);

  // Test PWM motors
  /*robot.setMotorPWM(0, 0);
  robot.setMotorPWM(1, 0);
  robot.setMotorPWM(2, 0);
  robot.setMotorPWM(3, 0);*/

  // Initialization
  current_micros = micros();
  previous_micros = current_micros;
  last_motor_update_millis = millis();
}

void loop() {
  //static unsigned long blink_led_decimate = 0;
  uint32_t delta;

  serialRead();

  current_micros = micros();
  delta = current_micros - previous_micros;
  if (delta > kMotCtrlTimeUs) {
    if (kMotCtrlTimeoutEnable) {
      checkMotorsTimeout();
    }

    if (!timeout) {
      previous_micros = current_micros;
      
      // Update and send data
      robot.update(delta);
      robot.send();

      // Debug (Serial Monitor)
      //serialWrite('\n');

      // Blink LED
      /*blink_led_decimate++;
      if (blink_led_decimate >= kMotCtrlLEDOkCount) {
        if (builtin_led_state == LOW) {
          builtin_led_state = HIGH;
        } else {
          builtin_led_state = LOW;
        }
        digitalWrite(LED_BUILTIN, builtin_led_state);
        blink_led_decimate = 0;
      }*/
    }
  }
}



/******************************************************************************
 * FUNCTIONS IMPLEMENTATIONS
 ******************************************************************************/
void processSerialPacket(char channel, uint32_t value, channels_t& obj) {
  uint8_t mot_i;
  int16_t pwm;

  // Reset watchdog
  if ((channel == 'G') || (channel == 'K')) {
    last_motor_update_millis = millis();
  }

  // Process incomming serial packet
  switch (channel) {
    // - reference angular speed
    case 'G':
    case 'H':
    case 'I':
    case 'J':
      mot_i = channel - 'G';
      // set reference angular speed for the motors
#ifdef CONFIG_LAZARUS
      robot.setMotorWref(mot_i, ((int32_t) value) * kEncImp2MotW );
#endif
#ifdef CONFIG_ROS
      robot.setMotorWref(mot_i, *((float*) &value) );
#endif
      break;
  
    // - PWM
    case 'K':
      mot_i = (value >> 24) & 0x03;
      pwm = value & 0xFFFF;
      robot.setMotorPWM(mot_i, pwm);
      break;

    // - solenoid
    case 'L':
      digitalWrite(kRobotActSolenoidPin, value);
      break;
  }
}

void serialWrite(uint8_t b) {
  Serial.write(b);
}

void serialWriteChannel(char channel, int32_t value) {
  serial_channels.send(channel, value);
}

void serialRead() {
  uint8_t serial_byte;

  if (Serial.available() > 0) {
    serial_byte = Serial.read();
    serial_channels.StateMachine(serial_byte);
  }
}

void checkMotorsTimeout() {
  if (millis() - last_motor_update_millis > kMotCtrlTimeout) {
    timeout = true;

    robot.stop();
    digitalWrite(kRobotActSolenoidPin, 0);

    /*builtin_led_state = LOW;
    digitalWrite(LED_BUILTIN, builtin_led_state);*/

  } else {
    if (timeout) {
      robot.init(serialWriteChannel);
    }

    timeout = 0;
  }
}
