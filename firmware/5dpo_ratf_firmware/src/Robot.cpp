#include "Robot.h"

void Robot::init(void (*serialWriteChannelFunction)(char c, int32_t v)) {
  uint8_t i;

  // Serial write channels function
  serialWriteChannel = serialWriteChannelFunction;

  // General inputs / outputs
  // - solenoid
  pinMode(kRobotSensSwitchPin, INPUT_PULLUP);
  pinMode(kRobotActSolenoidPin, OUTPUT);
  digitalWrite(kRobotActSolenoidPin, 0);

  // Encoders
  initEnc();
  updateEncodersState();
  for (i = 0; i < 4; i++) {
    encoders[i].delta = 0;
  }
  Timer1.attachInterrupt(updateEncodersState);
  Timer1.initialize(20);  // calls every X us

  // Motors
  AFMS.begin();
  for (i = 0; i < 4; i++) {
    mot[i].init(AFMS.getMotor(i + 1));
  }

  // Controllers
  for (i = 0; i < 4; i++) {
    initCtrlPID(i);
  }
}

void Robot::update(uint32_t &delta) {
  uint8_t i;
  dt = delta;

  // Encoders
  for (i = 0; i < 4; i++) {
    enc[i].updateTick();
  }

  // Controllers
  for (i = 0; i < 4; i++) {
    pid[i].update(enc[i].odo * kEncImp2MotW);
  }

  // Actuators
  for (i = 0; i < 4; i++) {
    if (pid[i].active) {
      mot[i].setPWM( round( kMotV2MotPWM * pid[i].m ) );
    }
  }
}

void Robot::send(void) {
  (*serialWriteChannel)('g', enc[0].tick);
  (*serialWriteChannel)('h', enc[1].tick);
  (*serialWriteChannel)('i', enc[2].tick);
  (*serialWriteChannel)('j', enc[3].tick);

  (*serialWriteChannel)('k', dt);

  (*serialWriteChannel)('s', (digitalRead(kRobotSensSwitchPin) << 0));
}

void Robot::stop(void) {
  uint8_t i;

  for (i = 0; i < 4; i++) {
    setMotorPWM(i, 0);
  }
}

void Robot::setMotorWref(uint8_t index, float new_w_r) {
  pid[index].enable(true);
  pid[index].w_ref = new_w_r;
}

void Robot::setMotorPWM(uint8_t index, int16_t pwm) {
  pid[index].enable(false);
  mot[index].setPWM(pwm);
}

void Robot::initEnc() {
  pinMode(kMotEncPin0A, INPUT_PULLUP);
  pinMode(kMotEncPin0B, INPUT_PULLUP);
  pinMode(kMotEncPin1A, INPUT_PULLUP);
  pinMode(kMotEncPin1B, INPUT_PULLUP);
  pinMode(kMotEncPin2A, INPUT_PULLUP);
  pinMode(kMotEncPin2B, INPUT_PULLUP);
  pinMode(kMotEncPin3A, INPUT_PULLUP);
  pinMode(kMotEncPin3B, INPUT_PULLUP);
}

void Robot::initCtrlPID(uint8_t index) {
  pid[index].active = false;
  pid[index].kp = kMotCtrlKc;
  if (kMotCtrlTi == 0) {
    pid[index].ki = 0;
  } else {
    pid[index].ki = kMotCtrlKc / kMotCtrlTi;
  }
  pid[index].kd = 0;
  pid[index].kf = kMotCtrlKf;
  pid[index].dt = kMotCtrlTime;

  pid[index].m_max = kMotVmax;

  pid[index].hamm_vd = 0;
  pid[index].hamm_v0 = 0;

  pid[index].reset();
}
