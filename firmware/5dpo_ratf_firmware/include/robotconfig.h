#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <Arduino.h>

/******************************************************************************
 * Configuration (uncomment the appropriate configuration)
 * - Lazarus-based applications (original channels implementation)
 * - ROS-based navigation stack (modified implementation of channels with the
 *     using the ROS package serial_communication_channels
 ******************************************************************************/
//#define CONFIG_LAZARUS        //!< firmware communicates to a Lazarus app
#define CONFIG_ROS            //!< firmware communicates to a ROS-based app

/******************************************************************************
 * Robot parameters
 * - kinematic configuration
 * - battery
 ******************************************************************************/
const float kRobotL[] = {     //!< robot geometric distances
  0.200,    //!< front-back wheel distance
  0.200     //!< left-right wheel distance
};
const float kRobotWhD[] = {   //!< diameter of the wheels 0-3 (m)
  0.0600,   //!< back-right wheel
  0.0600,   //!< back-left  wheel
  0.0600,   //!< front-right wheel
  0.0600    //!< front-left  wheel
};
const float kRobotBattVnom = 11.1;    //!< nominal battery level (V)

/******************************************************************************
 * General inputs / outputs
 * - solenoid
 * - switch
 ******************************************************************************/
const uint8_t kRobotActSolenoidPin = 30;
const uint8_t kRobotSensSwitchPin = 53;

/******************************************************************************
 * Motor parameters
 * - gear reduction ratio and encoders resolution
 * - controllers parameters
 ******************************************************************************/
const float kMotNgear  = 43.8;    //!< gear reduction ratio (n:1)
const float kMotEncRes = 16*4;    //!< encoder resolution (tick count per rev.)

const uint8_t kMotEncPin0A = 22;  //!< encoder channel A of back-right wheel
const uint8_t kMotEncPin0B = 23;  //!< encoder channel B of back-right wheel
const uint8_t kMotEncPin1A = 24;  //!< encoder channel A of back-left wheel
const uint8_t kMotEncPin1B = 25;  //!< encoder channel B of back-left wheel
const uint8_t kMotEncPin2A = 26;  //!< encoder channel A of front-right wheel
const uint8_t kMotEncPin2B = 27;  //!< encoder channel B of front-right wheel
const uint8_t kMotEncPin3A = 28;  //!< encoder channel A of front-left wheel
const uint8_t kMotEncPin3B = 29;  //!< encoder channel B of front-left wheel

const float kMotModelKp  = 4.5000;    //!< gain (rad.s^(-1) / V)
const float kMotModelTau = 0.1000;    //!< time constant (s)
const float kMotModelLag = 0.0000;    //!< lag lag (s)

const unsigned long kMotCtrlFreq = 50UL;      //!< frequency (Hz)
const float kMotCtrlTime = 1.0 / kMotCtrlFreq;//!< period (s)
const unsigned long kMotCtrlTimeUs = 1000000UL / kMotCtrlFreq;
const unsigned long kMotCtrlTimeout = 100UL;  //!< watchdog timeout (ms)
const bool kMotCtrlTimeoutEnable = true;      //!< enable watchdog (true/false)

const unsigned long kMotCtrlLEDOkFreq = 4UL;  //!< heartbeat LED frequency (Hz)
const unsigned long kMotCtrlLEDOkCount =
    1000000UL / kMotCtrlLEDOkFreq / kMotCtrlTimeUs / 2;

const float kMotVmax = 12;          //!< maximum voltage appliable to motors (V)
const int16_t kMotPWMmax = 255;     //!< maximum PWM (0..1023)
const int16_t kMotPWMDeltaMax = 100;//!< maximum variation in PWM (0..1023)
const bool kMotPWMDeltaMaxEnabled = true; //!< enable limits on PWM variation

const float kMotHammV0 = 0.20;         //!< estimated motors' deadzone (V)
const float kMotHammVd = 0.12;         //!< compensated motors' deadzone (V)

//! IMC tunning: desired time constant for the closed-loop (s)
const float kMotCtrlTauCl = kMotModelTau / 1.0;
//! IMC tunning: Kc_PI * Kp_plant
const float kMotCtrlKcKp = kMotModelTau / (kMotCtrlTauCl + kMotModelLag);
//! PI proportional gain (V / rad.s^(-1))
const float kMotCtrlKc = kMotCtrlKcKp / kMotModelKp;
//! PI integration time (s)
const float kMotCtrlTi = kMotModelTau;
//! Feed-Forward constant
const float kMotCtrlKf = 0.456294584;

/******************************************************************************
 * Conversion constants
 ******************************************************************************/
//! Conversion constant: encoder pulses (ticks) > motor angular speed (rad/s)
const float kEncImp2MotW =
    2 * PI * 1000000 / (1.0 * kMotCtrlTimeUs * kMotNgear * kMotEncRes);
//! Conversion constant: motor voltage (V) > PWM (0..PWM_max)
const float kMotV2MotPWM = kMotPWMmax * 1.0 / kRobotBattVnom;

#endif
