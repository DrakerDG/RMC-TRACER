#include "SpeedCtrl.hpp"
#include "Config.hpp"                                       // Config header with configuration constants (thresholds, speeds)

using namespace webots;

SpeedCtrl::SpeedCtrl(Supervisor *robot, int initSpeed) {
  L_motor = robot->getMotor("L_motor::gear");               // Get device (L_motor::gear) coupled gear and left motor
  R_motor = robot->getMotor("R_motor::gear");               // get device (R_motor::gear) coupled gear and right motor
  L_motor->setPosition(INFINITY);                           // Set position to continuous motor (L_motor)
  R_motor->setPosition(INFINITY);                           // Set position to continuous motor (R_motor)
  L_motor->setVelocity(initSpeed);                          // Set velocity 0 rad/s (L_motor)
  R_motor->setVelocity(initSpeed);                          // Set velocity 0 rad/s (R_motor)    

  current_Speed = initSpeed;                                // Set current speed
   target_Speed = initSpeed;                                // Set target speed
}

void SpeedCtrl::setTarget(int target) {                     // Function to set target speed
  target_Speed = target;
  switch (target) {
    case SP001: limit_Speed = SP002; break;
    default:    limit_Speed = SP002;
  }

}

void SpeedCtrl::update() {                                  // Function to update the current speed using the target speed
  if (current_Speed < target_Speed) {
    current_Speed += ST_CH;
    if (current_Speed > target_Speed)
      current_Speed = target_Speed;
  }
  else if (current_Speed > target_Speed) {
    current_Speed -= ST_CH;
    if (current_Speed < target_Speed)
      current_Speed = target_Speed;
  }
}

void SpeedCtrl::setSpeed(int  leftSpeed,                    // Function to set the speed motors
                         int rightSpeed) {
  L_motor->setVelocity( leftSpeed);                         // Set the speed (L_motor)
  R_motor->setVelocity(rightSpeed);                         // Set the speed (R_motor)
}

int SpeedCtrl::getSpeed() const {                           // Function to get the current speed
  return current_Speed;
}

int SpeedCtrl::getLimit() const {                           // Function to get the limit speed
  return limit_Speed;
}

