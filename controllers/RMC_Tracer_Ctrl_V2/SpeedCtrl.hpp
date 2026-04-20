#pragma once

#include <webots/Supervisor.hpp>                  // Header of Webots Robot
#include <webots/Motor.hpp>                       // Header of Webots motor
#include "Config.hpp"                             // Config header with configuration constants (thresholds, speeds)

class SpeedCtrl {
public:
  SpeedCtrl(webots::Supervisor *robot, int initSpeed);

  void setTarget(int target);                     // Function to set the speed motors
  void update();                                  // Function to update the current speed using the target speed
  void setSpeed(int leftSpeed, int rightSpeed);   // Module to update the speed of the motors.
  
  int  getSpeed() const;                          // Function to get the current speed
  int  getLimit() const;                          // Function to get the limit speed

private:
  webots::Motor *L_motor;                         // Declaration of the object of the L_motor::gear of Webots
  webots::Motor *R_motor;                         // Declaration of the object of the R_motor::gear of Webots
  int current_Speed;                              // Variable of current speed
  int  target_Speed;                              // Variable of target speed
  int   limit_Speed;                              // Variable of limit speed
};
