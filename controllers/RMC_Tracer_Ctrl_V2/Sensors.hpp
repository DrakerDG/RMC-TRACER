#pragma once

#include <webots/DistanceSensor.hpp>              // Header of Webots distance sensor
#include <webots/Robot.hpp>                       // Header of Webots robot
#include "Config.hpp"                             // Config header with configuration constants (thresholds, speeds)

class Sensors {
public:
  Sensors(webots::Robot *robot, int timeStep);    // Module to initialize sensors
  void updateCalibration();                       // Module to calibration sensors
  int normalize(int i) const;                     // Function to normalize the sensor value, using min, max and normal target (NR_GS)
  double lineError();                             // Function to get the line error
  int getValue(int i) const;                      // Function to get the line sensor values
  int getGS(int i) const;                         // Function to get the normaliced line sensor value
  int getRL(int i) const;                         // Function to get the wheel sensor value (Right: 0  |  Left: 1)
  void cout_max_min();                            // Function to show the max and min sensor value    <--- to debug
  void sensor_norm();                             // Function to show the normalized sensor value     <--- to debug
  void sensor_value();                            // Function to show the normalized sensor value     <--- to debug
private:
  webots::DistanceSensor *gs[N_SEN];              // Declaration of the array of objects of the Webots ground sensors
  webots::DistanceSensor *right_s;                // Declaration of the object of the right sensor of Webots
  webots::DistanceSensor *left_s;                 // Declaration of the object of the left sensor of Webots
};
