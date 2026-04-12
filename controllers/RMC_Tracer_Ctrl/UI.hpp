#pragma once

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Node.hpp>            // Header of Webots Node
#include <webots/Camera.hpp>          // Header of Webots Camera
#include <webots/Display.hpp>         // Header of Webots Display
#include <webots/LED.hpp>             // Header of Webots Led
#include "Config.hpp"                 // Config header with configuration constants (thresholds, speeds)
#include "Sensors.hpp"                // Header of Sensors
#include "State.hpp"                  // Header of State

class Display_cam {
public:
  Display_cam(webots::Supervisor *robot, int timeStep);     // Module to initialize camera, display, blue led and get self node
  void printSensor(int x, int y, int sensed);
  void printStatus(const Sensors &s,                        // Module to robot speed, robot state and sensors values
                   RobotState state);
  void getCurrentTime(webots::Supervisor *robot);           // Function to get current time and elapsed time
  bool isDone(double time);                                 // Function to determine if time has elapsed
  void resetTimer();                                        // Function to reset timer
  
private:
  webots::Camera *camera;                                   // Webots camera
  webots::Display *display;                                 // Webots display
  webots::LED *led;                                         // Webots led
  webots::Node *self;                                       // Webots self node (robot)
};
