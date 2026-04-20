#pragma once

#include <webots/Supervisor.hpp>                              // Header of Webots Supervisor
#include <webots/Node.hpp>                                    // Header of Webots Node
#include <webots/Camera.hpp>                                  // Header of Webots Camera
#include <webots/Display.hpp>                                 // Header of Webots Display
#include <webots/LED.hpp>                                     // Header of Webots Led
#include "Config.hpp"                                         // Config header with configuration constants (thresholds, speeds)
#include "Sensors.hpp"                                        // Header of Sensors
#include "State.hpp"                                          // Header of State
#include "PID.hpp"                                            // Header of PID
#include "Graph.hpp"                                          // Header of Graph

class UI {
public:
  UI(webots::Supervisor *robot, int timeStep);                // Module to initialize camera, display, blue led and get self node
  void printStatus(const Sensors &s,                          // Fucntion to print robot speed, robot state and sensors values
                   RobotState state);
  void pritGraph(const PIDState& PID_state,                   // Function to print Graph 
                    bool graph_mode);
  void getCurrentTime(webots::Supervisor *robot);             // Function to get current time and elapsed time
  bool isDone(double time);                                   // Function to determine if time has elapsed
  void resetTimer();                                          // Function to reset timer
  
private:
  void printSensor(int x, int y, int sensed);                 // Function to print sensor bars
  void printKey(int x, int y, double val_key, int color);     // Function to print PID and keyes bars
  void printGrid(int x, int y, int r_width, int r_height);    // Function to print grid lines
  webots::Camera *camera;                                     // Declaration of the object of the camera of Webots
  webots::Display *display;                                   // Declaration of the object of the display of Webots
  webots::Display *monitor;                                   // Declaration of the object of the monitor of Webots
  webots::LED *led;                                           // Declaration of the object of the LED of Webots
  webots::Node *self;                                         // Declaration of the object of the self node of Webots
  Graph alphaGraph;                                           // Declaration of the object of the alphaGraph
  Graph alphaFilteredGraph;                                   // Declaration of the object of the alphaFilteredGraph
};
