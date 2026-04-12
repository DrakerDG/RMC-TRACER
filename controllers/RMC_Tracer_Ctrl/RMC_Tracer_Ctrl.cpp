// File:          RMC_Tracer_Ctrl.cpp |  Main  |
// Date:          April 2026
// Description:   Controller in C++ to 3D model of RMC-TRACER
// Author:        DrakerDG
// Modifications: 2.1 Beta
//        ____             __             ____  ______
//       / __ \_________ _/ /_____  _____/ __ \/ ____/
//      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
//     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
//    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
//       

#include <iostream>                   // Input/Output Streams to use cout to debug routines
#include <algorithm>                  // Algorithm library
#include <webots/Supervisor.hpp>      // Header of Webots Supervisor

#include "Config.hpp"                 // Config header with configuration constants (thresholds, speeds)
#include "State.hpp"                  // State header with robot state and stop state (pause)
#include "Sensors.hpp"                // Sensors header with class to calibrate, normalize and determine curve marking detection
#include "PID.hpp"                    // PID header with class to set PID keys and calculate PID error
#include "SpeedCtrl.hpp"              // SpeedCtrol header with class to set the limit and target speed to update the base speed robot 
#include "Sound.hpp"                  // Sound header with class to play sounds
#include "UI.hpp"                     // User Interface header with class to print status of robot in 3D viewport


using namespace webots;

// Main module of Webots
int main(int argc, char **argv) {

  Supervisor *robot = new Supervisor();          // To supervisor mode
  int timeStep = robot->getBasicTimeStep();      // Set time step

  // Initialize motors
  SpeedCtrl speedCtrl(robot, 0);                 // Get device (motors) through SpeedCtrl class

  // Initialize sensors
  Sensors sensors(robot, timeStep);              // Get device (sensors) through Sensor class

  // Initialize camera, display and led
  Display_cam display_cam(robot, timeStep);      // Get device (camera) through Display_cam class

  // Initialize speaker
  Sound sound(robot);                            // Get device (speaker) through Sound class

  RobotState nextState = RobotState::STOP;       // Set next state to stop run
  RobotState state = RobotState::CALIBRATION;    // Set current state to calibration

  //PID pid(0.06, 0.24, 0.000024);                   // Set PID on soft values (Speed 220 rad/s)
  PID pid(0.09, 0.36, 0.000036);                   // Set PID on medium values (Speed 220 rad/s)
  //PID pid(0.12, 0.48, 0.000048);                   // Set PID on hard values (Speed 220 rad/s)
  
  // Main loop:
  while (robot->step(timeStep) != -1) {

    double error = sensors.lineError();                     // Calculate the error sensed using sensors class
    double correction = pid.update(error, timeStep);        // Calculate the correction error using PID class
    
    speedCtrl.update();                                     // Update the speed limit gradually

    int baseSpeed = speedCtrl.getSpeed();                   // Get the current speed
    int limit = speedCtrl.getLimit();                       // Get the limit speed
    
    int leftSpeed  = baseSpeed + correction;                // Determine the  left speed with correction
    int rightSpeed = baseSpeed - correction;                // Determine the right speed with correction   

    // To debug:
    //std::string leftSpeed_text = std::to_string(leftSpeed);     //   <--- to debug
    //std::string rightSpeed_text = std::to_string(rightSpeed);     //   <--- to debug
    //std::cout << "leftSpeed: " << leftSpeed_text << "    rightSpeed: " << rightSpeed_text << std::endl;   //   <--- to debug
  
    leftSpeed  = std::clamp(leftSpeed,  -limit, limit);     // Limits the  left speed within a defined range
    rightSpeed = std::clamp(rightSpeed, -limit, limit);     // Limits the right speed within a defined range
    
    switch (state) {
    
    case RobotState::CALIBRATION:
      sensors.updateCalibration();                                  // Update the calibration of each sensor, recording the maximum and minimum values ​​detected
      
      speedCtrl.setSpeed(SP000, -SP000);                            // Set the calibration speed (motor_L and motor_R)

      if (display_cam.isDone(CALIB_DLY)) {                              // If the calibration has finished
        pid.reset();                                                // Reset PID
        speedCtrl.setTarget(0);                                     // Set the target speed to 0 rad/s
        nextState  = RobotState::FOLLOWER;                          // Set next state to initial run
        state      = RobotState::STOP;                              // Set current state to stop run
        display_cam.resetTimer();
        sound.play(0);

        // to debug:
        //sensors.cout_max_min();                                     //   <--- to debug

      }
      
      break;

    case RobotState::FOLLOWER:
      speedCtrl.setSpeed(leftSpeed, rightSpeed);                    // Set the calibration speed (motor_L and motor_R)
      
      // to debug:
      //sensors.sensor_norm();                                       //   <--- to debug

      break;

    case RobotState::STOP:                                          // Stop run state, when the robot pauses or stops
      speedCtrl.setSpeed(leftSpeed, rightSpeed);

      if (display_cam.isDone(STOP_DLY)) {                                // If the pause counter is longer than the stop time
        speedCtrl.setTarget(SP001);                                 // Set the target speed to SP001 (low speed)
        state = nextState;                                       // Set the next state
        display_cam.resetTimer();
        sound.play(0);
      }

      break;
    }

    display_cam.getCurrentTime(robot);
    display_cam.printStatus(sensors, state);                 // Print rspeed, robot state and sensors values

  };

  delete robot;
  return 0;
}
