#include "UI.hpp"                                                // Header of UI
#include <cmath>                                                 // Math library to make math operations like sqrt 
#include <iomanip>                                               // Format library to use setw to set some formats with decimals
#include <string>                                                // String library to easy way to handle text 
#include <cstdio>                                                // I/O library to manage data input and output.
#include "Config.hpp"                                            // Config header with configuration constants (thresholds, speeds)

using namespace webots;
using namespace std::string_literals;

static std::string stateToStr(RobotState s) {                    // Function to convert the robot state to string
  switch (s) {
    case RobotState::CALIBRATION:   return "CALIBRATION";
    case RobotState::FOLLOWER:      return "FOLLOWER";
    case RobotState::STOP:          return "STOP";
    default:                        return "UNKNOWN";
  }
}

double current_time = 0;                                         // Variable to obtain the current time
double state_start_time = 0;                                     // Variable to set the state start time
double elapsed = 0;                                              // Variable to set the elapsed time

Display_cam::Display_cam(Supervisor *robot, int timeStep) {      // Module to initialize camera, display, blue led and get self node 

  self = robot->getSelf();                                       // Get self robot
  
  // Initialize camera
  camera = robot->getCamera("camera");                           // Get device (camera)
  camera->enable(timeStep * 4);                                  // Set time step x 4 to camera

  // Initialize display
  display = robot->getDisplay("display");                        // Get device (display)
  display->setFont("Lucida Console", 10, true);                  // Set font, size and antialiasing
  display->drawText("RMC-TRACER [Webots]", L_Margin, T_Margin);
  
  // Initialize blue led
  led = robot->getLED("led");                                    // Get device (blue led)
  led->set(1);                                                   // Set ON
}

std::string hms(double sec) {
    int h = static_cast<int>(sec / 3600);
    int m = static_cast<int>((static_cast<int>(sec) % 3600) / 60);
    int s = static_cast<int>(static_cast<int>(sec) % 60);
    int c = static_cast<int>((sec - static_cast<int>(sec)) * 100);

    char buffer[15];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d.%02d", h, m, s, c);
    
    return std::string(buffer);
}

void Display_cam::printSensor(int x, int y, int sensed){
  int height = (int) sensed * S_height / 1000;
  int y_pos = y + S_height - height;
  if (height > 0) {
    display->fillRectangle(x, y_pos, S_width, height);
  }
  else {
    display->drawLine(x, y_pos, x + S_width, y_pos);
  }
}

void Display_cam::printStatus(const Sensors &s,                  // Module to robot speed, robot state and sensors values
                              RobotState state) {

  display->setColor(0x000000);
  display->fillRectangle(0, T_Margin * 3, display->getWidth(), display->getHeight() - T_Margin * 3);
  display->setColor(0xFFFFFF);

  std::string timertext = "Timer:   "s + hms(current_time);
  display->drawText(timertext, L_Margin, T_Margin + P_Spacing);
 
  std::string elapsedtext = "Elapsed: "s + hms(elapsed);
  display->drawText(elapsedtext, L_Margin, T_Margin + P_Spacing*2);

  std::string stateText = "State: " + stateToStr(state);
  display->drawText(stateText, L_Margin, T_Margin + P_Spacing*3);
  
  const double *v = self->getVelocity();
  double speed = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%.2f", speed);
  std::string text = "Speed: "s + buffer + " m/s"; 
  display->drawText(text, L_Margin, T_Margin + P_Spacing*4);

  //                    L  3  2  1  0  R
  //                  L █| █  █  █  █ |█ R
  std::string strS = "L  |            |  R";
  display->drawText(strS, L_Margin, T_Margin + P_Spacing*5);
  
  for (int i = 0; i < (N_SEN+2); i++) {
    int sensor_value = 0;
    if (i == 0) {
      if (s.getRL(1) > THOLD) sensor_value = NR_GS;
    }
    else {
      if (i == 5) {
        if (s.getRL(0) > THOLD) sensor_value = NR_GS;
      }
      else sensor_value = s.getGS(N_SEN - i);
    }
    printSensor(S_Margin + S_Spacing*i, T_Margin + P_Spacing*5-1, sensor_value);
  }
}

void Display_cam::getCurrentTime(Supervisor *robot) {            // Function to get current time and elapsed time
  current_time = robot->getTime();
  elapsed = current_time - state_start_time;
}

bool Display_cam::isDone(double time) {                          // Function to determine if time has elapsed
  return elapsed >= time;
}

void Display_cam::resetTimer() {                                 // Function to reset timer
  state_start_time = current_time;
  //std::string timertext =  std::to_string(elapsed);     //   <--- to debug
  //std::cout << "elapsed: " << timertext << std::endl;   //   <--- to debug
}
