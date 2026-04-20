#include "UI.hpp"                                                // Header of UI
#include <cmath>                                                 // Math library to make math operations like sqrt 
#include <iomanip>                                               // Format library to use setw to set some formats with decimals
#include <string>                                                // String library to easy way to handle text 
#include <cstdio>                                                // I/O library to manage data input and output.
#include "Config.hpp"                                            // Header of Config (configuration constants like thresholds, speeds, PID keyes, etc.)
#include "Graph.hpp"                                             // Header of Graph

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

UI::UI(Supervisor *robot, int timeStep)                          // Module to initialize camera, display, blue led and get self node 
  : alphaGraph(100, 0.0, 1.0), 
    alphaFilteredGraph(100, 0.0, 1.0)
{

  self = robot->getSelf();                                       // Get self robot
  
  // Initialize camera
  camera = robot->getCamera("camera");                           // Get device (camera)
  camera->enable(timeStep * 4);                                  // Set time step x 4 to camera

  // Initialize display
  display = robot->getDisplay("display");                        // Get device (display)
  display->setFont("Lucida Console", 10, true);                  // Set font, size and antialiasing
  display->drawText("RMC-TRACER [Webots]", L_Margin, T_Margin);
  
  // Initialize monitor
  monitor = robot->getDisplay("monitor");                        // Get device (monitor)
  monitor->setFont("Lucida Console", 10, true);                  // Set font, size and antialiasing
  monitor->setColor(0xFFB400);
  monitor->fillRectangle(240, 45, 10, 10);
  monitor->drawText("α", 255, 45);
  monitor->setColor(0x00FF00);
  monitor->fillRectangle(240, 130, 10, 10);
  monitor->drawText("LPF(α)", 255, 130);
  monitor->setColor(0x9E9E9E);
  monitor->drawText("Kp   Ki   Kd", 320, 160);
  monitor->setColor(0x42A5F5);
  monitor->drawText("PID", 437, 160);

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

void UI::printSensor(int x, int y, int sensed){                  // Function to print sensor bars
  int height = (int) sensed * S_height / NR_GS;
  int y_pos = y + S_height - height;
  if (height > 0) {
    display->fillRectangle(x, y_pos, S_width, height);
  }
  else {
    display->drawLine(x, y_pos, x + S_width, y_pos);
  }
}

void UI::printKey(int x, int y, double val_key, int color){      // Function to print PID and keyes bars
  double val_lim = val_key;
  if (val_lim > 1) val_lim = 1;
  
  int rem_val = K_height - K_height * val_lim + 2 * (led_hgt + led_pad);
  
  int segments = K_height / (led_hgt + led_pad);
  
  for (int i = 0; i < segments; i++) {
    int seg_y = y + K_height - ((i + 1) * (led_hgt + led_pad));
    if (seg_y > rem_val) {
      monitor->setColor(color);
    }
    else {
      monitor->setColor(0x202020);
    }
    monitor->fillRectangle(x, seg_y, K_width, led_hgt);
  }
}

void UI::printGrid(int x, int y, int r_width, int r_height){     // Function to print grid lines
  int H_lines = r_height / Gap_line + 1;
  int V_lines = r_width / Gap_line + 1;

  monitor->setColor(0x202020);
  for (int i = 0; i < (H_lines); i++) {
    monitor->drawLine(x ,  y + Gap_line * i, x + r_width, y + Gap_line * i);
  } 
  for (int i = 0; i < (V_lines); i++) {
    monitor->drawLine(x + Gap_line * i,  y, x + Gap_line * i, y + r_height);
  }  
}

void UI::printStatus(const Sensors &s,                           // Fucntion to print robot speed, robot state and sensors values
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
  
  /*
  // To caputre display
  if(current_time == 0.016) {
    webots::ImageRef *ir = display->imageCopy(0, 0, display->getWidth(), display->getHeight());
    display->imagePaste(ir, 0, 0, false);
    display->imageSave(ir, "display01.png");
  }
  */
  
  // to debug
  //std::string timertxt =  std::to_string(current_time);     //   <--- to debug
  //std::cout << "current_time: " << timertxt << std::endl;   //   <--- to debug  
}

void UI::pritGraph(const PIDState& PID_state,                    // Function to print Graph  
                             bool graph_mode) {

  monitor->setColor(0x000000);
  monitor->fillRectangle(L_Margin, T_Margin, 220, monitor->getHeight() - 2*T_Margin + 1);
  
  printGrid(L_Margin, T_Margin, 220, monitor->getHeight() - 2*T_Margin + 1);
  
  // to debug
  /*
  std::string strS = "alpha: " + std::to_string(PID_state.alpha) 
                + "  alpha_f: " + std::to_string(PID_state.alpha_filtered) 
                + "  |  Kp: " + std::to_string(PID_state.Kp / KI_STRONG)
                + "  Ki: " + std::to_string(PID_state.Ki / KI_STRONG)
                + "  Kd: " + std::to_string(PID_state.Kd / KI_STRONG * 500)
                + "  |  PID: " + std::to_string(fabs(PID_state.PID_value)/100);
  std::cout << strS << std::endl;
  */
  
  alphaGraph.addValue(PID_state.alpha);
  alphaFilteredGraph.addValue(PID_state.alpha_filtered);

  int y1 = T_Margin;
  // Overlapping charts
  int y2 = y1;
  int height_g = 160;
  
  if (graph_mode) { // Separate graphs
    y2 = 95;
    height_g = 75;
  }

  auto points_alpha = alphaGraph.getPoints(L_Margin, y1, 222, height_g);
  auto points_filtered = alphaFilteredGraph.getPoints(L_Margin, y2, 222, height_g);

  for (size_t i = 0; i < points_alpha.size() - 1; i++) {
    monitor->setColor(0xFFB400);
    monitor->drawLine(
      points_alpha[i].x, points_alpha[i].y,
      points_alpha[i+1].x, points_alpha[i+1].y
    );
    monitor->setColor(0x00FF00);
    monitor->drawLine(
      points_filtered[i].x, points_filtered[i].y,
      points_filtered[i+1].x, points_filtered[i+1].y
    );
  }
  printKey(320, 20, PID_state.Kp / KI_STRONG, 0x9E9E9E);
  printKey(360, 20, PID_state.Ki / KI_STRONG, 0x9E9E9E);
  printKey(400, 20, 400 * PID_state.Kd / KI_STRONG, 0x9E9E9E);
  printKey(440, 20, fabs(PID_state.PID_value) / 90, 0x42A5F5);

}

void UI::getCurrentTime(Supervisor *robot) {                     // Function to get current time and elapsed time
  current_time = robot->getTime();
  elapsed = current_time - state_start_time;
}

bool UI::isDone(double time) {                                   // Function to determine if time has elapsed
  return elapsed >= time;
}

void UI::resetTimer() {                                          // Function to reset timer
  state_start_time = current_time;
  //std::string timertext =  std::to_string(elapsed);     //   <--- to debug
  //std::cout << "elapsed: " << timertext << std::endl;   //   <--- to debug
}
