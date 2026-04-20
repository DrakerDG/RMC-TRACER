#pragma once

#include <webots/Supervisor.hpp>           // Header of Webots Supervisor
#include <webots/Speaker.hpp>              // Header of Webots Speaker

class Sound {
public:
  Sound(webots::Supervisor *robot);        // Module to initialize speaker
  void play(int track);                    // Function to play sounds

private:
  webots::Speaker *buzzer;                 // Buzzer object declaration
};
