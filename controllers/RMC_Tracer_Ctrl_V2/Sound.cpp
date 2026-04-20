#include "Sound.hpp"                          // Header of Sound

using namespace webots;

Sound::Sound(Supervisor *robot) {             // Module to initialize speaker (buzzer)
  buzzer = robot->getSpeaker("buzzer");
}

void Sound::play(int track) {                 // Function to text to speech and play sounds
  if (track == 0)
    buzzer->speak("Starting sensor calibration!", 1.0);
  else if (track == 1)
    buzzer->speak("Calibration complete!", 1.0);
  else if (track == 2)
    buzzer->speak("Let's start by following the line!", 1.0);
  else if (track == 3)
    buzzer->playSound(buzzer, buzzer, "sounds/beep.wav", 1, 1, 0, false);
}
