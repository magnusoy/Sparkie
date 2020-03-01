This are C++ libraries for Arduino.

Installation
--------------------------------------------------------------------------------

To install thelibraries, just place this entire folder as a subfolder in your
Arduino/libraries folder, or convert them each to .zip folders, and import them
through the Arduino IDE.

When installed, this library should look like:

Arduino/libraries/Arduino-ButtonTimer                        (this library's folder)
Arduino/libraries/Arduino-ButtonTimer/ButtonTimer.cpp        (the library implementation file)
Arduino/libraries/Arduino-ButtonTimer/ButtonTimer.h          (the library description file)
Arduino/libraries/Arduino-ButtonTimer/keywords.txt           (the syntax coloring file)
Arduino/libraries/Arduino-ButtonTimer/examples               (the examples in the "open" menu)
Arduino/libraries/Arduino-ButtonTimer/readme.txt             (the readme file)

Arduino/libraries/PID_Controller                             (this library's folder)
Arduino/libraries/PID_Controller/PID.cpp              	     (the library implementation file)
Arduino/libraries/PID_Controller/PID.h               	     (the library description file)
Arduino/libraries/PID_Controller/keywords.txt                (the syntax coloring file)
Arduino/libraries/PID_Controller/examples                    (the examples in the "open" menu)
Arduino/libraries/PID_Controller/readme.txt                  (the readme file)

Arduino/libraries/SerialHandler                              (this library's folder)
Arduino/libraries/SerialHandler/SerialHandler.cpp            (the library implementation file)
Arduino/libraries/SerialHandler/SerialHandler.h              (the library description file)
Arduino/libraries/SerialHandler/keywords.txt                 (the syntax coloring file)
Arduino/libraries/SerialHandler/examples                     (the examples in the "open" menu)
Arduino/libraries/SerialHandler/readme.txt                   (the readme file)

Arduino/libraries/Timer                                      (this library's folder)
Arduino/libraries/Timer/Timer.cpp                            (the library implementation file)
Arduino/libraries/Timer/Timer.h                              (the library description file)
Arduino/libraries/Timer/keywords.txt                         (the syntax coloring file)
Arduino/libraries/Timer/examples                             (the examples in the "open" menu)
Arduino/libraries/Timer/readme.txt                           (the readme file)

Building
--------------------------------------------------------------------------------

After the libraries are installed, you just have to start the Arduino application.
You may see a few warning messages as it's built.

To use this library in a sketch, go to the Sketch | Import Library menu and
select ButtonTimer, PID, SerialHandler, Timer.