This is a C++ library for Arduino.

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/libraries folder.

When installed, this library should look like:

Arduino/libraries/SerialHandler                      (this library's folder)
Arduino/libraries/SerialHandler/SerialHandler.cpp    (the library implementation file)
Arduino/libraries/SerialHandler/SerialHandler.h      (the library description file)
Arduino/libraries/SerialHandler/keywords.txt         (the syntax coloring file)
Arduino/libraries/SerialHandler/examples             (the examples in the "open" menu)
Arduino/libraries/SerialHandler/readme.txt           (this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you just have to start the Arduino application.
You may see a few warning messages as it's built.

To use this library in a sketch, go to the Sketch | Import Library menu and
select SerialHandler.  This will add a corresponding line to the top of your sketch:
#include <SerialHandler.h>

To stop using this library, delete that line from your sketch.

Geeky information:
After a successful build of this library, a new file named "Test.o" will appear
in "Arduino/lib/targets/libraries/SerialHandler". This file is the built/compiled library
code.

If you choose to modify the code for this library (i.e. "SerialHandler.cpp" or "SerialHandler.h"),
then you must first 'unbuild' this library by deleting the "SerialHandler.o" file. The
new "SerialHandler.o" with your code will appear after the next press of "verify"
