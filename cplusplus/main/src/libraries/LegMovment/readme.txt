This is a C++ library for Arduino.

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/libraries folder.

When installed, this library should look like:

Arduino/libraries/LegMovment                                    (this library's folder)
Arduino/libraries/LegMovment/LegMovment.cpp                     (the library implementation file)
Arduino/libraries/LegMovment/LegMovment.h                       (the library description file)
Arduino/libraries/LegMovment/keywords.txt                       (the syntax coloring file)
Arduino/libraries/LegMovment/examples                           (the examples in the "open" menu)
Arduino/libraries/LegMovment/readme.txt                         (this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you just have to start the Arduino application.
You may see a few warning messages as it's built.

To use this library in a sketch, go to the Sketch | Import Library menu and
select LegMovment.  This will add a corresponding line to the top of your sketch:
#include <LegMovment.h>

To stop using this library, delete that line from your sketch.

Geeky information:
After a successful build of this library, a new file named "Test.o" will appear
in "Arduino/lib/targets/libraries/LegMovment". This file is the built/compiled library
code.

If you choose to modify the code for this library (i.e. "LegMovment.cpp" or "LegMovment.h"),
then you must first 'unbuild' this library by deleting the "LegMovment.o" file. The
new "LegMovment.o" with your code will appear after the next press of "verify"

