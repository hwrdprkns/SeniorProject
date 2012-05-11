AR.Drone WayPoint Navigation Senior Project
===========================================

#File/Folder Information

##/navigation_statemachine

This is where all the navigation code is stored. This folder interfaces the Command library with the Navigation algorithm to make the Drone travel the path. 

###Command.h

This is the command library file. It is an abstract interface for the "AT*" commands that the AR.Drone recognizes.

###Streaming.h

This is a simple (external) streaming library. It allows one to simply "pipe" out most variables (i.e. Serial << "Print me:" << 23456 ) All credit to the creators who can be found in the header of the file.

###navigation_statemachine.ino

This is the main Arduino sketch for the navigation algorithm. Dependencies are referenced inside the sketch. 

###TinyGPS.h

This is another external library which allows efficient parsing and reading of pertinent information in the GPS sentences delivered by our GPS receiver. This source code was not initially created by us, however it was modified for our needs. It now only receives GPRMC updates to correct the course. The specific code modifications can be found in the commit history of this file.ls

## /ATCommand

This is a folder where we test changes to our command library. The command files here are identical to the navigation_statemachine folder. This is because Arduino doesn't currently support referencing files from different folder's than the sketch folder. 

### ATCommand.ino

This is the test file for our command library. 

### Command.h

Same as in /navigation_statemachine.

### Streaming.h 

Same as in /navigation_statemachine.

Credits
-------

Electrical and Computer Engineering Department
Tufts University

Copyright 2011-2012
All rights reserved. 

* [Taylor H. Perkins](http://about.me/hwrdprkns)
* Weiyi Zheng
* Yorman Garcia
* Farhan Shaukat

Please contact us if you would like to use this code, as there are some binary files omitted here that are necessary for functioning. 
