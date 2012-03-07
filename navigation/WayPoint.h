
#ifndef Waypoint_h
#define Waypoint_h

#include <Arduino.h>

class WayPoint{
  
  public:
  
    static double computeDistance(double lat1, double lon1,double lat2, double lon2);
    static double computeInitialBearing(double lat1, double lon1,double lat2, double lon2);
    static double computeFinalBearing(double lat1, double lon1,double lat2, double lon2);
    
};

#endif
