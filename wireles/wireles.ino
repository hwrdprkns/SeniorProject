#include "Wirefree.h"

WIFI_PROFILE wirelessProfile = { "ARDrone_?",       /* SSID */
                        "" ,        /* WPA/WPA2 passphrase */
                        "192.168.1.2" ,   /* IP address */
                        "255.255.255.0" ,   /* subnet mask */
                        "192.168.1.1"   ,   /* Gateway IP */
                      };



String droneServer = "192.168.1.1";
int droneDefaultPort = 80;

WifiClient drone(droneServer,droneDefaultPort);


void parseRxData(String data){}

void setup(){
  
	Wireless.begin(&wirelessProfile,&parseRxData);
	if(drone.connect()){
		Serial.println("Holy shit I just came EVERYWHERE.");
	} else {
		Serial.println("Fucking failure.");
	}
  
}

void loop(){
	
	while(drone.available()){
		Serial.write(drone.read());
	}
	
        drone.stop();

}
