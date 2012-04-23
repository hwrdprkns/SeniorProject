#define WIFIsrl Serial2

void setup(){
  
    Serial.begin(9600);
    WIFIsrl.begin(9600);
 
    WIFIsrl.println("");
    WIFIsrl.println("AT&F");
    WIFIsrl.println("ATE1"); //turn off echo
    WIFIsrl.print("AT+NMAC=00:1d:c9:10:39:6f\r"); //set MAC address
    WIFIsrl.println("AT+WM=0");
    //WIFIsrl.println("AT+WS");
    WIFIsrl.println("AT+NDHCP=1");
  
    /* drone's network profile, change if needed*/
    WIFIsrl.println("AT+WA=ardrone_279440");
    WIFIsrl.println("AT+NCUDP=192.168.1.1,5556");
  
    // abandon autoconnection mode
    //WIFIsrl.println("AT+NAUTO=0,0,192.168.1.3,5556");
    //WIFIsrl.println("AT+NSTAT=?");
    //WIFIsrl.println("AT+CID=?");
    //WIFIsrl.print("ATA2\r");
    
    dronePrint();
      
}



void loop(){
  
        while(WIFIsrl.available()){
          Serial.write(WIFIsrl.read());
        } 
}

void dronePrint(){
   String droneBlink = "AT*LED=1,2,1073741824,3\r\n";
   sendwifi(droneBlink);
   
}

void sendwifi(String s) {
    WIFIsrl.write(27); //esc
    WIFIsrl.print("S0"); //choose connection CID 0
    WIFIsrl.print(s);
    WIFIsrl.write(27);
    WIFIsrl.print("E");
}
  



