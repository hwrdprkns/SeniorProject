void setup(){
  
        Serial.begin(9600);
        WIFIsrl.begin(9600);
 
        WIFIsrl.print("+++");
        delay(1250);
        WIFIsrl.println("");
        WIFIsrl.println("AT&F");
        WIFIsrl.println("ATC1");
        WIFIsrl.println("AT+WM=0");
        WIFIsrl.println("AT+WS");
        WIFIsrl.println("AT+WA=ardrone_154516");
        WIFIsrl.println("AT+NDHCP=1");
        
        WIFIsrl.println("AT+NAUTO=0,0,192.168.1.1,5556");
        //WIFIsrl.println("AT+NCUDP=192.168.1.1,5556");
        WIFIsrl.println("AT+NSTAT=?");
        WIFIsrl.println("AT+CID=?");
        WIFIsrl.print("ATA2\r");
        
        //WIFIsrl.print("AT*LED=1,2,1073741824,3\r");
        
        dronePrint();
      
}



void loop(){
        
        while(WIFIsrl.available()){
          Serial.write(WIFIsrl.read());
        } 
       
        
        //if(!sent){sent = true;dronePrint();}
    
        

}

void dronePrint(byte i){
   String droneBlink = "AT*LED=1,2,1073741824,3\r";
        
        WIFIsrl.write("AT*LED=1,2,1073741824,3\r\n");
}
  


