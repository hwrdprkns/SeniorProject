
void setup() {
  Serial1.begin(57600); 
  Serial.begin(57600);  
}

void loop() {
  if (Serial1.available()) {
    Serial.write(Serial1.read());
   }
}

