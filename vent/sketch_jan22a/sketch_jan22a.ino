void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly: 
//  Serial.println(Serial.read());
  
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    if (inChar == '@')
      Serial.println("get LF");
    else
      Serial.println(inChar);
  }
}
