#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
void setup() {
  mcp1.begin(0);
  mcp2.begin(2);-

  for(int i=1;i<15;i++) {
    mcp1.pinMode(i, INPUT);
    mcp2.pinMode(i, INPUT);
  }
  
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  delay(2000);
  Serial.println("Started...");
}

void loop() {
  for(int i=1;i<15;i++) {
    if(mcp1.digitalRead(i) == HIGH) {
      Serial.print("mcp1 ");
      Serial.println(i);
    }
    if(mcp2.digitalRead(i) == LOW) {
      Serial.print("mcp2 ");
      Serial.println(i);
    }
  }

}
