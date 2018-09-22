#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);

byte addresses[][6] = {"1Node", "2Node"};

struct Ack {
  float voltage;
  float current;
};

Ack ack;

struct MyData {
  int val;
};

MyData data;
void resetData() 
{
  // 'safe' values to use when no radio input is detected
  ack.voltage = 0;
}
void setup() {
  pinMode(3,OUTPUT);
  Serial.begin(9600);
  radio.begin();

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(124);
  radio.enableAckPayload();
  radio.openWritingPipe(addresses[1]);
  radio.stopListening();
}

unsigned long lastRecvTime = 0;

void loop() {
  
  int cal = 0;
  for(int i=0;i<=4;i++){
  cal += analogRead(0);
  }
  data.val = cal/5;
  
  if(radio.write(&data, sizeof(MyData))){
    if(radio.isAckPayloadAvailable()){
      radio.read(&ack,sizeof(Ack));
      lastRecvTime = millis();
      }
    }
    unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }

  Serial.println(ack.voltage);

}
