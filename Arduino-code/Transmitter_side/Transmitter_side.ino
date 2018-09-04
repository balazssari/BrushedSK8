#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

byte addresses[][6] = {"1Node","2Node"};

RF24 radio(9, 10);

int ThrottleControlPin = 0;
int val = 0;
int alarm = 0;

void resetData() 
{
  val = 512;

}


void setup() {
  Serial.begin(9600);
  
  pinMode(3,OUTPUT);  //Buzzer
  
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(124);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  
  resetData();
}

void loop() {
  
  val = analogRead(ThrottleControlPin);
  
  //SENDING DATA
  radio.stopListening();

  if (!radio.write( &val, sizeof(val) )) {
    Serial.println("No acknowledgement of transmission - receiving radio device connected?");
    resetData();
  }

  //RECEIVING TELEMETRY DATA
  radio.startListening();
  unsigned long started_waiting_at = millis();
  while ( ! radio.available() ) {
    if (millis() - started_waiting_at > 100 ) {
      Serial.println("No telemetry received - timeout!");
      digitalWrite(3,LOW);
      alarm=0;
      return;
    }
  }
  if ( radio.available() == 1 ){
  radio.read( &alarm, sizeof(alarm) );
  if ( alarm ==  1 ) tone( 3, 1000);

  }
}
