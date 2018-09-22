#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>
#include<avr/wdt.h>
//USER SETTINGS
//Control Potentiometer Calibration:
int low_threshold  = 300;
int high_threshold = 700;
int middle_point   = 512;
bool invert_controls = 1;

//Throttle "curve"
int section1 = 0;         int rate1 = 20;
int section2 = 300;       int rate2 = 5;
int section3 = 500;       int rate3 = 2;
int section4 = 700;       int rate4 = 2;
int section5 = 900;       int rate5 = 1;
int section6 = 1023;

float start_current_limit = 20.0;           //current limit for the time when the motor is starting up
float ride_current_limit  = 8.0;            //current limit for the time when the motor is already running

int t = 10;                                 // loop-time in ms, refresh rate = 1000/t, this parameter affects throttle curve settings
//USER SETTINGS END HERE//
//////////////////////////////////////////////

float current_limit = 8.0;

int rate=3;

//NRF24L01 stuff
RF24 radio(9, 10);

byte addresses[][6] = {"1Node","2Node"};

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
  // 'safe' data.values to use when no radio input is detected
  data.val = 512;
}

unsigned long lastRecvTime = 0;

// Variables for keeping the cycle-time
int time_start;
int time_end;
int ms_delay;

//PWM outputs
int drive_pin = 3;        //PWM output for drive FET
int brake_pin = 5;        //PWM output for active-brake FET

//Current and Voltage sensor inputs
int current_sens_pin = 0;
int voltage_sens_pin = 1;

float current = 0;
float voltage = 0;

int throttle = 0;       //Variable for the calculated throttle

int throttle_input=0;
int brake_input=0;
/*******************FUNCTIONS**********************/
void inputvalues(){
  if ( data.val > middle_point ){
    brake_input = 0;
    throttle_input = ((data.val-high_threshold)*(1023.0/(1023.0-high_threshold)));
    if( data.val < high_threshold ) throttle_input = 0;
  }
  if ( data.val < middle_point ){
    throttle_input = 0;
    brake_input = abs((data.val-low_threshold))*(1023.0/low_threshold);
    if( data.val > low_threshold ) brake_input = 0;
  }
  if ( invert_controls == 1 ){
    int storage = throttle_input;
    throttle_input = brake_input;
    brake_input = storage;
  }
  }
  
void calcCurrentVoltage(){
  //INA169 : RL = 50 kOhm, Imax = 100 A, Rs = 0,001 Ohm
  //Formula: Vo = ( Imax * Rs * RL ) / 10kOhm
  //Formula: 5V = ( 80A * 0.001 Ohm * 50 kOhm ) / 10 kOhm
  //10 bit AD converter: 1023/80A = 12.79
  current = 0;
  for (int i = 0; i < 10; i++) current += analogRead(current_sens_pin) / 12.79;
  current = current / 10.0;
  //33kOhm, 8.2kOhm
  voltage = analogRead(voltage_sens_pin) / 40.595;
  }
void throttleCurveCurrent(){
  if ( section1 < throttle && throttle > section3) current_limit = start_current_limit;
  else current_limit = ride_current_limit;
  
  // Dealing with the throttle curve:
  if ( throttle == 0) rate = rate1;
  if ( section1 < throttle && throttle > section2) rate = rate1;
  if ( section2 < throttle && throttle > section3) rate = rate2;
  if ( section3 < throttle && throttle > section4) rate = rate3;
  if ( section4 < throttle && throttle > section5) rate = rate4;
  if ( section5 < throttle && throttle > section6) rate = rate5;
  
  if( throttle_input > 0 && brake_input == 0){
    if( current > current_limit ){
      if( throttle > 3 ) throttle -= rate;
      }
      else{
        if( current < current_limit-0.2 && throttle < throttle_input ) throttle += rate;
        else if( throttle > throttle_input ) throttle -= rate; 
        }
    }
  else{
    throttle = 0;
    }
  if ( throttle > 1024 ) throttle = 1024; 
  }
  
/**************************************************/

void setup() {
  wdt_enable (WDTO_60MS);                   //The watchdog timer will restart the MC if something's go wrong

  Serial.begin(9600);
 
  radio.begin();

  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(124);
  radio.enableAckPayload();
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
  resetData();
 

}

void loop() {
  time_start=millis();                        //Start a timer for calculating busy loop-time


  if(radio.available()){
    radio.read(&data,sizeof(MyData));
    radio.writeAckPayload(1, &ack, sizeof(Ack));
    lastRecvTime = millis();
    }
    
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }

  inputvalues();                              //converting the input from radio to basic throttle-brake data.values
  
  calcCurrentVoltage();                       //reading and processing data from AD
  ack.voltage = voltage;
  ack.current = current;
  throttleCurveCurrent();                     //applying the curve for throttle and limit the current
  
  //DEBUG HERE:
  Serial.println(throttle);

  
  // DO NOT CHANGE THIS PART OF CODE
  // ANTI SELF-DESTRUCTION SECTION START
  if(throttle > 0 && brake_input > 0){
    throttle = 0;
    brake_input = 0;
    }
  if(throttle == 0) analogWrite( drive_pin, 0 );
  if(brake_input == 0 && throttle > 0){
    analogWrite( drive_pin, ( throttle/4 ) );
    }
  if(brake_input == 0) analogWrite( brake_pin, 0 );
  if(throttle == 0 && brake_input > 0){
    analogWrite( brake_pin, ( brake_input/20 ) ); // 20% duty cycle max for active braking
    }
  time_end = millis();                            // Calculate busy loop-time
  ms_delay = t - (time_end - time_start);         // Delay required for maintain the pre-set loop-time
  time_start = 0;
  time_end = 0;
  delay(ms_delay);
  wdt_reset();                                    // Reset if something stuck in
  // ANTI SELF-DESTRUCTION SECTION END
}
