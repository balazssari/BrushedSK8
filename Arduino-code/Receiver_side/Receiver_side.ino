#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
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

//NRF24L01 stuff
byte addresses[][6] = {"1Node","2Node"};
RF24 radio(9, 10);
int val = 0;
int alarm = 0;

float current_limit = 8.0;

int rate=3;

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
void inputValues(){
  if ( val > middle_point ){
    brake_input = 0;
    throttle_input = ((val-high_threshold)*(1023.0/(1023.0-high_threshold)));
    if( val < high_threshold ) throttle_input = 0;
  }
  if ( val < middle_point ){
    throttle_input = 0;
    brake_input = abs((val-low_threshold))*(1023.0/low_threshold);
    if( val > low_threshold ) brake_input = 0;
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

  void resetData(){
  val = 512;
  }
/**************************************************/



void setup(){ 
  wdt_enable (WDTO_60MS);                   //The watchdog timer will restart the MC if something's go wrong
  
  Serial.begin(19200);
  
  pinMode(drive_pin,OUTPUT);
  pinMode(brake_pin,OUTPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(124);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  
  // Start the radio listening for data
  radio.startListening();
  resetData();
}

void loop(){
  time_start=millis();                        //Start a timer for calculating busy loop-time
  
if ( radio.available()) {

                                              //Go and read the data and put it into that variable
    while (radio.available()) {
      radio.read( &val, sizeof(val));
      lastRecvTime = millis();
    }

    radio.stopListening();                    //Stop listening so we can send some telemetry data out to controller
    radio.write( &alarm, sizeof(alarm) );
  
    radio.startListening();                   //Start listening again
    }
    
  unsigned long now = millis();
  if ( now - lastRecvTime > 500 ) {
    // signal lost?

      resetData();
      
  }
  
  inputValues();                              //converting the input from radio to basic throttle-brake values
  
  calcCurrentVoltage();                       //reading and processing data from AD

  throttleCurveCurrent();                     //applying the curve for throttle and limit the current
 
  
  if ( voltage < 22.0){
    alarm = 1;
    //transmit alarm something to the controller
    }
  
  //DEBUG HERE:
  /*
  Serial.print(throttle);
  Serial.print("    ");
  Serial.println(current);
  */
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
  
/**************************************************/
