#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>

#define sensor_top_address  0x29
#define sensor_bot_address  0x70

#define sensor_top_sda  D6
#define sensor_top_scl  D7

#define sensor_bot_sda  D2
#define sensor_bot_scl  D1

#define threshold_trigger_sensor_top  100
#define threshold_trigger_sensor_bot  100

#define light_step_dimming  80
#define light_step_dimming_delay 2
#define delay_between_stair_step  250

#define timer_switch_off  10000 

VL53L0X sensorTop;
VL53L0X sensorBot;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void scani2c();
void lightUpSimple();
void lightDownSimple();

bool direction=false;
bool lightState=false;

long turnOffTimer=millis();

void setup() {

delay(3000);
Serial.begin(9600);

// dummy set wire pinning to assign I2c address
Wire.begin(D3,D4);
sensorTop.setAddress(sensor_top_address);
sensorBot.setAddress(sensor_bot_address);

// scan both I2C bus for 5 times just to check
  for (int i=0;i<5;i++){
    Wire.begin(sensor_top_sda,sensor_top_scl);
    //Wire.setClock(100000); 
    Serial.println("SCAN BUS 1 TOP SENSOR");
    scani2c();

    Wire.begin(sensor_bot_sda,sensor_bot_scl);
    //Wire.setClock(100000); 
    Serial.println("SCAN BUS 2 BOT SENSOR");
    scani2c();
  }

// initialize top sensor
Wire.begin(sensor_top_sda,sensor_top_scl);
sensorTop.setTimeout(500);
if (!sensorTop.init()){
    Serial.println("Failed init SENSOR Top!");
    while (1);
}
sensorTop.setMeasurementTimingBudget(50000);
sensorTop.startContinuous(50);

// initialize bottom sensor
Wire.begin(sensor_bot_sda,sensor_bot_scl);
sensorBot.setTimeout(500);
if (!sensorBot.init()){
    Serial.println("Failed init SENSOR Bot!");
    while (1);
}
sensorBot.setMeasurementTimingBudget(50000);
sensorBot.startContinuous(50);

// Initialize PWM drivers
Wire.begin(sensor_bot_sda,sensor_bot_scl);
pwm.begin();
pwm.setOscillatorFrequency(27000000);
pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

}

void loop() {

  Wire.begin(sensor_top_sda,sensor_top_scl);
  int valueTop=sensorTop.readRangeSingleMillimeters();
  Serial.print("Sensor Top: ");
  Serial.print(valueTop);
  Serial.print('\t');

  if(valueTop<threshold_trigger_sensor_top && !lightState){
    Serial.println("Triggered Sensor Top!");
    direction=false;
    //lightUpSimple();
    //lightState=true;
    turnOffTimer=millis();
  }

  delay(25);

  Wire.begin(sensor_bot_sda,sensor_bot_scl);
  int valueBot=sensorBot.readRangeSingleMillimeters();
  Serial.print("Sensor Bot: ");
  Serial.println(valueBot);

  if(valueBot<threshold_trigger_sensor_bot && !lightState){
    Serial.println("Triggered Sensor Top!");
    direction=true;
    //lightUpSimple();
    //lightState=true;
    turnOffTimer=millis();
  }

  delay(25);

  if(millis()-turnOffTimer>timer_switch_off && lightState){
    Serial.println("Triggered light OFF");
    lightDownSimple();
    lightState=false;
  } 

}

void lightUpSimple(){
  if(direction){
    for(int i=0;i<14;i++){
      for(int x=0;x<4000;x=x+light_step_dimming){
        pwm.setPWM(i, 0, x);
        delay(light_step_dimming_delay); 
      }
      delay(delay_between_stair_step);
    }
  }else{
    for(int i=0;i<14;i++){
      Serial.println(i);
      for(int x=0;x<4000;x=x+light_step_dimming){
        pwm.setPWM(13-i, 0, x);
        delay(light_step_dimming_delay);
      }
      delay(delay_between_stair_step);
    }
  }
}

void lightDownSimple(){
  if(direction){
    for(int i=0;i<14;i++){
      for(int x=0;x<4000;x+=30){
        pwm.setPWM(i, 0, 4000-x);
        delay(2); 
      }
      delay(delay_between_stair_step);
    }
  }else{
    for(int i=0;i<14;i++){
      for(int x=0;x<4000;x=x+30){
        pwm.setPWM(13-i, 0, 4000-x);
        delay(2); 
      }
      delay(delay_between_stair_step);
    }
  }
}

void scani2c(){
  
    byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(100);           // wait 5 seconds for next scan
  
}