/*
Inition Nemeziz Boot SensorCode

Uses Adafruit BNO055 9DOF sensor
  Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
4.5" bend sensor from adafruit
SingleTact 1N 8mm pressure sensors x 5
Fabricated capacitive stroke sensors using triangles of copper tape on acetate
HC-SR04 distance sensor for proximity trigger
Data sent as comma separated string to serial in this format:
A, x, y, z, pressure1, pressure2, pressure3, pressure4, pressure5, stroke1, stroke2, stroke3, bend, distance
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <CapacitiveSensor.h>
//Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)
#include <NewPing.h>
//pins for distance sensor
#define TRIGGER_PIN  13  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Adafruit_BNO055 bno = Adafruit_BNO055(55);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance

CapacitiveSensor   cs_11_10 = CapacitiveSensor(11, 10); //for stroke sensor1
CapacitiveSensor   cs_9_8 = CapacitiveSensor(9, 8); //for stroke sensor2
CapacitiveSensor   cs_7_6 = CapacitiveSensor(7, 6); //for stroke sensor2

int bendPin = 0;    // select the input pin for the potentiometer
float bendValue = 0;  // variable to store the value coming from the sensor
int bendMin = 115; //was 160
int bendMax = 107; //was 130

//define i2c addresses for 5 singletact sensors (set thro singletact PC application)
byte i2cAddress1 = 0x05;
byte i2cAddress2 = 0x06;
byte i2cAddress3 = 0x07;
byte i2cAddress4 = 0x08;
byte i2cAddress5 = 0x09;
//byte i2cAddress6 = 0x10;

long distance;  //for distance measure

float qw, qx, qy, qz, x, y, z, xcal;

short data1, data2, data3, data4, data5, data6;  //for singletact pressure sensors

long lastTime;  //to record time to charge stroke sensor capacitors
long start, total1, total2, total3; //for capacitive values from stroke sensors

boolean sendData = false;  //send data when this flag is true

void setup(void)
{
  Serial.begin(9600);
  Serial.flush();
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  lastTime = millis();
}

void loop(void)
{
  distance = sonar.ping_cm();
  //Serial.println(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  
  //start counter for measuring capacitance
  start = millis();
  total1 =  cs_11_10.capacitiveSensor(30);  //get value for stroke 1
  total2 =  cs_9_8.capacitiveSensor(30);  //get value for stroke2
  total3 =  cs_7_6.capacitiveSensor(30);  //get value for stroke3

  //bend sensor
  bendValue = analogRead(bendPin);
  //Serial.println(bendValue);
 bendValue = map(bendValue, bendMin, bendMax, 0, 90);
  getPressure();


  /* Get a new position sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Quaternion quat = bno.getQuat();
  //check calibration
     uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  if(sys>0){
    sendData=true;
  }

  //assemble string and send over serial
  if (sendData == true) {
    /*
    Serial.print(sys, DEC);
    Serial.print(",");
  Serial.print(gyro, DEC);
  Serial.print(",");
  Serial.print(accel, DEC);
  Serial.print(",");
  Serial.print(mag, DEC);
  Serial.print(",");
  */
    Serial.print('A');  //send a character to identify start of string
    Serial.print(",");

     //send values from position sensor
    Serial.print(quat.x(), 4);
    Serial.print(",");
    Serial.print(quat.y(), 4);
    Serial.print(",");
    Serial.print(quat.z(), 4);
    Serial.print(",");
    Serial.print(quat.w(), 4);

    Serial.print(",");
    //send values from pressure sensors
    Serial.print(data1);
    Serial.print(",");
    Serial.print(data2);
    Serial.print(",");
    Serial.print(data3);
    Serial.print(",");
    Serial.print(data4);
    Serial.print(",");
    Serial.print(data5);
    Serial.print(",");
    //send stroke sensor values
    Serial.print(total1);      //stroke sensor1
    Serial.print(",");
    Serial.print(total2);      //stroke sensor2
    Serial.print(",");
    Serial.print(total3);      //stroke sensor3
    Serial.print(",");
    Serial.print(bendValue);
    Serial.print(",");
    Serial.print(distance); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println();
    //Serial.print("\n");

  }

  /* Also send calibration data for each sensor. */
  // uint8_t sys, gyro, accel, mag = 0;
  //bno.getCalibration(&sys, &gyro, &accel, &mag);
   //Serial.println(mag, DEC);
  /*
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
  */
  delay(BNO055_SAMPLERATE_DELAY_MS);

}


void getPressure() {
  data1 = readDataFromSensor(i2cAddress1);
  data2 = readDataFromSensor(i2cAddress2);
  data3 = readDataFromSensor(i2cAddress3);
  data4 = readDataFromSensor(i2cAddress4);
  data5 = readDataFromSensor(i2cAddress5);
  // data6 = readDataFromSensor(i2cAddress6);
}

short readDataFromSensor(short address)
{
  byte i2cPacketLength = 6;//i2c packet length. Just need 6 bytes from each slave
  byte outgoingI2CBuffer[3];//outgoing array buffer
  byte incomingI2CBuffer[6];//incoming array buffer

  outgoingI2CBuffer[0] = 0x01;//I2c read command
  outgoingI2CBuffer[1] = 128;//Slave data offset
  outgoingI2CBuffer[2] = i2cPacketLength;//require 6 bytes

  Wire.beginTransmission(address); // transmit to device
  Wire.write(outgoingI2CBuffer, 3);// send out command
  byte error = Wire.endTransmission(); // stop transmitting and check slave status
  if (error != 0) return -1; //if slave not exists or has error, return -1
  Wire.requestFrom(address, i2cPacketLength);//require 6 bytes from slave

  byte incomeCount = 0;
  while (incomeCount < i2cPacketLength)    // slave may send less than requested
  {
    if (Wire.available())
    {
      incomingI2CBuffer[incomeCount] = Wire.read(); // receive a byte as character
      incomeCount++;
    }
    else
    {
      delayMicroseconds(10); //Wait 10us
    }
  }

  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5]; //get the raw data

  return rawData;
}

