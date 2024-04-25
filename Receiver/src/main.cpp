#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

#define CS 4
#define G0 3

//Declaring radio and buffer vars
RH_RF95 driver(CS, G0);
uint8_t pkt[32];
uint8_t *length;
bool read;

//Declare functions
void imuRead();
void wheelRead();
void dataLogRead();
void imitateRadio();


void setup(){
  Serial.begin(9600); //Set Baud rate, don't set too high or risk data loss
  while (!Serial) ; // Wait for serial port to be available
  if (!driver.init())
    Serial.println("init failed"); 
  else
    Serial.println("init succeded");
  driver.setFrequency(915.0); // Median of Hz range
  driver.setTxPower(15, true); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
  driver.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
  driver.setModeRx();
}

void loop() {
  if(driver.available()){
    read = driver.recv(pkt, length);
    uint8_t ID = pkt[0];
    //If we were able to read packet choose decode function using ID
    if(read){
      switch(ID){
        case 1:
          imuRead();
          break;
        case 2:
          wheelRead();
          break;
        case 3:
          dataLogRead();
          break;
        case 5:
          Serial.println("Test data received");
        default:
          Serial.println("received empty packet");
          break;
      }
    } else {
      Serial.println("Unable to read packet");
    }
  } else {
    Serial.println("Sender not available :(");
  }
}

void imuRead(){
  //Vals printed to serial:
  // ID, Time (ms), x Acceleration (mG), y Acceleration (mG), z Acceleration (mG), x Gyro (mdps), y Gyro (mdps), z Gyro (mdps)
  unsigned long timestamp = (unsigned long) pkt[1] << 24 | (unsigned long) pkt[2] << 16 | (unsigned long) pkt[3] << 8 | (unsigned long) pkt[4];
  float xAccel = (pkt[5] << 24) | (pkt[6] << 16) | (pkt[7] << 8) | pkt[8]; 
  float yAccel = (pkt[9] << 24) | (pkt[10] << 16) | (pkt[11] << 8) | pkt[12];
  float zAccel = (pkt[13] << 24) | (pkt[14] << 16) | (pkt[15] << 8) | pkt[16];


  float xGyro = (pkt[17] << 24) | (pkt[18] << 16) | (pkt[19] << 8) | pkt[20];
  float yGyro = (pkt[21] << 24) | (pkt[22] << 16) | (pkt[23] << 8) | pkt[24];
  float zGyro = (pkt[25] << 24) | (pkt[26] << 16) | (pkt[27] << 8) | pkt[28];

  Serial.print("1" + String(timestamp) + ","  + 
    String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + "," + 
    String(xGyro)  + "," + String(yGyro)  + "," + String(zGyro)  + "\n");
}

void wheelRead() {
  //Vals printed to serial:
  // ID, Time (ms), FL speed(RPM), FL brake temp(C), FL ambient temp (C), FR speed(RPM), FR brake temp(C), FR ambient temp (C), 
  //  BL speed(RPM), BL brake temp(C), BL ambient temp (C), BR speed(RPM), BR brake temp(C), BR ambient temp (C)
  unsigned long timestamp = (unsigned long) pkt[1] << 24 | (unsigned long) pkt[2] << 16 | (unsigned long) pkt[3] << 8 | (unsigned long) pkt[4];
  float fl_speed = (pkt[5] << 8) | pkt[6];
  float fl_brakeTemp = (pkt[7] << 8) | pkt[8];
  float fl_ambTemp = (pkt[9] << 8) | pkt[10];

  float fr_speed = (pkt[11] << 8) | pkt[12];
  float fr_brakeTemp = (pkt[13] << 8) | pkt[14];
  float fr_ambTemp = (pkt[15] << 8) | pkt[16];

  float bl_speed = (pkt[17] << 8) | pkt[18];
  float bl_brakeTemp = (pkt[19] << 8) | pkt[20];
  float bl_ambTemp = (pkt[21] << 8) | pkt[22];

  float br_speed = (pkt[23] << 8) | pkt[24];
  float br_brakeTemp = (pkt[25] << 8) | pkt[26];
  float br_ambTemp = (pkt[27] << 8) | pkt[28];

  Serial.print("2" + String(timestamp) + ","  + 
    String(fl_speed) + "," + String(fl_brakeTemp) + "," + String(fl_ambTemp) + "," + 
    String(fr_speed) + "," + String(fr_brakeTemp) + "," + String(fr_ambTemp) + "," + 
    String(bl_speed) + "," + String(bl_brakeTemp) + "," + String(bl_ambTemp) + "," +
    String(br_speed) + "," + String(br_brakeTemp) + "," + String(br_ambTemp) + "\n");
}

void dataLogRead() {
  //Vals printed to serial:
  //ID, Time (ms), DRS (bool), steeringAngle(deg), throttle input (percent), Front brake pressure (BAR), Rear break pressure (BAR), 
  //  GPS Lattitude (DD), GPS Longitude (DD),battery voltage (mV), daq draw (mA)
  unsigned long timestamp = (unsigned long) pkt[1] << 24 | (unsigned long) pkt[2] << 16 | (unsigned long) pkt[3] << 8 | (unsigned long) pkt[4];
  uint8_t drsToggle = pkt[5];
  float steeringAngle = (pkt[6] << 8) | pkt[7];
  float throttleInput = (pkt[8] << 8) | pkt[9];

  float frontBrakePressure = (pkt[10] << 8) | pkt[11];
  float rearBrakePressure = (pkt[12] << 8) | pkt[13];

  float gpsLatitude = ((pkt[14] << 24) | (pkt[15] << 16) | (pkt[16] << 8) | pkt[17])/10000000;
  float gpsLongitude = ((pkt[18]<< 24) | (pkt[19] << 16) | (pkt[20] << 8) | pkt[21])/10000000;
  
  float batteryVoltage = (pkt[22] << 24) | (pkt[23] << 16) | (pkt[24] << 8) | pkt[25];
  float daqCurrentDraw = (pkt[26] << 24) | (pkt[27] << 16) | (pkt[28] << 8) | pkt[29];
  Serial.print("3" + String(timestamp) + ","  + String(drsToggle) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
    String(frontBrakePressure) + "," + String(rearBrakePressure) + "," + 
    String(gpsLatitude)        + "," + String(gpsLongitude)      + "," + 
    String(batteryVoltage)     + "," + String(daqCurrentDraw)    + "\n");
}

