#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

#define CS0 4
#define G00 0

#define CS1 5
#define G01 1

#define CS2 6
#define G02 2

#define CS3 7
#define G03 3

#define testing false

int mode = 2; // 0 for general, 1 for suspension, 2 for damper, 3 for driver, 4 for slip/slide


bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;

//Declaring radio and buffer vars
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);

uint8_t pkt[251];
uint8_t length = sizeof(pkt);

bool read1;
bool read2;
bool read3;
bool read4;

//Declare functions
void testRead();
void packetRead();

//interval for checking if sender is available in milliseconds
const unsigned long INTERVAL1 = 500;

//interval for waiting for new packet before searching for connection
const unsigned long INTERVAL2 = 5000;

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup(){
  Serial.begin(BAUD);

  while(!Serial);

  radio1 = driver1.init();
  if (!radio1)
    Serial.println("init 1 failed"); 
  else{
    Serial.println("init 1 succeded");
    driver1.setFrequency(915.0); // Median of Hz range
    driver1.setTxPower(23, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
    driver1.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
    driver1.setModeRx();
  }
  
  radio2 = driver2.init();
  if (!radio2)
    Serial.println("init 2 failed"); 
  else{
    Serial.println("init 2 succeded");
    driver2.setFrequency(915.0);
    driver2.setTxPower(RH_RF95_MAX_POWER, false);
    driver2.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);
  }

  radio3 = driver3.init();
  if (!radio3)
    Serial.println("init 3 failed"); 
  else{
    Serial.println("init 3 succeded");
    driver3.setFrequency(915.0);
    driver3.setTxPower(RH_RF95_MAX_POWER, false);
    driver3.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);
  }

  radio4 = driver4.init();
  if (!radio4)
    Serial.println("init 4 failed"); 
  else{
    Serial.println("init 4 succeded");
    driver4.setFrequency(915.0);
    driver4.setTxPower(RH_RF95_MAX_POWER, false);
    driver4.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);
  }  
}

void loop() {
  read1 = false;
  read2 = false;
  read3 = false;
  read4 = false;
  
  if(driver1.available() && radio1){
    read1 = driver1.recv(pkt, &length);
    if (testing){
      Serial.print("Driver1: ");
      testRead();
    } else {
      packetRead();
    }
    driver1.waitAvailableTimeout(INTERVAL2);
  } else if(driver2.available() && radio2){
    read2 = driver2.recv(pkt, &length);
    if (testing){
      Serial.print("Driver2: ");
      testRead();
    } else {
      packetRead();
    }
    driver2.waitAvailableTimeout(INTERVAL2);
  } else if(driver3.available() && radio3){
    read3 = driver3.recv(pkt, &length);
    if (testing){
      Serial.print("Driver3: ");
      testRead();
    } else {
      packetRead();
    }
    driver3.waitAvailableTimeout(INTERVAL2);
  } else if(driver4.available() && radio4){
    read4 = driver4.recv(pkt, &length);
    if (testing){
      Serial.print("Driver4: ");
      testRead();
    } else {
      packetRead();
    }
    driver4.waitAvailableTimeout(INTERVAL2);
  }
  if (testing){
    Serial.println("radio 1 init: " + String(radio1) + "\tradio 1 recive: " + String(read1));
    Serial.println("radio 2 init: " + String(radio2) + "\tradio 2 recive: " + String(read2));
    Serial.println("radio 3 init: " + String(radio3) + "\tradio 3 recive: " + String(read3));
    Serial.println("radio 4 init: " + String(radio4) + "\tradio 4 recive: " + String(read4));
  }
  //Serial.println("Reciver 3 not available :(");
}


void packetRead() {
  //Serial.println("Packet read");
  // IMU pkt 4-28, 
  // Wheel Boards 29-6
  
  switch (mode) {
    case 0: {// General
      unsigned long timestamp = (unsigned long) pkt[0] << 24 | (unsigned long) pkt[1] << 16 | (unsigned long) pkt[2] << 8 | (unsigned long) pkt[3];
      float xAccel = (pkt[4] << 24) | (pkt[5] << 16) | (pkt[6] << 8) | pkt[7]; 
      float yAccel = (pkt[8] << 24) | (pkt[9] << 16) | (pkt[10] << 8) | pkt[11];
      float zAccel = (pkt[12] << 24) | (pkt[13] << 16) | (pkt[14] << 8) | pkt[15];

      float xGyro = (pkt[16] << 24) | (pkt[17] << 16) | (pkt[18] << 8) | pkt[19];
      float yGyro = (pkt[20] << 24) | (pkt[21] << 16) | (pkt[22] << 8) | pkt[23];
      float zGyro = (pkt[24] << 24) | (pkt[25] << 16) | (pkt[26] << 8) | pkt[27];

      float fl_speed = (pkt[28] << 8) | pkt[29];
      float fl_brakeTemp = (pkt[30] << 8) | pkt[31];
      float fl_ambTemp = (pkt[32] << 8) | pkt[33];

      float fr_speed = (pkt[34] << 8) | pkt[35];
      float fr_brakeTemp = (pkt[36] << 8) | pkt[37];
      float fr_ambTemp = (pkt[38] << 8) | pkt[39];

      float bl_speed = (pkt[40] << 8) | pkt[41];
      float bl_brakeTemp = (pkt[42] << 8) | pkt[43];
      float bl_ambTemp = (pkt[44] << 8) | pkt[45];

      float br_speed = (pkt[46] << 8) | pkt[47];
      float br_brakeTemp = (pkt[48] << 8) | pkt[49];
      float br_ambTemp = (pkt[50] << 8) | pkt[51];

      float differentialSpeed = (pkt[52] << 8 | pkt[53]);

      uint8_t drsToggle = pkt[54];
      float steeringAngle = (pkt[55] << 8) | pkt[56];
      float throttleInput = (pkt[57] << 8) | pkt[58];

      float frontBrakePressure = (pkt[59] << 8) | pkt[60];
      float rearBrakePressure = (pkt[61] << 8) | pkt[62];

      float gpsLatitude = ((pkt[63] << 24) | (pkt[64] << 16) | (pkt[65] << 8) | pkt[66])/10000000;
      float gpsLongitude = ((pkt[67]<< 24) | (pkt[68] << 16) | (pkt[69] << 8) | pkt[70])/10000000;
      
      float batteryVoltage = (pkt[71] << 24) | (pkt[72] << 16) | (pkt[73] << 8) | pkt[74];
      float daqCurrentDraw = (pkt[75] << 24) | (pkt[76] << 16) | (pkt[77] << 8) | pkt[78];

      float fl_shock = (pkt[79] << 8) | pkt[80];
      float fr_shock = (pkt[81] << 8) | pkt[82];
      float bl_shock = (pkt[83] << 8) | pkt[84];
      float br_shock = (pkt[85] << 8) | pkt[86];
      
      if (testing) {
        Serial.println("[General]");
      }

      Serial.print("0," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + "," + 
        String(xGyro)  + "," + String(yGyro)  + "," + String(zGyro)); 
      Serial.print(String(fl_speed) + "," + String(fl_brakeTemp) + "," + String(fl_ambTemp) + "," + 
        String(fr_speed) + "," + String(fr_brakeTemp) + "," + String(fr_ambTemp) + "," + String(bl_speed) + "," + 
        String(bl_brakeTemp) + "," + String(bl_ambTemp) + "," + String(br_speed) + "," + String(br_brakeTemp) + "," + 
        String(br_ambTemp) + "," + String(differentialSpeed) + ",");
      Serial.print(String(drsToggle) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
        String(frontBrakePressure) + "," + String(rearBrakePressure) + "," + 
        String(gpsLatitude)        + "," + String(gpsLongitude)      + "," + 
        String(batteryVoltage)     + "," + String(daqCurrentDraw) + "," + String(fl_shock) + "," + String(fr_shock) + "," + String(bl_shock) + "," + String(br_shock) + "\n");
      break;
    }
    case 1: {// Suspension
      // IMU DATA
      for (int offset = 0; offset < 64; offset+=63) {
        unsigned long timestamp = (unsigned long) pkt[0 + offset] << 24 | (unsigned long) pkt[1 + offset] << 16 | (unsigned long) pkt[2 + offset] << 8 | (unsigned long) pkt[3 + offset];
        float xAccel = (pkt[4 + offset] << 24) | (pkt[5 + offset] << 16) | (pkt[6 + offset] << 8) | pkt[7 + offset]; 
        float yAccel = (pkt[8 + offset] << 24) | (pkt[9 + offset] << 16) | (pkt[10 + offset] << 8) | pkt[11 + offset];
        float zAccel = (pkt[12 + offset] << 24) | (pkt[13 + offset] << 16) | (pkt[14 + offset] << 8) | pkt[15 + offset];

        float xGyro = (pkt[16 + offset] << 24) | (pkt[17 + offset] << 16) | (pkt[18 + offset] << 8) | pkt[19 + offset];
        float yGyro = (pkt[20 + offset] << 24) | (pkt[21 + offset] << 16) | (pkt[22 + offset] << 8) | pkt[23 + offset];
        float zGyro = (pkt[24 + offset] << 24) | (pkt[25 + offset] << 16) | (pkt[26 + offset] << 8) | pkt[27 + offset];

        // WHEEL BOARD DATA
        float fl_speed = (pkt[28 + offset] << 8) | pkt[29 + offset];
        float fr_speed = (pkt[30 + offset] << 8) | pkt[31 + offset];
        float bl_speed = (pkt[32 + offset] << 8) | pkt[33 + offset];
        float br_speed = (pkt[34 + offset] << 8) | pkt[35 + offset];

        float differentialSpeed = (pkt[36 + offset] << 8 | pkt[37 + offset]);

        uint8_t drsToggle = pkt[38 + offset];
        float steeringAngle = (pkt[39 + offset] << 8) | pkt[40 + offset];
        float throttleInput = (pkt[41 + offset] << 8) | pkt[42 + offset];

        float frontBrakePressure =(pkt[43 + offset] << 8) | pkt[44 + offset];
        float rearBrakePressure = (pkt[45 + offset] << 8) | pkt[46 + offset];

        float gpsLatitude = ((pkt[47 + offset] << 24) | (pkt[48 + offset] << 16) | (pkt[49 + offset] << 8) | pkt[50 + offset])/10000000;
        float gpsLongitude =((pkt[51 + offset] << 24) | (pkt[52 + offset] << 16) | (pkt[53 + offset] << 8) | pkt[54 + offset])/10000000;

        float fl_shock = (pkt[55 + offset] << 8) | pkt[56 + offset];
        float fr_shock = (pkt[57 + offset] << 8) | pkt[58 + offset];
        float bl_shock = (pkt[59 + offset] << 8) | pkt[60 + offset];
        float br_shock = (pkt[61 + offset] << 8) | pkt[62 + offset];

        if (testing) {
          Serial.println("[Suspension]");
        }

        Serial.print("1," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + "," + 
          String(xGyro)  + "," + String(yGyro)  + "," + String(zGyro)); 
        Serial.print(String(fl_speed) + "," + String(fr_speed) + "," + String(bl_speed) + "," + String(br_speed) + "," + String(differentialSpeed) +  ",");
        Serial.print(String(drsToggle) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
          String(frontBrakePressure) + "," + String(rearBrakePressure) + "," + 
          String(gpsLatitude) + "," + String(gpsLongitude) + "," + String(fl_shock) + "," + String(fr_shock) + "," + String(bl_shock) + "," + String(br_shock) + "\n");
      }
      break;
    }
    case 2: {// Damper
      for (int offset = 0; offset < 209; offset+=26) {
        //Serial.println(offset);
        unsigned long timestamp = (unsigned long) pkt[0 + offset] << 24 | (unsigned long) pkt[1 + offset] << 16 | (unsigned long) pkt[2 + offset] << 8 | (unsigned long) pkt[3 + offset];
        float xAccel = (pkt[4 + offset] << 24) | (pkt[5 + offset] << 16) | (pkt[6 + offset] << 8) | pkt[7 + offset]; 
        float yAccel = (pkt[8 + offset] << 24) | (pkt[9 + offset] << 16) | (pkt[10 + offset] << 8) | pkt[11 + offset];
        float zAccel = (pkt[12 + offset] << 24) | (pkt[13 + offset] << 16) | (pkt[14 + offset] << 8) | pkt[15 + offset];

        float fl_speed = (pkt[16 + offset] << 8) | pkt[17 + offset];

        float fl_shock = (pkt[18 + offset] << 8) | pkt[19 + offset];
        float fr_shock = (pkt[20 + offset] << 8) | pkt[21 + offset];
        float bl_shock = (pkt[22 + offset] << 8) | pkt[23 + offset];
        float br_shock = (pkt[24 + offset] << 8) | pkt[25 + offset];

        if (testing) {
          Serial.println("[Damper]");
        }

        Serial.print("2," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + ","); 
        Serial.print(String(fl_speed) + "," + String(fl_shock) + "," + String(fr_shock) + "," + String(bl_shock) + "," + String(br_shock) + "\n");
      }
      break;
    }
    case 3: {// Driver
      for (int offset = 0; offset < 74; offset+=37) {
        //Serial.println("DRIVER");
        unsigned long timestamp = (unsigned long) pkt[0 + offset] << 24 | (unsigned long) pkt[1 + offset] << 16 | (unsigned long) pkt[2 + offset] << 8 | (unsigned long) pkt[3 + offset];
        float xAccel = (pkt[4 + offset] << 24) | (pkt[5 + offset] << 16) | (pkt[6 + offset] << 8) | pkt[7 + offset]; 
        float yAccel = (pkt[8 + offset] << 24) | (pkt[9 + offset] << 16) | (pkt[10 + offset] << 8) | pkt[11 + offset];

        float fl_speed = (pkt[12 + offset] << 8) | pkt[13 + offset];
        float bl_speed = (pkt[14 + offset] << 8) | pkt[15 + offset];
        float br_speed = (pkt[16 + offset] << 8) | pkt[17 + offset];

        float differentialSpeed = (pkt[18 + offset] << 8 | pkt[19 + offset]);

        uint8_t drsToggle = pkt[20 + offset];
        float steeringAngle = (pkt[21 + offset] << 8) | pkt[22 + offset];
        float throttleInput = (pkt[23 + offset] << 8) | pkt[24 + offset];

        float frontBrakePressure = (pkt[25 + offset] << 8) | pkt[26 + offset];
        float rearBrakePressure = (pkt[27 + offset] << 8) | pkt[28 + offset];

        float fl_shock = (pkt[29 + offset] << 8) | pkt[30 + offset];
        float fr_shock = (pkt[31 + offset] << 8) | pkt[32 + offset];
        float bl_shock = (pkt[33 + offset] << 8) | pkt[34 + offset];
        float br_shock = (pkt[35 + offset] << 8) | pkt[36 + offset];

        if (testing) {
          Serial.println("[Driver]");
        }

        Serial.print("3," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(fl_speed) + "," + String(bl_speed) + "," + String(br_speed) + ",");
        Serial.print(String(differentialSpeed) + "," + (drsToggle) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
          String(frontBrakePressure) + "," + String(rearBrakePressure) + "," + String(fl_shock) + "," + String(fr_shock) + "," + String(bl_shock) + "," + String(br_shock) + "\n");
      }
      break;
    }
    case 4: {// Slip/Slide
      for (int offset = 0; offset < 151; offset+=30) {
        unsigned long timestamp = (unsigned long) pkt[0 + offset] << 24 | (unsigned long) pkt[1 + offset] << 16 | (unsigned long) pkt[2 + offset] << 8 | (unsigned long) pkt[3 + offset];
        float xAccel = (pkt[4 + offset] << 24) | (pkt[5 + offset] << 16) | (pkt[6 + offset] << 8) | pkt[7 + offset]; 
        float yAccel = (pkt[8 + offset] << 24) | (pkt[9 + offset] << 16) | (pkt[10 + offset] << 8) | pkt[11 + offset];

        float fl_speed = (pkt[12 + offset] << 8) | pkt[13 + offset];
        float fr_speed = (pkt[14 + offset] << 8) | pkt[15 + offset];
        float bl_speed = (pkt[16 + offset] << 8) | pkt[17 + offset];
        float br_speed = (pkt[18 + offset] << 8) | pkt[19 + offset];

        float differentialSpeed = (pkt[20 + offset] << 8 | pkt[21 + offset]);

        float steeringAngle = (pkt[22 + offset] << 8) | pkt[23 + offset];
        float throttleInput = (pkt[24 + offset] << 8) | pkt[25 + offset];

        float frontBrakePressure = (pkt[26 + offset] << 8) | pkt[27 + offset];
        float rearBrakePressure = (pkt[28 + offset] << 8) | pkt[29 + offset];

        if (testing) {
          Serial.println("[SLIP/SLIDE]");
        }

        Serial.print("4," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(fl_speed) + "," + String(fr_speed) + "," + String(bl_speed) + "," + 
          String(br_speed) + "," + String(differentialSpeed) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
          String(frontBrakePressure) + "," + String(rearBrakePressure) + "\n");

      }
      break;
    }
    default: {
      Serial.println("Nothing Called");
      break;
  }
  }
}
void testRead(){
  for (int i = 0; i < sizeof(pkt); i++){
    Serial.print(String(pkt[i]) + ", ");
  }
  Serial.println();
}