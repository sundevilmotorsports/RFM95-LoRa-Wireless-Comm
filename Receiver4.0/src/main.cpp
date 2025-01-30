#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TimeLib.h>

using namespace std;

#define CS0 10
#define G00 21

#define CS1 9
#define G01 20

#define CS2 7
#define G02 19

#define CS3 6
#define G03 18

#define SCK 13

#define testing false

#define MODEM_CONFIG RH_RF95::ModemConfigChoice::Bw500Cr45Sf128

bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;

//Declaring radio and buffer vars
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);

uint8_t pkt[255];
uint8_t length = sizeof(pkt);

bool read1;
bool read2;
bool read3;
bool read4;

//Declare functions
void testRead();
void packetRead();

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

int receiver_inc = 0;

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void setup(){
  setSyncProvider(getTeensy3Time);
  Serial.begin(BAUD);

  while(!Serial);

  radio1 = driver1.init();
  if (!radio1){
    Serial.println("init 1 failed"); 
  } else {
    Serial.println("init 1 succeded");
    driver1.setFrequency(915.0); // Median of Hz range
    driver1.setTxPower(RH_RF95_MAX_POWER, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
    // driver1.setModemConfig(MODEM_CONFIG); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
    driver1.setSpreadingFactor(7);
    driver1.setSignalBandwidth(500000);
    driver1.setCodingRate4(6);
    driver1.setModeRx();
  }
  
  radio2 = driver2.init();
  if (!radio2){
    Serial.println("init 2 failed"); 
  } else {
    Serial.println("init 2 succeded");
    driver2.setFrequency(915.0);
    driver2.setTxPower(RH_RF95_MAX_POWER, false);
    // driver2.setModemConfig(MODEM_CONFIG);
    driver2.setSpreadingFactor(7);
    driver2.setSignalBandwidth(500000);
    driver2.setCodingRate4(6);
    driver2.setModeRx();
  }

  radio3 = driver3.init();
  if (!radio3) {
    Serial.println("init 3 failed"); 
  } else{
    Serial.println("init 3 succeded");
    driver3.setFrequency(915.0);
    driver3.setTxPower(RH_RF95_MAX_POWER, false);
    // driver3.setModemConfig(MODEM_CONFIG);
    driver3.setSpreadingFactor(7);
    driver3.setSignalBandwidth(500000);
    driver3.setCodingRate4(6);
    driver3.setModeRx();
  }

  radio4 = driver4.init();
  if (!radio4){
    Serial.println("init 4 failed"); 
  } else{
    Serial.println("init 4 succeded");
    driver4.setFrequency(915.0);
    driver4.setTxPower(RH_RF95_MAX_POWER, false);
    // driver4.setModemConfig(MODEM_CONFIG);
    driver4.setSpreadingFactor(7);
    driver4.setSignalBandwidth(500000);
    driver4.setCodingRate4(6);
    driver4.setModeRx();
  }  
}

int timing = 0;
int timing1 = 0;
int timing2 = 0;
int timing3 = 0;
int timing4 = 0;

void loop() {
  read1 = false;
  read2 = false;
  read3 = false;
  read4 = false;

  //Serial.println("anti-hang tech: " + String(receiver_inc));

  switch (receiver_inc) {
    case 0: {
      if(radio1) {
        read1 = driver1.recv(pkt, &length);
        if (read1) {
          if(pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0){
            //Serial.println("Evaded zero 1");
          } else {
            if (testing){
              Serial.print("Driver1: ");
              testRead();
            } else {
              packetRead();
              // Serial.println("\nbuf len: " + String(length) + "\n");
            }
            int temp = millis();
            //Serial.println("A milli A milli A milli 3: " + String(temp - timing3));
            timing1 = temp;
          }
        }
      } break;
    } 
    case 1: {
      if(radio2) {
        read2 = driver2.recv(pkt, &length);
        if (read2) {
          if(pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0){
            //Serial.println("Evaded zero 2");
          } else {
            if (testing){
              Serial.print("Driver2: ");
              testRead();
            } else {
              packetRead();
              // Serial.println("\nbuf len: " + String(length) + "\n");
            }
            int temp = millis();
            //Serial.println("A milli A milli A milli 2: " + String(temp - timing3));
            timing2 = temp;
          }
        }
      } break;
    } 
    case 2: {
      if(radio3) {
        read3 = driver3.recv(pkt, &length);
        if (read3) {
          if(pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0){
            //Serial.println("evaded zero 3");
          } else {
            if (testing){
              Serial.print("Driver3: ");
              testRead();
            } else {
              packetRead();
              // Serial.println("\nbuf len: " + String(length) + "\n");
            }
            int temp = millis();
            //Serial.println("A milli A milli A milli 3: " + String(temp - timing3));
            timing3 = temp;
          }
        } 
      } break;
    } 
    case 3: {
      if(radio4) {
        read4 = driver4.recv(pkt, &length);
        if (read4) {
          if(pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0){
            //Serial.println("evaded zero 4");
          } else {
            if (testing){
              Serial.print("Driver4: ");
              testRead();
            } else {
              packetRead();
              // Serial.println("\nbuf len: " + String(length) + "\n");
            }
            int temp = millis();
            //Serial.println("A milli A milli A milli 4: " + String(temp - timing4));
            timing4 = temp;
          }
        }
      } break;
    } 
  }
  if (testing){
    //Serial.println("radio 1 init: " + String(radio1) + "\tradio 1 recive: " + String(read1));
    //Serial.println("radio 2 init: " + String(radio2) + "\tradio 2 recive: " + String(read2));
    //Serial.println("radio 3 init: " + String(radio3) + "\tradio 3 recive: " + String(read3));
    //Serial.println("radio 4 init: " + String(radio4) + "\tradio 4 recive: " + String(read4));
  }
  receiver_inc = (receiver_inc+1) % 4;
}


void packetRead() {
  //Serial.println("Packet read");
  // IMU pkt 4-28, 
  // Wheel Boards 29-6
  
  switch (pkt[0]) {
    case 0: {// General
      unsigned long timestamp = (unsigned long) pkt[1] << 24 | (unsigned long) pkt[2] << 16 | (unsigned long) pkt[3] << 8 | (unsigned long) pkt[4];
      float xAccel = (pkt[5] << 24) | (pkt[6] << 16) | (pkt[7] << 8) | pkt[8]; 
      float yAccel = (pkt[9] << 24) | (pkt[10] << 16) | (pkt[11] << 8) | pkt[12];
      float zAccel = (pkt[13] << 24) | (pkt[14] << 16) | (pkt[15] << 8) | pkt[16];

      float xGyro = (pkt[17] << 24) | (pkt[18] << 16) | (pkt[19] << 8) | pkt[20];
      float yGyro = (pkt[21] << 24) | (pkt[22] << 16) | (pkt[23] << 8) | pkt[24];
      float zGyro = (pkt[25] << 24) | (pkt[26] << 16) | (pkt[27] << 8) | pkt[28];

      float fl_speed = (pkt[29] << 8) | pkt[30];
      float fl_brakeTemp = (pkt[31] << 8) | pkt[32];
      float fl_ambTemp = (pkt[33] << 8) | pkt[34];

      float fr_speed = (pkt[35] << 8) | pkt[36];
      float fr_brakeTemp = (pkt[37] << 8) | pkt[38];
      float fr_ambTemp = (pkt[39] << 8) | pkt[40];

      float bl_speed = (pkt[41] << 8) | pkt[42];
      float bl_brakeTemp = (pkt[43] << 8) | pkt[44];
      float bl_ambTemp = (pkt[45] << 8) | pkt[46];

      float br_speed = (pkt[47] << 8) | pkt[48];
      float br_brakeTemp = (pkt[49] << 8) | pkt[50];
      float br_ambTemp = (pkt[51] << 8) | pkt[52];

      float differentialSpeed = (pkt[53] << 8) | pkt[54];

      uint8_t drsToggle = pkt[55];
      float steeringAngle = (pkt[56] << 8) | pkt[57];
      float throttleInput = (pkt[58] << 8) | pkt[59];

      float frontBrakePressure = (pkt[60] << 8) | pkt[61];
      float rearBrakePressure = (pkt[62] << 8) | pkt[63];

      float gpsLatitude = ((pkt[64] << 24) | (pkt[65] << 16) | (pkt[66] << 8) | pkt[67]) / 10000000;
      float gpsLongitude = ((pkt[68] << 24) | (pkt[69] << 16) | (pkt[70] << 8) | pkt[71]) / 10000000;

      float batteryVoltage = (pkt[72] << 24) | (pkt[73] << 16) | (pkt[74] << 8) | pkt[75];
      float daqCurrentDraw = (pkt[76] << 24) | (pkt[77] << 16) | (pkt[78] << 8) | pkt[79];

      float fl_shock = (pkt[80] << 8) | pkt[81];
      float fr_shock = (pkt[82] << 8) | pkt[83];
      float bl_shock = (pkt[84] << 8) | pkt[85];
      float br_shock = (pkt[86] << 8) | pkt[87];

      if (testing) {
        Serial.println("[General]");
      }

      Serial.print("0," + String(timestamp) + ","  + String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + "," + 
        String(xGyro)  + "," + String(yGyro)  + "," + String(zGyro) + ","); 
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
    case 1: {// Lap Timing
      uint8_t gateNum = pkt[1];

      uint8_t starting_second = pkt[2];
      uint8_t starting_minute = pkt[3];
      uint8_t starting_hour = pkt[4];
      uint8_t starting_day = pkt[5];
      uint8_t starting_month = pkt[6];
      uint8_t starting_year = pkt [7];

      int start_millis = ((pkt[8] << 24) | (pkt[9] << 16) | (pkt[10] << 8) | pkt[11]);
      int now_millis = ((pkt[12] << 24) | (pkt[13] << 16) | (pkt[14] << 8) | pkt[15]);

      Serial.println("1," + String(gateNum) + "," + String(starting_year) + "," + String(starting_month) + "," + String(starting_day) + "," + String(starting_hour) + "," + String(starting_minute) + "," + String(starting_second) + "," + String(start_millis) + "," + String(now_millis) + "," + String(now_millis-start_millis));
      break;
    }
    default: {
      Serial.println("Nothing Called");
      break;
    }
  }
}
void testRead(){
  for (unsigned int i = 0; i < sizeof(pkt); i++){
    Serial.print(String(pkt[i]) + ", ");
  }
  Serial.println();
}