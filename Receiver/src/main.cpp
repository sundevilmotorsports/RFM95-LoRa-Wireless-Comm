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

#define testing true

//Declaring radio and buffer vars
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
uint8_t pkt[76];
uint8_t length = sizeof(pkt);
bool read;

//Declare functions
// void imuRead();
// void wheelRead();
// void dataLogRead();
void testRead();
void canRead();

//interval for checking if sender is available in milliseconds
const unsigned long INTERVAL1 = 500;

//interval for waiting for new packet before searching for connection
const unsigned long INTERVAL2 = 5000;

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup(){
  Serial.begin(BAUD);
  while(!Serial);
  driver1.init();
  if (!driver1.init())
    Serial.println("driver 1 failed"); 
  else
    Serial.println("driver 1 succeded");

  driver2.init();
  if (!driver2.init())
    Serial.println("driver 2 failed"); 
  else
    Serial.println("driver 2 succeded");

  driver1.setFrequency(915.0); // Median of Hz range
  driver1.setTxPower(23, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
  driver1.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
  driver1.setModeRx();
}

void loop() {
  if(driver1.available()){
    read = driver1.recv(pkt, &length);
    if (testing){
      testRead();
    } else {
      canRead();
    }
    driver1.waitAvailableTimeout(INTERVAL2);
  } else {
    Serial.println("Sender not available :(");
    delay(INTERVAL1);
  }
}


void canRead() {
// IMU pkt 4-28, 
// Wheel Boards 29-6
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

  uint8_t drsToggle = pkt[52];
  float steeringAngle = (pkt[53] << 8) | pkt[54];
  float throttleInput = (pkt[55] << 8) | pkt[56];

  float frontBrakePressure = (pkt[57] << 8) | pkt[58];
  float rearBrakePressure = (pkt[59] << 8) | pkt[60];

  float gpsLatitude = ((pkt[61] << 24) | (pkt[62] << 16) | (pkt[63] << 8) | pkt[64])/10000000;
  float gpsLongitude = ((pkt[65]<< 24) | (pkt[66] << 16) | (pkt[67] << 8) | pkt[68])/10000000;
  
  float batteryVoltage = (pkt[69] << 24) | (pkt[70] << 16) | (pkt[71] << 8) | pkt[72];
  float daqCurrentDraw = (pkt[73] << 24) | (pkt[74] << 16) | (pkt[75] << 8) | pkt[76];

  Serial.print("1," + String(timestamp) + ","  + 
    String(xAccel) + "," + String(yAccel) + "," + String(zAccel) + "," + 
    String(xGyro)  + "," + String(yGyro)  + "," + String(zGyro)  + "\n");
  Serial.print("2," + String(timestamp) + ","  + 
    String(fl_speed) + "," + String(fl_brakeTemp) + "," + String(fl_ambTemp) + "," + 
    String(fr_speed) + "," + String(fr_brakeTemp) + "," + String(fr_ambTemp) + "," + 
    String(bl_speed) + "," + String(bl_brakeTemp) + "," + String(bl_ambTemp) + "," +
    String(br_speed) + "," + String(br_brakeTemp) + "," + String(br_ambTemp) + "\n");
  Serial.print("3," + String(timestamp) + ","  + String(drsToggle) + "," + String(steeringAngle) + "," + String(throttleInput) + "," + 
    String(frontBrakePressure) + "," + String(rearBrakePressure) + "," + 
    String(gpsLatitude)        + "," + String(gpsLongitude)      + "," + 
    String(batteryVoltage)     + "," + String(daqCurrentDraw)    + "\n");
}
void testRead(){
  for (int i = 0; i < sizeof(pkt); i++){
    Serial.print(String(pkt[i]) + ", ");
  }
  Serial.println();
}