#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <iostream>
#include <chrono>

#define CS0 38
#define G00 21

#define CS1 37
#define G01 20

#define CS2 36
#define G02 19

#define CS3 10
#define G03 18

#define LED 13

#define TESTING false

uint8_t mode = 0;

//Declaring radio and can
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

uint8_t susOffset = 63;
uint8_t damperOffset = 26;
uint8_t driveOffset = 37;
uint8_t slideOffset = 30;

uint8_t susGroupNum = 2;    
uint8_t damperGroupNum = 9; 
uint8_t driveGroupNum = 2;  
uint8_t slideGroupNum = 5;

uint8_t currSus = 0;    
uint8_t currDamp = 0; 
uint8_t currDrive = 0;  
uint8_t currSlide = 0;

bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;
//Declare packets
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
//Length Determined by offset (packet size) * number of groups being sent
uint8_t general[90];
uint8_t suspension[126];
uint8_t damper[234];
uint8_t drive[74];
uint8_t slide[150];

//indexes for general:
  //imu starts at 4
  //wheel packet starts at 28
  //daq packet starts at 52

// IMU VARIABLES
int xAccel = -1;
int yAccel = -1;
int zAccel = -1;
int xGyro = -1;
int yGyro = -1;
int zGyro = -1;

// WHEEL VARIABLES
  // FRONT
  uint16_t fl_speed;
  uint16_t fr_speed;
  short fl_brakeTemp;
  short fr_brakeTemp;
  short fl_ambTemp;
  short fr_ambTemp;
  // BACK
  uint16_t bl_speed;
  uint16_t br_speed;
  short bl_brakeTemp;
  short br_brakeTemp;
  short bl_ambTemp;
  short br_ambTemp;

// DATALOG VARIABLES
bool DRS = false;
int steeringAngle = -1;
int throttleInput = -1;
int frontBrakePressure = -1;
int rearBrakePressure = -1;
int gps_lat = -1;
int gps_long = -1;
int batteryVoltage = -1;
int daqCurrentDraw = -1;


// Declare functions
void canSniff(const CAN_message_t &msg);
void getPacket();

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup() {
  pinMode(LED, OUTPUT);

  Serial.begin(BAUD); //Set Baud rate, don't set too high or risk data loss - might be unused for teensy needs testing
  radio1 = driver1.init();
  radio2 = driver2.init();
  radio3 = driver3.init();
  radio4 = driver4.init();
  if (!radio1)
      Serial.println("init 1 failed"); 
  else
      Serial.println("init 1 succeded");

  if (!radio2)
      Serial.println("init 2 failed"); 
  else
      Serial.println("init 2 succeded");

  if (!radio3)
      Serial.println("init 3 failed"); 
  else
      Serial.println("init 3 succeded");

  if (!radio4)
      Serial.println("init 4 failed"); 
  else
      Serial.println("init 4 succeded");

  driver1.setFrequency(915.0); // Median of Hz range
  driver1.setTxPower(RH_RF95_MAX_POWER, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
  driver1.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048

  driver2.setFrequency(915.0);
  driver2.setTxPower(RH_RF95_MAX_POWER, false);
  driver2.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);

  driver3.setFrequency(915.0);
  driver3.setTxPower(RH_RF95_MAX_POWER, false);
  driver3.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);

  driver4.setFrequency(915.0);
  driver4.setTxPower(RH_RF95_MAX_POWER, false);
  driver4.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);

  Can.begin();
  Can.setBaudRate(1000000);
  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  Can.onReceive(canSniff);
}


void canSniff(const CAN_message_t &msg)
{
  unsigned long currentMillis = millis();
  general[0] = (currentMillis >> 24) & 0xFF;
  general[1] = (currentMillis >> 16) & 0xFF;
  general[2] = (currentMillis >> 8) & 0xFF;
  general[3] = currentMillis & 0xFF;

  switch (msg.id)
  {
    case 0x360:
      xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      general[4] = (xAccel >> 24) & 0xFF;
      general[5] = (xAccel >> 16) & 0xFF;
      general[6] = (xAccel >> 8) & 0xFF;
      general[7] = xAccel & 0xFF;
      yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      general[8] = (yAccel >> 24) & 0xFF;
      general[9] = (yAccel >> 16) & 0xFF;
      general[10] = (yAccel >> 8) & 0xFF;
      general[11] = yAccel & 0xFF;
      break;
    case 0x361:
      zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      general[12] = (zAccel >> 24) & 0xFF;
      general[13] = (zAccel >> 16) & 0xFF;
      general[14] = (zAccel >> 8) & 0xFF;
      general[15] = zAccel & 0xFF;
      xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      general[16] = (xGyro >> 24) & 0xFF;
      general[17] = (xGyro >> 16) & 0xFF;
      general[18] = (xGyro >> 8) & 0xFF;
      general[19] = xGyro & 0xFF;
      break;
    case 0x362:
      yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      general[20] = (yGyro >> 24) & 0xFF;
      general[21] = (yGyro >> 16) & 0xFF;
      general[22] = (yGyro >> 8) & 0xFF;
      general[23] = yGyro & 0xFF;
      zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      general[24] = (zGyro >> 24) & 0xFF;
      general[25] = (zGyro >> 16) & 0xFF;
      general[26] = (zGyro >> 8) & 0xFF;
      general[27] = zGyro & 0xFF;
      break;
    case 0x363:
      fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      general[28] = (fl_speed >> 8) & 0xFF;
      general[29] = fl_speed & 0xFF;
      fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      general[30] = (fl_brakeTemp >> 8) & 0xFF;
      general[31] = fl_brakeTemp & 0xFF;
      fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      general[32] = (fl_ambTemp >> 8) & 0xFF;
      general[33] = fl_ambTemp & 0xFF;
      break;
    case 0x364:
      fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
      general[34] = (fr_speed >> 8) & 0xFF;
      general[35] = fr_speed & 0xFF;
      fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      general[36] = (fr_brakeTemp >> 8) & 0xFF;
      general[37] = fr_brakeTemp & 0xFF;
      fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      general[38] = (fr_ambTemp >> 8) & 0xFF;
      general[39] = fr_ambTemp & 0xFF;
      break;
    case 0x365:
      bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      general[40] = (bl_speed >> 8) & 0xFF;
      general[41] = bl_speed & 0xFF;
      bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      general[42] = (bl_brakeTemp >> 8) & 0xFF;
      general[43] = bl_brakeTemp & 0xFF;
      bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      general[44] = (bl_ambTemp >> 8) & 0xFF;
      general[45] = bl_ambTemp & 0xFF;
      break;
    case 0x366:
      br_speed = (msg.buf[0]) | msg.buf[1] << 8;
      general[46] = (br_speed >> 8) & 0xFF;
      general[47] = br_speed & 0xFF;
      br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      general[48] = (br_brakeTemp >> 8) & 0xFF;
      general[49] = br_brakeTemp & 0xFF;
      br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      general[50] = (br_ambTemp >> 8) & 0xFF;
      general[51] = br_ambTemp & 0xFF;
      break;
    case 0x367:
      DRS = msg.buf[0];
      general[54] = DRS ? 1 : 0;
      break;
    case 0x368:
      steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
      general[55] = (steeringAngle << 8) & 0xFF;
      general[56] = steeringAngle & 0xFF;
      throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
      general[57] = (throttleInput << 8) & 0xFF;
      general[58] = throttleInput & 0xFF;
      frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;
      general[59] = (frontBrakePressure << 8) & 0xFF;
      general[60] = frontBrakePressure & 0xFF;
      rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
      general[61] = (rearBrakePressure << 8) & 0xFF;
      general[62] = rearBrakePressure & 0xFF;
      break;
    case 0x369:
      gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      general[63] = (gps_lat >> 24) & 0xFF;
      general[64] = (gps_lat >> 16) & 0xFF;
      general[65] = (gps_lat >> 8) & 0xFF;
      general[66] = gps_lat & 0xFF;
      gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      general[67] = (gps_long >> 24) & 0xFF;
      general[68] = (gps_long >> 16) & 0xFF;
      general[69] = (gps_long >> 8) & 0xFF;
      general[70] = gps_long & 0xFF;
      break;
    case 0x36A:
      batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      general[71] = (batteryVoltage >> 24) & 0xFF;
      general[72] = (batteryVoltage >> 16) & 0xFF;
      general[73] = (batteryVoltage >> 8) & 0xFF;
      general[74] = batteryVoltage & 0xFF;
      daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      general[75] = (batteryVoltage >> 24) & 0xFF;
      general[76] = (batteryVoltage >> 16) & 0xFF;
      general[77] = (batteryVoltage >> 8) & 0xFF;
      general[78] = batteryVoltage & 0xFF;
      break;
  }
  //Only prints if serial monitor is open
  if(Serial && !TESTING){
    Serial.println("Received Data Log: " + String(msg.id) + ", " + String(currentMillis) + "ms"
                     + "\nDRS, " + String(DRS)
                     + "\nSteering Angle, " + String(steeringAngle) + "°"
                     + "\nThrottle, " + String(throttleInput) + "%"
                     + "\nBrake Pressure, " + String(frontBrakePressure) + "BAR, " + String(rearBrakePressure) + "BAR"
                     + "\ngps, " + String(gps_lat) + "Decimal Degrees, " + String(gps_long) + "Decimal Degrees"
                     + "\nBattery, " + String(batteryVoltage) + "mV, " + String(daqCurrentDraw) + "mA");
     Serial.println("Recived imu data: " + String(currentMillis) + "ms"
                     + "\nAcceleration, " + String(xAccel) + "mG, " + String(yAccel) + "mG, " + String(zAccel) + "mG"
                     + "\nGyro, " + String(xGyro) + "mdps, " + String(yGyro) + "mpds, " + String(zGyro) + "mdps");
    Serial.println("Received Wheel Data: " + String(currentMillis) + "ms"
                    + "\nFront left, " + String(fl_speed) + "RPM, " + String(fl_brakeTemp) + "°, " + String(fl_ambTemp) + "°"
                    + "\nFront right, "+ String(fr_speed) + "RPM, " + String(fr_brakeTemp) + "°, " + String(fr_ambTemp) + "°"
                    + "\nRear left, "  + String(bl_speed) + "RPM, " + String(bl_brakeTemp) + "°, " + String(bl_ambTemp) + "°"
                    + "\nRear right, " + String(br_speed) + "RPM, " + String(br_brakeTemp) + "°, " + String(br_ambTemp) + "°");
  }
}

void testPacket(){
  bool boolTest = rand();
  unsigned long timeTest = rand();
  short shortTest = rand();
  uint16_t int16Test = rand();
  int intTest = rand();
  
  general[0] = (timeTest >> 24) & 0xFF;
  general[1] = (timeTest >> 16) & 0xFF;
  general[2] = (timeTest >> 8) & 0xFF;
  general[3] = timeTest & 0xFF;

  general[4] = (intTest >> 24) & 0xFF;
  general[5] = (intTest >> 16) & 0xFF;
  general[6] = (intTest >> 8) & 0xFF;
  general[7] = intTest & 0xFF;
  
  general[28] = (int16Test >> 8) & 0xFF;
  general[29] = int16Test & 0xFF;

  general[30] = (shortTest >> 8) & 0xFF;
  general[31] = shortTest & 0xFF;

  general[52] = boolTest ? 1 : 0;

  if(Serial){
    Serial.println("Created test data: \nlong,\t" 
                  + String(timeTest) + "ms"
                  + "\nbool,\t" + String(boolTest)
                  + "\nshort,\t" + String(shortTest)
                  + "\n16int,\t" + String(int16Test)
                  + "\nint,\t" + String(intTest));
  }
}
void loop() {
  int timing = ((general[0] | general[1] | general[2] | general[3]) % 1000)/250;
  bool sent_pkt1 = false;
  bool sent_pkt2 = false;
  bool sent_pkt3 = false;
  bool sent_pkt4 = false;
  getPacket();

  switch (timing){
    case  0:
      if(!radio1){
        //Serial.println("Driver 1 unresponsive");
        break;
      }
      switch(mode){
        case 0:
          sent_pkt1 = driver1.send(general, sizeof(general));
          break;
        case 1:
          sent_pkt1 = driver1.send(suspension, sizeof(suspension));
          break;
        case 2:
          sent_pkt1 = driver1.send(damper, sizeof(damper));
          break;
        case 3:
          sent_pkt1 = driver1.send(drive, sizeof(drive));
          break;
        case 4:
          sent_pkt1 = driver1.send(slide, sizeof(slide));
          break;
      }
      if (Serial){
        Serial.println("Sent pkt: " + String(sent_pkt1) + "\tmode: " + String(mode));
      }
      break;

    case 1:
      if(!radio2){
        //Serial.println("Driver 2 unresponsive");
        break;
      }
      switch(mode){
        case 0:
          sent_pkt2 = driver2.send(general, sizeof(general));
          break;
        case 1:
          sent_pkt2 = driver2.send(suspension, sizeof(suspension));
          break;
        case 2:
          sent_pkt2 = driver2.send(damper, sizeof(damper));
          break;
        case 3:
          sent_pkt2 = driver2.send(drive, sizeof(drive));
          break;
        case 4:
          sent_pkt2 = driver2.send(slide, sizeof(slide));
          break;         
      }
      if (Serial){
        Serial.println("Sent pkt: " + String(sent_pkt2 * 2) + "\tmode: " + String(mode));
      }
      break;
    case 2:
      if(!radio3){
        //Serial.println("Driver 3 unresponsive");
        break;
      }
      switch(mode){
        case 0:
          sent_pkt3 = driver3.send(general, sizeof(general));
          break;
        case 1:
          sent_pkt3 = driver3.send(suspension, sizeof(suspension));
          break;
        case 2:
          sent_pkt3 = driver3.send(damper, sizeof(damper));
          break;
        case 3:
          sent_pkt3 = driver3.send(drive, sizeof(drive));
          break;
        case 4:
          sent_pkt3 = driver3.send(slide, sizeof(slide));
      }
      if (Serial){
        Serial.println("Sent pkt: " + String(sent_pkt3 * 3) + "\tmode: " + String(mode));
      }
      break;
    case 3:
      if(!radio4){
        //Serial.println("Driver 4 unresponsive");
        break;
      }
      switch(mode){
        case 0:
          sent_pkt4 = driver4.send(general, sizeof(general));
        case 1:
          driver4.send(suspension, sizeof(suspension));
          break;
        case 2:
          sent_pkt4 = driver4.send(damper, sizeof(damper));
          break;
        case 3:
          sent_pkt4 = driver4.send(drive, sizeof(drive));
          break;
        case 4:
          sent_pkt4 = driver4.send(slide, sizeof(slide));
      }
      if (Serial){
        Serial.println("Sent pkt: " + String(sent_pkt4 * 4) + "\tmode: " + String(mode));
      }
      break;
  }
}

void getPacket(){
  uint8_t currOffset;
  switch(mode){
    case 1:
      currOffset = susOffset * currSus;
      if ((general[0] | general[1] | general[2] | general[3]) > (suspension[0 + currOffset] | suspension[1 + currOffset] | suspension[2 + currOffset] | suspension[3 + currOffset]) + 576) {
        if(currSus + 1 == susGroupNum){
          currSus = 0;
        } else {
          currSus ++;
        }
        suspension[0 + currOffset] = general[0];
        suspension[1 + currOffset] = general[1];
        suspension[2 + currOffset] = general[2];
        suspension[3 + currOffset] = general[3];
        currOffset = susOffset * currSus;
        suspension[0 + currOffset] = general[0];
        suspension[1 + currOffset] = general[1];
        suspension[2 + currOffset] = general[2];
        suspension[3 + currOffset] = general[3];
      }
      suspension[4 + currOffset] = general[4];
      suspension[5 + currOffset] = general[5];
      suspension[6 + currOffset] = general[6];
      suspension[7 + currOffset] = general[7];
      suspension[8 + currOffset] = general[8];
      suspension[9 + currOffset] = general[9];
      suspension[10 + currOffset] = general[10];
      suspension[11 + currOffset] = general[11];
      suspension[12 + currOffset] = general[12];
      suspension[13 + currOffset] = general[13];
      suspension[14 + currOffset] = general[14];
      suspension[15 + currOffset] = general[15];
      suspension[16 + currOffset] = general[16];
      suspension[17 + currOffset] = general[17];
      suspension[18 + currOffset] = general[18];
      suspension[19 + currOffset] = general[19];
      suspension[20 + currOffset] = general[20];
      suspension[21 + currOffset] = general[21];
      suspension[22 + currOffset] = general[22];
      suspension[23 + currOffset] = general[23];
      suspension[24 + currOffset] = general[24];
      suspension[25 + currOffset] = general[25];
      suspension[26 + currOffset] = general[26];
      suspension[27 + currOffset] = general[27];
      suspension[28 + currOffset] = general[28];
      suspension[29 + currOffset] = general[29];

      suspension[30 + currOffset] = general[34];
      suspension[31 + currOffset] = general[35];

      suspension[32 + currOffset] = general[40];
      suspension[33 + currOffset] = general[41];

      suspension[34 + currOffset] = general[46];
      suspension[35 + currOffset] = general[47];

      suspension[36 + currOffset] = general[52];
      suspension[37 + currOffset] = general[53];
      suspension[38 + currOffset] = general[54];
      suspension[39 + currOffset] = general[55];
      suspension[40 + currOffset] = general[56];
      suspension[41 + currOffset] = general[57];
      suspension[42 + currOffset] = general[58];
      suspension[43 + currOffset] = general[59];
      suspension[44 + currOffset] = general[60];
      suspension[45 + currOffset] = general[61];
      suspension[46 + currOffset] = general[62];
      suspension[47 + currOffset] = general[63];
      suspension[48 + currOffset] = general[64];
      suspension[49 + currOffset] = general[65];
      suspension[50 + currOffset] = general[66];
      suspension[51 + currOffset] = general[67];
      suspension[52 + currOffset] = general[68];
      suspension[53 + currOffset] = general[69];
      suspension[54 + currOffset] = general[70];

      suspension[55 + currOffset] = general[79];
      suspension[56 + currOffset] = general[80];
      suspension[57 + currOffset] = general[81];
      suspension[58 + currOffset] = general[82];
      suspension[59 + currOffset] = general[83];
      suspension[60 + currOffset] = general[84];
      suspension[61 + currOffset] = general[85];
      suspension[62 + currOffset] = general[86];
      suspension[62 + currOffset] = general[87];
      break;
    case 2:
      currOffset = damperOffset * currDamp;
      if ((general[0] | general[1] | general[2] | general[3]) > ( damper[0 + currOffset] | damper[1 + currOffset] | damper[2 + currOffset] | damper[3 + currOffset]) + 981) {
        if(currDamp + 1 == damperGroupNum){
          currDamp = 0;
        } else {
          currDamp ++;
        }
        damper[0 + currOffset] = general[0];
        damper[1 + currOffset] = general[1];
        damper[2 + currOffset] = general[2];
        damper[3 + currOffset] = general[3];
        currOffset = damperOffset * currDamp;
        damper[0 + currOffset] = general[0];
        damper[1 + currOffset] = general[1];
        damper[2 + currOffset] = general[2];
        damper[3 + currOffset] = general[3];
      }
      damper[4 + currOffset] = general[4];
      damper[5 + currOffset] = general[5];
      damper[6 + currOffset] = general[6];
      damper[7 + currOffset] = general[7];
      damper[8 + currOffset] = general[8];
      damper[9 + currOffset] = general[9];
      damper[10 + currOffset] = general[10];
      damper[11 + currOffset] = general[11];
      damper[12 + currOffset] = general[12];
      damper[13 + currOffset] = general[13];
      damper[14 + currOffset] = general[14];
      damper[15 + currOffset] = general[15];

      damper[16 + currOffset] = general[28];
      damper[17 + currOffset] = general[29];

      damper[18 + currOffset] = general[79];
      damper[19 + currOffset] = general[80];
      damper[20 + currOffset] = general[81];
      damper[21 + currOffset] = general[82];
      damper[22 + currOffset] = general[83];
      damper[23 + currOffset] = general[84];
      damper[24 + currOffset] = general[85];
      damper[25 + currOffset] = general[86];
    case 3:
      currOffset = driveOffset * currDrive;
      if ((general[0] | general[1] | general[2] | general[3]) > ( drive[0 + currOffset] | drive[1 + currOffset] | drive[2 + currOffset] | drive[3 + currOffset]) + 381) {
        if(currDrive + 1 == driveGroupNum){
          currDrive = 0;
        } else {
          currDrive ++;
        }
        drive[0 + currOffset] = general[0];
        drive[1 + currOffset] = general[1];
        drive[2 + currOffset] = general[2];
        drive[3 + currOffset] = general[3];
        currOffset = driveOffset * currDrive;
        drive[0 + currOffset] = general[0];
        drive[1 + currOffset] = general[1];
        drive[2 + currOffset] = general[2];
        drive[3 + currOffset] = general[3];
      }
      drive[4 + currOffset] = general[4];
      drive[5 + currOffset] = general[5];
      drive[6 + currOffset] = general[6];
      drive[7 + currOffset] = general[7];
      drive[8 + currOffset] = general[8];
      drive[9 + currOffset] = general[9];
      drive[10 + currOffset] = general[10];
      drive[11 + currOffset] = general[11];

      drive[12 + currOffset] = general[28];
      drive[13 + currOffset] = general[29];

      drive[14 + currOffset] = general[40];
      drive[15 + currOffset] = general[41];

      drive[16 + currOffset] = general[46];
      drive[17 + currOffset] = general[47];

      drive[18 + currOffset] = general[52];
      drive[19 + currOffset] = general[53];
      drive[20 + currOffset] = general[54];
      drive[21 + currOffset] = general[55];
      drive[22 + currOffset] = general[56];
      drive[23 + currOffset] = general[57];
      drive[24 + currOffset] = general[58];
      drive[25 + currOffset] = general[59];
      drive[26 + currOffset] = general[60];
      drive[27 + currOffset] = general[61];
      drive[28 + currOffset] = general[62];
      
      drive[29 + currOffset] = general[79];
      drive[30 + currOffset] = general[80];
      drive[31 + currOffset] = general[81];
      drive[32 + currOffset] = general[82];
      drive[33 + currOffset] = general[83];
      drive[34 + currOffset] = general[84];
      drive[35 + currOffset] = general[85];
      drive[36 + currOffset] = general[86];
    case 4:
      currOffset = slideOffset * currSlide;
      if ((general[0] | general[1] | general[2] | general[3]) > ( slide[0 + currOffset] | slide[1 + currOffset] | slide[2 + currOffset] | slide[3 + currOffset]) + 666) {
        if(currSlide + 1 == slideGroupNum){
          currSlide = 0;
        } else {
          currSlide ++;
        }
        slide[0 + currOffset] = general[0];
        slide[1 + currOffset] = general[1];
        slide[2 + currOffset] = general[2];
        slide[3 + currOffset] = general[3];
        currOffset = slideOffset * currSlide;
        slide[0 + currOffset] = general[0];
        slide[1 + currOffset] = general[1];
        slide[2 + currOffset] = general[2];
        slide[3 + currOffset] = general[3];
      }
      slide[4 + currOffset] = general[4];
      slide[5 + currOffset] = general[5];
      slide[6 + currOffset] = general[6];
      slide[7 + currOffset] = general[7];
      slide[8 + currOffset] = general[8];
      slide[9 + currOffset] = general[9];
      slide[10 + currOffset] = general[10];
      slide[11 + currOffset] = general[11];

      slide[12 + currOffset] = general[28];
      slide[13 + currOffset] = general[29];

      slide[14 + currOffset] = general[34];
      slide[15 + currOffset] = general[35];

      slide[16 + currOffset] = general[40];
      slide[17 + currOffset] = general[41];

      slide[18 + currOffset] = general[46];
      slide[19 + currOffset] = general[47];

      slide[20 + currOffset] = general[52];
      slide[21 + currOffset] = general[53];

      slide[22 + currOffset] = general[55];
      slide[23 + currOffset] = general[56];
      slide[24 + currOffset] = general[57];
      slide[25 + currOffset] = general[58];
      slide[26 + currOffset] = general[59];
      slide[27 + currOffset] = general[60];
      slide[28 + currOffset] = general[61];
      slide[29 + currOffset] = general[62];
    default:
      break;
  }
}