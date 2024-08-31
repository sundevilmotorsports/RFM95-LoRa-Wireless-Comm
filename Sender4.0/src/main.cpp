#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <iostream>
#include <chrono>

#define CS0 10
#define G00 2
#define CS1 0
#define G01 3
#define LED 13
#define TESTING true

//Declaring radio and can
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

//Declare packets
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
uint8_t pkt1[78];
uint8_t pkt2[78];

//indexes:
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
  u_int16_t fl_speed;
  u_int16_t fr_speed;
  short fl_brakeTemp;
  short fr_brakeTemp;
  short fl_ambTemp;
  short fr_ambTemp;
  // BACK
  u_int16_t bl_speed;
  u_int16_t br_speed;
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

int temp;

// Declare functions
void canSniff(const CAN_message_t &msg);
void sendData(uint8_t pkt1[78]);

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup() {
  pinMode(LED, OUTPUT);
  
  Serial.begin(BAUD); //Set Baud rate, don't set too high or risk data loss - might be unused for teensy needs testing
  if (!driver2.init())
      Serial.println("init 2 failed"); 
  else
      Serial.println("init 2 succeded");
  if (!driver1.init())
      Serial.println("init 1 failed"); 
  else
      Serial.println("init 1 succeded");

  

  driver1.setFrequency(915.0); // Median of Hz range
  driver1.setTxPower(RH_RF95_MAX_POWER, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
  driver1.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
  
  driver2.setFrequency(915.0);
  driver2.setTxPower(RH_RF95_MAX_POWER, false);
  driver2.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);

  Can.begin();
  Can.setBaudRate(1000000);
  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  Can.onReceive(canSniff);
}


void canSniff(const CAN_message_t &msg)
{
  unsigned long currentMillis = millis();
  pkt1[0] = (currentMillis >> 24) & 0xFF;
  pkt1[1] = (currentMillis >> 16) & 0xFF;
  pkt1[2] = (currentMillis >> 8) & 0xFF;
  pkt1[3] = currentMillis & 0xFF;
  switch (msg.id)
  {
    case 0x360:
      xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt1[4] = (xAccel >> 24) & 0xFF;
      pkt1[5] = (xAccel >> 16) & 0xFF;
      pkt1[6] = (xAccel >> 8) & 0xFF;
      pkt1[7] = xAccel & 0xFF;
      temp = pkt1[4] | pkt1[5] | pkt1[6] | pkt1[7];
      yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt1[8] = (yAccel >> 24) & 0xFF;
      pkt1[9] = (yAccel >> 16) & 0xFF;
      pkt1[10] = (yAccel >> 8) & 0xFF;
      pkt1[11] = yAccel & 0xFF;
      break;
    case 0x361:
      zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt1[12] = (zAccel >> 24) & 0xFF;
      pkt1[13] = (zAccel >> 16) & 0xFF;
      pkt1[14] = (zAccel >> 8) & 0xFF;
      pkt1[15] = zAccel & 0xFF;
      xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt1[16] = (xGyro >> 24) & 0xFF;
      pkt1[17] = (xGyro >> 16) & 0xFF;
      pkt1[18] = (xGyro >> 8) & 0xFF;
      pkt1[19] = xGyro & 0xFF;
      break;
    case 0x362:
      yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt1[20] = (yGyro >> 24) & 0xFF;
      pkt1[21] = (yGyro >> 16) & 0xFF;
      pkt1[22] = (yGyro >> 8) & 0xFF;
      pkt1[23] = yGyro & 0xFF;
      zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt1[24] = (zGyro >> 24) & 0xFF;
      pkt1[25] = (zGyro >> 16) & 0xFF;
      pkt1[26] = (zGyro >> 8) & 0xFF;
      pkt1[27] = zGyro & 0xFF;
      break;
    case 0x363:
      fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt1[28] = (fl_speed >> 8) & 0xFF;
      pkt1[29] = fl_speed & 0xFF;
      fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt1[30] = (fl_brakeTemp >> 8) & 0xFF;
      pkt1[31] = fl_brakeTemp & 0xFF;
      fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt1[32] = (fl_ambTemp >> 8) & 0xFF;
      pkt1[33] = fl_ambTemp & 0xFF;
      break;
    case 0x364:
      fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt1[34] = (fr_speed >> 8) & 0xFF;
      pkt1[35] = fr_speed & 0xFF;
      fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt1[36] = (fr_brakeTemp >> 8) & 0xFF;
      pkt1[37] = fr_brakeTemp & 0xFF;
      fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt1[38] = (fr_ambTemp >> 8) & 0xFF;
      pkt1[39] = fr_ambTemp & 0xFF;
      break;
    case 0x365:
      bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt1[40] = (bl_speed >> 8) & 0xFF;
      pkt1[41] = bl_speed & 0xFF;
      bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt1[42] = (bl_brakeTemp >> 8) & 0xFF;
      pkt1[43] = bl_brakeTemp & 0xFF;
      bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt1[44] = (bl_ambTemp >> 8) & 0xFF;
      pkt1[45] = bl_ambTemp & 0xFF;
      break;
    case 0x366:
      br_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt1[46] = (br_speed >> 8) & 0xFF;
      pkt1[47] = br_speed & 0xFF;
      br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt1[48] = (br_brakeTemp >> 8) & 0xFF;
      pkt1[49] = br_brakeTemp & 0xFF;
      br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt1[50] = (br_ambTemp >> 8) & 0xFF;
      pkt1[51] = br_ambTemp & 0xFF;
      break;
    case 0x367:
      DRS = msg.buf[0];
      pkt1[52] = DRS ? 1 : 0;
      break;
    case 0x368:
      steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
      pkt1[53] = (steeringAngle << 8) & 0xFF;
      pkt1[54] = steeringAngle & 0xFF;
      throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
      pkt1[55] = (throttleInput << 8) & 0xFF;
      pkt1[56] = throttleInput & 0xFF;
      frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;
      pkt1[57] = (frontBrakePressure << 8) & 0xFF;
      pkt1[58] = frontBrakePressure & 0xFF;
      rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
      pkt1[59] = (rearBrakePressure << 8) & 0xFF;
      pkt1[60] = rearBrakePressure & 0xFF;
      break;
    case 0x369:
      gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt1[61] = (gps_lat >> 24) & 0xFF;
      pkt1[62] = (gps_lat >> 16) & 0xFF;
      pkt1[63] = (gps_lat >> 8) & 0xFF;
      pkt1[64] = gps_lat & 0xFF;
      gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt1[65] = (gps_long >> 24) & 0xFF;
      pkt1[66] = (gps_long >> 16) & 0xFF;
      pkt1[67] = (gps_long >> 8) & 0xFF;
      pkt1[68] = gps_long & 0xFF;
      break;
    case 0x36A:
      batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt1[69] = (batteryVoltage >> 24) & 0xFF;
      pkt1[70] = (batteryVoltage >> 16) & 0xFF;
      pkt1[71] = (batteryVoltage >> 8) & 0xFF;
      pkt1[72] = batteryVoltage & 0xFF;
      daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt1[73] = (batteryVoltage >> 24) & 0xFF;
      pkt1[74] = (batteryVoltage >> 16) & 0xFF;
      pkt1[75] = (batteryVoltage >> 8) & 0xFF;
      pkt1[76] = batteryVoltage & 0xFF;
      break;
  }
  //Only prints if serial monitor is open and testing
  if(Serial && TESTING){
    Serial.println("Received Data Log: " + String(msg.id) + ",\t" + String(currentMillis) + " ms"
                     + "\nDRS, " + String(DRS)
                     + "\nSteering Angle, " + String(steeringAngle) + " °"
                     + "\nThrottle, " + String(throttleInput) + " %"
                     + "\nBrake Pressure, " + String(frontBrakePressure) + " BAR,\t" + String(rearBrakePressure) + "BAR"
                     + "\ngps, " + String(gps_lat) + " Decimal Degrees,\t" + String(gps_long) + " Decimal Degrees"
                     + "\nBattery, " + String(batteryVoltage) + " mV,\t" + String(daqCurrentDraw) + " mA");
     Serial.println("Recived imu data: " + String(currentMillis) + " ms"
                     + "\nAcceleration, " + String(xAccel) + " mG,\t" + String(yAccel) + " mG,\t" + String(zAccel) + " mG"
                     + "\nGyro, " + String(xGyro) + " mdps,\t" + String(yGyro) + " mpds,\t" + String(zGyro) + " mdps");
    Serial.println("Received Wheel Data: " + String(currentMillis) + " ms"
                    + "\nFront left, " + String(fl_speed) + " RPM,\t" + String(fl_brakeTemp) + " °,\t" + String(fl_ambTemp) + "°"
                    + "\nFront right, "+ String(fr_speed) + " RPM,\t" + String(fr_brakeTemp) + " °,\t" + String(fr_ambTemp) + "°"
                    + "\nRear left, "  + String(bl_speed) + " RPM,\t" + String(bl_brakeTemp) + " °,\t" + String(bl_ambTemp) + "°"
                    + "\nRear right, " + String(br_speed) + " RPM,\t" + String(br_brakeTemp) + " °,\t" + String(br_ambTemp) + "°");
    
  }
}

void testPacket(){
  bool boolTest = rand();
  unsigned long timeTest = rand();
  short shortTest = rand();
  u_int16_t int16Test = rand();
  int intTest = rand();
  
  pkt1[0] = (timeTest >> 24) & 0xFF;
  pkt1[1] = (timeTest >> 16) & 0xFF;
  pkt1[2] = (timeTest >> 8) & 0xFF;
  pkt1[3] = timeTest & 0xFF;

  pkt1[4] = (intTest >> 24) & 0xFF;
  pkt1[5] = (intTest >> 16) & 0xFF;
  pkt1[6] = (intTest >> 8) & 0xFF;
  pkt1[7] = intTest & 0xFF;
  
  pkt1[28] = (int16Test >> 8) & 0xFF;
  pkt1[29] = int16Test & 0xFF;

  pkt1[30] = (shortTest >> 8) & 0xFF;
  pkt1[31] = shortTest & 0xFF;

  pkt1[52] = boolTest ? 1 : 0;

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
  // if(TESTING){
  //   testPacket();
  // }
  bool sent_pkt1 = driver1.send(pkt1, sizeof(pkt1));
  bool sent_pkt2 = driver2.send(pkt2, sizeof(pkt2));
  driver1.waitPacketSent();
  driver2.waitPacketSent();
  if(Serial){
    Serial.println("Sent pkt1: " + String(sent_pkt1));
    Serial.println("Sent pkt2: " + String(sent_pkt2));
  }
}

