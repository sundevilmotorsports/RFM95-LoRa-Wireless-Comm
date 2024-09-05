#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <iostream>
#include <chrono>

#define CS0 37
#define G00 21

#define CS1 36
#define G01 20

#define LED 13


#define TESTING false

//Declaring radio and can
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

//Declare packets
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
uint8_t pkt0[78];
uint8_t pkt1[78];

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


// Declare functions
void canSniff(const CAN_message_t &msg);
void sendData(uint8_t pkt0[78]);

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup() {
  pinMode(LED, OUTPUT);

  Serial.begin(BAUD); //Set Baud rate, don't set too high or risk data loss - might be unused for teensy needs testing
  if (!driver1.init())
      Serial.println("init 1 failed"); 
  else
      Serial.println("init 1 succeded");

  if (!driver2.init())
      Serial.println("init 2 failed"); 
  else
      Serial.println("init 2 succeded");

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
  pkt0[0] = (currentMillis >> 24) & 0xFF;
  pkt0[1] = (currentMillis >> 16) & 0xFF;
  pkt0[2] = (currentMillis >> 8) & 0xFF;
  pkt0[3] = currentMillis & 0xFF;

  switch (msg.id)
  {
    case 0x360:
      xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt0[4] = (xAccel >> 24) & 0xFF;
      pkt0[5] = (xAccel >> 16) & 0xFF;
      pkt0[6] = (xAccel >> 8) & 0xFF;
      pkt0[7] = xAccel & 0xFF;
      yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt0[8] = (yAccel >> 24) & 0xFF;
      pkt0[9] = (yAccel >> 16) & 0xFF;
      pkt0[10] = (yAccel >> 8) & 0xFF;
      pkt0[11] = yAccel & 0xFF;
      break;
    case 0x361:
      zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt0[12] = (zAccel >> 24) & 0xFF;
      pkt0[13] = (zAccel >> 16) & 0xFF;
      pkt0[14] = (zAccel >> 8) & 0xFF;
      pkt0[15] = zAccel & 0xFF;
      xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt0[16] = (xGyro >> 24) & 0xFF;
      pkt0[17] = (xGyro >> 16) & 0xFF;
      pkt0[18] = (xGyro >> 8) & 0xFF;
      pkt0[19] = xGyro & 0xFF;
      break;
    case 0x362:
      yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt0[20] = (yGyro >> 24) & 0xFF;
      pkt0[21] = (yGyro >> 16) & 0xFF;
      pkt0[22] = (yGyro >> 8) & 0xFF;
      pkt0[23] = yGyro & 0xFF;
      zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt0[24] = (zGyro >> 24) & 0xFF;
      pkt0[25] = (zGyro >> 16) & 0xFF;
      pkt0[26] = (zGyro >> 8) & 0xFF;
      pkt0[27] = zGyro & 0xFF;
      break;
    case 0x363:
      fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt0[28] = (fl_speed >> 8) & 0xFF;
      pkt0[29] = fl_speed & 0xFF;
      fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt0[30] = (fl_brakeTemp >> 8) & 0xFF;
      pkt0[31] = fl_brakeTemp & 0xFF;
      fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt0[32] = (fl_ambTemp >> 8) & 0xFF;
      pkt0[33] = fl_ambTemp & 0xFF;
      break;
    case 0x364:
      fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt0[34] = (fr_speed >> 8) & 0xFF;
      pkt0[35] = fr_speed & 0xFF;
      fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt0[36] = (fr_brakeTemp >> 8) & 0xFF;
      pkt0[37] = fr_brakeTemp & 0xFF;
      fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt0[38] = (fr_ambTemp >> 8) & 0xFF;
      pkt0[39] = fr_ambTemp & 0xFF;
      break;
    case 0x365:
      bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt0[40] = (bl_speed >> 8) & 0xFF;
      pkt0[41] = bl_speed & 0xFF;
      bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt0[42] = (bl_brakeTemp >> 8) & 0xFF;
      pkt0[43] = bl_brakeTemp & 0xFF;
      bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt0[44] = (bl_ambTemp >> 8) & 0xFF;
      pkt0[45] = bl_ambTemp & 0xFF;
      break;
    case 0x366:
      br_speed = (msg.buf[0]) | msg.buf[1] << 8;
      pkt0[46] = (br_speed >> 8) & 0xFF;
      pkt0[47] = br_speed & 0xFF;
      br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      pkt0[48] = (br_brakeTemp >> 8) & 0xFF;
      pkt0[49] = br_brakeTemp & 0xFF;
      br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;
      pkt0[50] = (br_ambTemp >> 8) & 0xFF;
      pkt0[51] = br_ambTemp & 0xFF;
      break;
    case 0x367:
      DRS = msg.buf[0];
      pkt0[52] = DRS ? 1 : 0;
      break;
    case 0x368:
      steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
      pkt0[53] = (steeringAngle << 8) & 0xFF;
      pkt0[54] = steeringAngle & 0xFF;
      throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
      pkt0[55] = (throttleInput << 8) & 0xFF;
      pkt0[56] = throttleInput & 0xFF;
      frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;
      pkt0[57] = (frontBrakePressure << 8) & 0xFF;
      pkt0[58] = frontBrakePressure & 0xFF;
      rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
      pkt0[59] = (rearBrakePressure << 8) & 0xFF;
      pkt0[60] = rearBrakePressure & 0xFF;
      break;
    case 0x369:
      gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt0[61] = (gps_lat >> 24) & 0xFF;
      pkt0[62] = (gps_lat >> 16) & 0xFF;
      pkt0[63] = (gps_lat >> 8) & 0xFF;
      pkt0[64] = gps_lat & 0xFF;
      gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt0[65] = (gps_long >> 24) & 0xFF;
      pkt0[66] = (gps_long >> 16) & 0xFF;
      pkt0[67] = (gps_long >> 8) & 0xFF;
      pkt0[68] = gps_long & 0xFF;
      break;
    case 0x36A:
      batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      pkt0[69] = (batteryVoltage >> 24) & 0xFF;
      pkt0[70] = (batteryVoltage >> 16) & 0xFF;
      pkt0[71] = (batteryVoltage >> 8) & 0xFF;
      pkt0[72] = batteryVoltage & 0xFF;
      daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];
      pkt0[73] = (batteryVoltage >> 24) & 0xFF;
      pkt0[74] = (batteryVoltage >> 16) & 0xFF;
      pkt0[75] = (batteryVoltage >> 8) & 0xFF;
      pkt0[76] = batteryVoltage & 0xFF;
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
  u_int16_t int16Test = rand();
  int intTest = rand();
  
  pkt0[0] = (timeTest >> 24) & 0xFF;
  pkt0[1] = (timeTest >> 16) & 0xFF;
  pkt0[2] = (timeTest >> 8) & 0xFF;
  pkt0[3] = timeTest & 0xFF;

  pkt0[4] = (intTest >> 24) & 0xFF;
  pkt0[5] = (intTest >> 16) & 0xFF;
  pkt0[6] = (intTest >> 8) & 0xFF;
  pkt0[7] = intTest & 0xFF;
  
  pkt0[28] = (int16Test >> 8) & 0xFF;
  pkt0[29] = int16Test & 0xFF;

  pkt0[30] = (shortTest >> 8) & 0xFF;
  pkt0[31] = shortTest & 0xFF;

  pkt0[52] = boolTest ? 1 : 0;

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
  bool sent_pkt1 = driver1.send(pkt0, sizeof(pkt0));
  bool sent_pkt2 = driver2.send(pkt1, sizeof(pkt1));

  driver1.waitPacketSent();
  driver2.waitPacketSent();
  
  if(Serial){
    Serial.println("Sent pkt1: " + String(sent_pkt1));
    Serial.println("Sent pkt2: " + String(sent_pkt2));
    Serial.println();
  }
}

