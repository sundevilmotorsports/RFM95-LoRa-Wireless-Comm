#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <iostream>
#include <chrono>

using namespace std;

#define CS0 38
#define G00 21

#define CS1 37
#define G01 20

#define CS2 36
#define G02 19

#define CS3 10
#define G03 18

#define SCK 13

#define TESTING false

uint8_t mode = 1;
int _delay;


//Declare packets
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
//Length Determined by offset (packet size) * number of groups being sent
uint8_t general[90];
//uint8_t suspension[126];
//uint8_t damper[234];
//uint8_t drive[74];
//uint8_t slide[150];

class packetMode {
  private:
    uint8_t currGroup = 0;
    uint8_t currOffset = 0;
    uint8_t *general_indexies;
  public:
    uint8_t latency;
    uint8_t indexies;
    uint8_t groupNum;
    uint8_t offset;
    vector<uint8_t> packet;
    packetMode(uint8_t offset, uint8_t groupNum, uint8_t latency,uint8_t *general_indexies, uint8_t indexies){
      this->offset = offset;
      this->groupNum = groupNum;
      this->indexies = indexies;
      this->latency = latency;
      this->general_indexies = general_indexies;
      this->packet.resize(offset * groupNum);
      };
  void update (){
    if ((general[0] << 24 | general[1] << 16 | general[2] << 8 | general[3]) > (packet[0 + currOffset] << 24 | packet[1 + currOffset] << 16 | packet[2 + currOffset] << 8 | packet[3 + currOffset]) + (this->latency/this->groupNum)) {
      if (currGroup + 1 == groupNum){
        this -> currGroup = 0;
      } else {
        this -> currGroup ++;
      }
      this -> currOffset = offset * currGroup;
      for(int i = 0; i < 4; i++){
        this -> packet[i + currOffset] = general[i];
      }
    }
    int index = currOffset + 4;
    for(uint8_t group = 0; group < indexies; group += 2){
      for(uint8_t general_idx = general_indexies[group]; general_idx <= general_indexies[group + 1]; general_idx++){
        this->packet[index] = general[general_idx];
        index++;
        //Serial.println("Group: " + String(group));
        //Serial.println("general value" + String(general[general_idx]));
      }
    }
    //Serial.println("Index: " + String(index));
  }
  void print_packet(){
    Serial.print("packet: " + String(this->packet[0]));
    for(uint8_t i = 1; i < this->groupNum * this->offset; i++){
      Serial.print(", " + String(packet[i]));
    }
    Serial.println();
  }
};

uint8_t susIndexies[12] = {4,29, 34,35, 40,41, 46,47, 52,70, 79,86};
uint8_t dampIndexies[6] = {4,15, 28,29, 79,86};
uint8_t driveIndexies[12] = {4,11, 28,29, 40,41, 46,47, 52,62, 79,86};
uint8_t slideIndexies[14] = {4,11, 28,29, 34,35, 40,41, 46,47, 52,53, 55,62};

packetMode suspension = packetMode(uint8_t(63), uint8_t(2), uint8_t(576), susIndexies, uint8_t(sizeof(susIndexies)));
packetMode damper = packetMode(uint8_t(26), uint8_t(9), uint8_t(981), dampIndexies, uint8_t(sizeof(dampIndexies)));
packetMode drive = packetMode(uint8_t(37), uint8_t(2), uint8_t(381), driveIndexies, uint8_t(sizeof(driveIndexies)));
packetMode slide = packetMode(uint8_t(30), uint8_t(5), uint8_t(666), slideIndexies, uint8_t(sizeof(slideIndexies)));

//Declaring radio and can
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

//uint8_t susOffset = 63;
uint8_t damperOffset = 26;
uint8_t driveOffset = 37;
uint8_t slideOffset = 30;

//uint8_t susGroupNum = 2;    
uint8_t damperGroupNum = 9; 
uint8_t driveGroupNum = 2;  
uint8_t slideGroupNum = 5;

//uint8_t currSus = 0;    
uint8_t currDamp = 0; 
uint8_t currDrive = 0;  
uint8_t currSlide = 0;

bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;

//indexes for general: !!!NOT UPDATED!!!
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

//Baud rate, don't set too high or risk data loss
unsigned long BAUD = 9600;

void setup() {
  switch (mode){
  case 0:
    _delay = 250;
    break;
  case 1:
    _delay = suspension.latency;
  case 2:
    _delay = damper.latency;
  case 3:
    _delay = drive.latency;
  case 4:
    _delay = slide.latency;
  default:
    break;
  }
  pinMode(SCK, OUTPUT);

  Serial.begin(BAUD); //Set Baud rate, don't set too high or risk data loss - might be unused for teensy needs testing
  radio1 = driver1.init();
  if (!radio1)
    Serial.println("init 1 failed"); 
  else {
    Serial.println("init 1 succeded");
    driver1.setFrequency(915.0); // Median of Hz range
    driver1.setTxPower(RH_RF95_MAX_POWER, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
    driver1.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
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
  else {
    Serial.println("init 4 succeded");
    driver4.setFrequency(915.0);
    driver4.setTxPower(RH_RF95_MAX_POWER, false);
    driver4.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048);
  }
  
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
int radio = 0;

void loop() {
  //throw invalid_argument( "received negative value");
  
  bool sent_pkt1 = false;
  bool sent_pkt2 = false;
  bool sent_pkt3 = false;
  bool sent_pkt4 = false;
  
  suspension.update();
  damper.update();
  drive.update();
  slide.update();
  //Serial.println("anti hang tech " + String(testing));
  //damper.print_packet();  
  switch (radio % 4){
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
          sent_pkt1 = driver1.send(suspension.packet.data(), suspension.groupNum * suspension.offset);
          break;
        case 2:
          sent_pkt1 = driver1.send(damper.packet.data(), damper.groupNum * damper.offset);
          break;
        case 3:
          sent_pkt1 = driver1.send(drive.packet.data(), drive.groupNum * drive.offset);
          break;
        case 4:
          sent_pkt1 = driver1.send(slide.packet.data(), slide.groupNum * slide.offset);
          break;
      }
      if (Serial){
        Serial.println("Sent radio 1: " + String(sent_pkt1) + "\tmode: " + String(mode));
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
          sent_pkt2 = driver2.send(suspension.packet.data(), suspension.groupNum * suspension.offset);
          break;
        case 2:
          sent_pkt2 = driver2.send(damper.packet.data(), damper.groupNum * damper.offset);
          break;
        case 3:
          sent_pkt2 = driver2.send(drive.packet.data(), drive.groupNum * drive.offset);
          break;
        case 4:
          sent_pkt2 = driver2.send(slide.packet.data(), slide.groupNum * slide.offset);
          break;         
      }
      if (Serial){
        Serial.println("Sent radio 2: " + String(sent_pkt2 * 2) + "\tmode: " + String(mode));
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
          sent_pkt3 = driver3.send(suspension.packet.data(), suspension.groupNum * suspension.offset);
          break;
        case 2:
          sent_pkt3 = driver3.send(damper.packet.data(), damper.groupNum * damper.offset);
          break;
        case 3:
          sent_pkt3 = driver3.send(drive.packet.data(), drive.groupNum * drive.offset);
          break;
        case 4:
          sent_pkt3 = driver3.send(slide.packet.data(), slide.groupNum * slide.offset);
      }
      if (Serial){
        Serial.println("Sent radio 3: " + String(sent_pkt3 * 3) + "\tmode: " + String(mode));
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
          driver4.send(suspension.packet.data(), suspension.groupNum * suspension.offset);
          break;
        case 2:
          sent_pkt4 = driver4.send(damper.packet.data(), damper.groupNum * damper.offset);
          break;
        case 3:
          sent_pkt4 = driver4.send(drive.packet.data(), drive.groupNum * drive.offset);
          break;
        case 4:
          sent_pkt4 = driver4.send(slide.packet.data(), slide.groupNum * slide.offset);
      }
      if (Serial){
        Serial.println("Sent radio 4: " + String(sent_pkt4 * 4) + "\tmode: " + String(mode));
      }
      break;
  }
  if(radio < 4){
    delay(_delay);
  }
  radio++;
}