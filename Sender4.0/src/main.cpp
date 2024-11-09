#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <iostream>
#include <chrono>
#include <map>
#include <vector>

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

#define TESTING_CAN false
#define TESTING_RADIOS false

#define MODEM_CONFIG RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048

std::map<RH_RF95::ModemConfigChoice, std::pair<float, int>> config = {
	{RH_RF95::ModemConfigChoice::Bw500Cr45Sf128,  {0.37,8}},    //short & fast,     0.37x + 5 ms
  {RH_RF95::ModemConfigChoice::Bw125Cr45Sf128,  {1.47,31}},   // medium & medium, 1.47x + 31 ms
	{RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048, {15.01, 414}},// long & slow,     15.01x + 414 ms
	{RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512,{28.72,594}}, // long & sloww,    28.72x + 594 ms
	{RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096, {52.22,926}}  // long & slowwww,  52.22x + 926 ms
};

//Modes defined in Radio-Radio
//mode defs: 0 = general, 1 = suspension, 2 = damper, 3 = driver, 4 = slip/slide

uint8_t mode = 0;
uint16_t _delay;

//Declare packet
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
//General is the master packet that is updated then by canSniff then parsed through to update other packets
  //Testing determened send time was found with ((packet length * 0.015 * number of groupings)+0.414)/number of radios = latency in seconds from start to recive time

uint8_t general[86];
uint8_t fake_general[251];

class packetMode {
  private:
    uint8_t currGroup = 0;
    uint8_t currOffset = 0;
    uint8_t *general_indexies;
  public:
    uint16_t latency;
    uint8_t indexies;
    uint8_t groupNum;
    uint8_t offset;
    vector<uint8_t> packet;
    packetMode(uint8_t offset, uint8_t groupNum, uint16_t latency,uint8_t *general_indexies, uint8_t indexies){
      this->offset = offset;
      this->groupNum = groupNum;
      this->indexies = indexies;
      this->latency = latency;
      this->general_indexies = general_indexies;
      this->packet.resize(offset * groupNum);
      };

  //when called, update first compares the previous timestamp to the timestamp stored in general, and if it is over the estimated latency / the number of groups then it beings storing data in the next group
    //To compare timestamps it grabs there indexies, bit shifts them the appropriate ammount for their data type and performs logical or to get the final number
  //Next is iterating through all of the indexies in the general packet, odd indexes acting as a start index and even indexes acting as an end index, and then saving them into the coresponding position in the child packet
  void update (){
    int tempGroup;
    if(currGroup - 1 < 0){
      tempGroup = groupNum - 1;
    } else {
      tempGroup = currGroup - 1;
    }
    int tempOffset = offset * tempGroup;
    if ((general[0] << 24 | general[1] << 16 | general[2] << 8 | general[3]) > (packet[0 + tempOffset] << 24 | packet[1 + tempOffset] << 16 | packet[2 + tempOffset] << 8 | packet[3 + tempOffset]) + (this->latency/this->groupNum)) {
      if (currGroup + 1 == groupNum){
        this -> currGroup = 0;
      } else {
        this -> currGroup ++;
      }
      this -> currOffset = offset * currGroup;
    }
    int index = currOffset;
    for(uint8_t group = 0; group < indexies; group += 2){
      for(uint8_t general_idx = general_indexies[group]; general_idx <= general_indexies[group + 1]; general_idx++){
        this->packet[index] = general[general_idx];
        index++;
      }
    }
  }
  void print_packet(){
    Serial.print("packet: \nGroup 1: " + String(this->packet[0]) + ", ");
    for(uint8_t i = 1; i < this->groupNum * this->offset; i++){
      if(i %this->offset == 0){
        Serial.print("\nGroup " + String((i/offset) + 1) + ": ");
      }
      Serial.print(String(packet[i]) + ", ");
    }
    Serial.println();
  }
};

//Indexies within general that are being parced
uint8_t susIndexies[12] = {0,29, 34,35, 40,41, 46,47, 52,70, 79,86};
uint8_t dampIndexies[6] = {0,15, 28,29, 79,86};
uint8_t driveIndexies[12] = {0,11, 28,29, 40,41, 46,47, 52,62, 79,86};
uint8_t slideIndexies[14] = {0,11, 28,29, 34,35, 40,41, 46,47, 52,53, 55,62};

//declaration of packetMode variables
  //inputs offset(# indexies in each group), # of groups, desired latency in ms, indexies within general that are being parced, length of general indexies
packetMode suspension = packetMode(uint8_t(63), uint8_t(2), uint16_t(576), susIndexies, uint8_t(sizeof(susIndexies)));
packetMode damper = packetMode(uint8_t(26), uint8_t(9), uint16_t(981), dampIndexies, uint8_t(sizeof(dampIndexies)));
packetMode drive = packetMode(uint8_t(37), uint8_t(2), uint16_t(381), driveIndexies, uint8_t(sizeof(driveIndexies)));
packetMode slide = packetMode(uint8_t(30), uint8_t(5), uint16_t(666), slideIndexies, uint8_t(sizeof(slideIndexies)));

//Declaring LoRa instances, 1 for each radio
// from https://github.com/jecrespo/RadioHead
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);

//Declaring radio and can
// from https://github.com/tonton81/FlexCAN_T4
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

//stores if the radio wasn initialized
bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;

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
int fl_shock = -1;
int fr_shock = -1;
int rl_shock = -1;
int rr_shock = -1;

float num_radios = 0;

// Declare function
void canSniff(const CAN_message_t &msg);

unsigned long BAUD = 9600;
void setup() {

  //pinMode defines what pin you want an output to be, ie SCK (LED pin) is sent a signal when OUTPUT happens (radios send a packet)
  pinMode(SCK, OUTPUT);

  Serial.begin(BAUD);
  
  //attempt to initialize each radio, if successful then set the frequency median to 915 Hz, max power, and the modem config for our usecase
    //frequency isn't exact as it is dynamically tuned to open frequencies around that range
  radio1 = driver1.init();
  if (!radio1)
    Serial.println("init 1 failed"); 
  else {
    Serial.println("init 1 succeded");
    driver1.setFrequency(915.0); // Median of Hz range
    driver1.setTxPower(RH_RF95_MAX_POWER, false); //Max power, should increase range, but try to find min because a little rude to be blasting to everyone
    //driver1.setModemConfig(MODEM_CONFIG); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
    driver1.setSpreadingFactor(8);
    driver1.setSignalBandwidth(125000);
    driver1.setCodingRate4(5);
    num_radios++;
  }

  radio2 = driver2.init();
  if (!radio2)
    Serial.println("init 2 failed"); 
  else{
    Serial.println("init 2 succeded");
    driver2.setFrequency(915.0);
    driver2.setTxPower(RH_RF95_MAX_POWER, false);
    driver2.setModemConfig(MODEM_CONFIG);
    driver2.setSpreadingFactor(8);
    driver2.setSignalBandwidth(125000);
    driver2.setCodingRate4(5);
    num_radios++;
  }

  radio3 = driver3.init();
  if (!radio3)
    Serial.println("init 3 failed"); 
  else{
    Serial.println("init 3 succeded");
    driver3.setFrequency(915.0);
    driver3.setTxPower(RH_RF95_MAX_POWER, false);
    driver3.setModemConfig(MODEM_CONFIG);
    driver3.setSpreadingFactor(8);
    driver3.setSignalBandwidth(125000);
    driver3.setCodingRate4(5);
    num_radios++;
  }

  radio4 = driver4.init();
  if (!radio4)
    Serial.println("init 4 failed"); 
  else {
    Serial.println("init 4 succeded");
    driver4.setFrequency(915.0);
    driver4.setTxPower(RH_RF95_MAX_POWER, false);
    driver4.setModemConfig(MODEM_CONFIG);
    driver4.setSpreadingFactor(8);
    driver4.setSignalBandwidth(125000);
    driver4.setCodingRate4(5);
    num_radios++;
  }

  switch (mode){
  case 0:
    _delay = 0;
    break;
  case 1:
    _delay = suspension.latency;
    break;
  case 2:
    _delay = damper.latency;
    break;
  case 3:
    _delay = drive.latency;
    break;
  case 4:
    _delay = slide.latency;
    break;
  default:
    break;
  }
  
  Can.begin();
  Can.setBaudRate(1000000); // Our CAN loop's Baud rate
  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  //Can.onReceive(canSniff); // Calls can sniff when it recives a can message
}

//The can messages are sent as a CAN messgae struct saved into msg, for us the important parts of the struct is
  // timestamp - the FlexCAN time when the message arrived - BETA: Was millis(), testing how it changes the response
  // buf - uint8_t array that holds our data
  // id - identifier that tells us what message we recived

void canSniff(const CAN_message_t &msg)
{ 
  //Grabing current millisecond, shifting right logicical to place the selected 8 bits into the 8 least significant bits,
    //then performing a logical and with 0xFF to mask all the more significant bits
  //Serial.println("In can");
  unsigned long currentMillis =  millis();
  general[0] = (currentMillis >> 24) & 0xFF;
  general[1] = (currentMillis >> 16) & 0xFF;
  general[2] = (currentMillis >> 8) & 0xFF;
  general[3] = currentMillis & 0xFF;

  //Potential msg.id values
  // 0x360 = 864 = (imu) xAccel, yAccel = general 4 - 11
  // 0x361 = 865 = (imu) zAccel, xGyro = general 12 - 19
  // 0x362 = 866 = (imu) yGyro, zGyro = general 20 - 27

  // 0x363 = 867 = (fl_wheel) fl speed, fl temp, fl amiant temp = general 28 - 33
  // 0x364 = 868 = (fr_wheel) fr speed, fr temp, fr amiant temp = general 34 - 39
  // 0x365 = 869 = (rl_wheel) rl speed, rl temp, rl amiant temp = general 40 - 45
  // 0x366 = 870 = (rr_wheel) rr speed, rr temp, rr amiant temp = general 46 - 51
  // undef = undef = (diff_wheel) diff_speed = general = 52 - 53

  // 0x367 = 871 = (DRS) DRS open/cloosed = general 54

  // 0x368 = 872 = (Datalog) throttle angle, throttle input, front brake pressure, rear brake pressure = general 55 - 62
  // 0x369 = 873 = (Datalog) gps lattitude, gps longitude = general 63 - 70
  // 0x36A = 874 = (Datalog) battery voltage, DAQ current draw = 71 - 78

  // undef = undef = (Datalog) fl shock pot, fr shock pot, rl shock pot, rr shock pot = general 79 - 86

  // Shouldn't need to bit shift since it is passed through buf as a uint8_t array, keeping code incase it breaks

  switch (msg.id){
    case 0x360:
      //xAccel, yAccel
      // xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];  

      general[4] = msg.buf[0];
      general[5] = msg.buf[1];
      general[6] = msg.buf[2];
      general[7] = msg.buf[3];
      general[8] = msg.buf[4];
      general[9] = msg.buf[5];
      general[10] = msg.buf[6];
      general[11] = msg.buf[7];
      break;
    case 0x361:
      //zAccel, xGyro
      // zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[12] = msg.buf[0];
      general[13] = msg.buf[1];
      general[14] = msg.buf[2];
      general[15] = msg.buf[3];
      general[16] = msg.buf[4];
      general[17] = msg.buf[5];
      general[18] = msg.buf[6];
      general[19] = msg.buf[7];
      break;
    case 0x362:
      // yGyro, zGyro
      // yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[20] = msg.buf[0];
      general[21] = msg.buf[1];
      general[22] = msg.buf[2];
      general[23] = msg.buf[3];
      general[24] = msg.buf[4];
      general[25] = msg.buf[5];
      general[26] = msg.buf[6];
      general[27] = msg.buf[7];
      break;
    case 0x363:
      //fl speed, fl temp, fl ambiant temp
      // fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[28] = msg.buf[0];
      general[29] = msg.buf[1];
      general[30] = msg.buf[2];
      general[31] = msg.buf[3];
      general[32] = msg.buf[4];
      general[33] = msg.buf[5];
      break;
    case 0x364:
      //fr speed, fr temp, fr ambiant temp
      // fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[34] = msg.buf[0];
      general[35] = msg.buf[1];
      general[36] = msg.buf[2];
      general[37] = msg.buf[3];
      general[38] = msg.buf[4];
      general[39] = msg.buf[5];
      break;
    case 0x365:
      //rl speed, rl temp, rl ambiant temp
      // bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[40] = msg.buf[0];
      general[41] = msg.buf[1];
      general[42] = msg.buf[2];
      general[43] = msg.buf[3];
      general[44] = msg.buf[4];
      general[45] = msg.buf[5];
      break;
    case 0x366:
      //rr speed, rr temp, rr ambiant temp
      // br_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[46] = msg.buf[0];
      general[47] = msg.buf[1];
      general[48] = msg.buf[2];
      general[49] = msg.buf[3];
      general[50] = msg.buf[4];
      general[51] = msg.buf[5];
      break;
    case 0x367:
      //DRS = msg.buf[0];

      general[54] = msg.buf[0] ? 1 : 0;
      break;
    case 0x368:
      // steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
      // throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
      // rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
      // frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;

      general[55] = msg.buf[0];
      general[56] = msg.buf[1];
      general[57] = msg.buf[2];
      general[58] = msg.buf[3];
      general[59] = msg.buf[4];
      general[60] = msg.buf[5];
      general[61] = msg.buf[6];
      general[62] = msg.buf[7];
      break;
    case 0x369:
      // gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[63] = msg.buf[0];
      general[64] = msg.buf[1];
      general[65] = msg.buf[2];
      general[66] = msg.buf[3];
      general[67] = msg.buf[4];
      general[68] = msg.buf[5];
      general[69] = msg.buf[6];
      general[70] = msg.buf[7];
      break;
    case 0x36A:
      // batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[71] = msg.buf[0];
      general[72] = msg.buf[1];
      general[73] = msg.buf[2];
      general[74] = msg.buf[3];
      general[75] = msg.buf[4];
      general[76] = msg.buf[5];
      general[77] = msg.buf[6];
      general[78] = msg.buf[7];
      break;
    default:
      Serial.print("default: " + String(msg.buf[0]) + ", ");
      for(int i = 1; i < msg.len; i++){
        Serial.print(String(msg.buf[i]) + ", ");
      }
    break;
  }
}

void testPacket(){  
  int currentMillis = millis();
  general[0] = (currentMillis >> 24) & 0xFF;
  general[1] = (currentMillis >> 16) & 0xFF;
  general[2] = (currentMillis >> 8) & 0xFF;
  general[3] = currentMillis & 0xFF;

  xAccel = (rand() % 201) - (200 / 2);
  general[4] = (xAccel >> 24) & 0xFF;
  general[5] = (xAccel >> 16) & 0xFF;
  general[6] = (xAccel >> 8) & 0xFF;
  general[7] = xAccel & 0xFF;

  yAccel = (rand() % 201) - (200 / 2);
  general[8] = (yAccel >> 24) & 0xFF;
  general[9] = (yAccel >> 16) & 0xFF;
  general[10] = (yAccel >> 8) & 0xFF;
  general[11] = yAccel & 0xFF;

  zAccel = (rand() % 201) - (200 / 2);
  general[12] = (zAccel >> 24) & 0xFF;
  general[13] = (zAccel >> 16) & 0xFF;
  general[14] = (zAccel >> 8) & 0xFF;
  general[15] = zAccel & 0xFF;

  xGyro = (rand() % 201) - (200 / 2);
  general[16] = (xGyro >> 24) & 0xFF;
  general[17] = (xGyro >> 16) & 0xFF;
  general[18] = (xGyro >> 8) & 0xFF;
  general[19] = xGyro & 0xFF;
  
  yGyro = (rand() % 201) - (200 / 2);
  general[20] = (yGyro >> 24) & 0xFF;
  general[21] = (yGyro >> 16) & 0xFF;
  general[22] = (yGyro >> 8) & 0xFF;
  general[23] = yGyro & 0xFF;
  
  zGyro = (rand() % 201) - (200 / 2);
  general[24] = (zGyro >> 24) & 0xFF;
  general[25] = (zGyro >> 16) & 0xFF;
  general[26] = (zGyro >> 8) & 0xFF;
  general[27] = zGyro & 0xFF;

  fl_speed = (rand() % 201) - (200 / 2);
  general[28] = (fl_speed >> 8) & 0xFF;
  general[29] = fl_speed & 0xFF;

  fl_brakeTemp = (rand() % 201) - (200 / 2);
  general[30] = (fl_brakeTemp >> 8) & 0xFF;
  general[31] = fl_brakeTemp & 0xFF;

  fl_ambTemp = (rand() % 201) - (200 / 2);
  general[32] = (fl_ambTemp >> 8) & 0xFF;
  general[33] = fl_ambTemp & 0xFF;

  fl_ambTemp = (rand() % 201) - (200 / 2);
  general[34] = (fr_speed >> 8) & 0xFF;
  general[35] = fr_speed & 0xFF;
  
  fr_brakeTemp = (rand() % 201) - (200 / 2);
  general[36] = (fr_brakeTemp >> 8) & 0xFF;
  general[37] = fr_brakeTemp & 0xFF;
  
  fr_ambTemp = (rand() % 201) - (200 / 2);
  general[38] = (fr_ambTemp >> 8) & 0xFF;
  general[39] = fr_ambTemp & 0xFF;

  bl_speed = (rand() % 201) - (200 / 2);
  general[40] = (bl_speed >> 8) & 0xFF;
  general[41] = bl_speed & 0xFF;
  
  bl_brakeTemp = (rand() % 201) - (200 / 2);
  general[42] = (bl_brakeTemp >> 8) & 0xFF;
  general[43] = bl_brakeTemp & 0xFF;
  
  bl_ambTemp = (rand() % 201) - (200 / 2);
  general[44] = (bl_ambTemp >> 8) & 0xFF;
  general[45] = bl_ambTemp & 0xFF;

  br_speed = (rand() % 201) - (200 / 2);
  general[46] = (br_speed >> 8) & 0xFF;
  general[47] = br_speed & 0xFF;
  
  br_brakeTemp = (rand() % 201) - (200 / 2);
  general[48] = (br_brakeTemp >> 8) & 0xFF;
  general[49] = br_brakeTemp & 0xFF;
  
  br_ambTemp = (rand() % 201) - (200 / 2);
  general[50] = (br_ambTemp >> 8) & 0xFF;
  general[51] = br_ambTemp & 0xFF;

  DRS = rand() % 2;
  general[54] = DRS ? 1 : 0;

  steeringAngle = (rand() % 201) - (200 / 2);
  general[55] = (steeringAngle << 8) & 0xFF;
  general[56] = steeringAngle & 0xFF;
  
  throttleInput = (rand() % 201) - (200 / 2);
  general[57] = (throttleInput << 8) & 0xFF;
  general[58] = throttleInput & 0xFF;
  
  frontBrakePressure = (rand() % 201) - (200 / 2);
  general[59] = (frontBrakePressure << 8) & 0xFF;
  general[60] = frontBrakePressure & 0xFF;
  
  rearBrakePressure = (rand() % 201) - (200 / 2);
  general[61] = (rearBrakePressure << 8) & 0xFF;
  general[62] = rearBrakePressure & 0xFF;

  gps_lat = (rand() % 201) - (200 / 2);
  general[63] = (gps_lat >> 24) & 0xFF;
  general[64] = (gps_lat >> 16) & 0xFF;
  general[65] = (gps_lat >> 8) & 0xFF;
  general[66] = gps_lat & 0xFF;
  
  gps_long = (rand() % 201) - (200 / 2);
  general[67] = (gps_long >> 24) & 0xFF;
  general[68] = (gps_long >> 16) & 0xFF;
  general[69] = (gps_long >> 8) & 0xFF;
  general[70] = gps_long & 0xFF;
  
  batteryVoltage = (rand() % 201) - (200 / 2);
  general[71] = (batteryVoltage >> 24) & 0xFF;
  general[72] = (batteryVoltage >> 16) & 0xFF;
  general[73] = (batteryVoltage >> 8) & 0xFF;
  general[74] = batteryVoltage & 0xFF;
  
  daqCurrentDraw = (rand() % 201) - (200 / 2);
  general[75] = (batteryVoltage >> 24) & 0xFF;
  general[76] = (batteryVoltage >> 16) & 0xFF;
  general[77] = (batteryVoltage >> 8) & 0xFF;
  general[78] = batteryVoltage & 0xFF;

  fl_shock = (rand() % 201) - (200 / 2);
  general[79] = (fl_shock << 8) & 0xFF;
  general[80] = fl_shock & 0xFF;
  
  fr_shock = (rand() % 201) - (200 / 2);
  general[81] = (fr_shock << 8) & 0xFF;
  general[82] = fr_shock & 0xFF;
  
  rl_shock = (rand() % 201) - (200 / 2);
  general[83] = (rl_shock << 8) & 0xFF;
  general[84] = rl_shock & 0xFF;

  if(Serial){
    Serial.println("Received Data Log: " + String(currentMillis) + "ms"
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

//Counter that increments every loop, radio % 4 is what radio is chosen that loop and when radio is less than number of radios that initializes it sleeps the teensy for that delay 
int radio = 0;

void loop() {  
  bool sent_pkt1 = false;
  bool sent_pkt2 = false;
  bool sent_pkt3 = false;
  bool sent_pkt4 = false;

  if (TESTING_RADIOS){
    testPacket();
  }

  if(radio % 100000 == 0){
    //Serial.println("anti-hang tech " + String(radio));
  }

  switch (mode){
    case 1:{
      suspension.update();
      break;
    }
    case 2:{
      damper.update();
      break;
    }
    case 3:{
      drive.update();
      break;
    }
    case 4:{
      slide.update();
      break;
    }
  }
  if(TESTING_CAN){
    switch (mode){
      case 1:{
        suspension.print_packet();
        break;
      }
      case 2:{
        damper.print_packet();
        break;
      }
      case 3:{
        drive.print_packet();
        break;
      }
      case 4:{
        slide.print_packet();
        break;
      }
    }
  }

  
  switch (radio % 4){
    case  0:
      if(!radio1){
        //Serial.println("Driver 1 unresponsive");
        break;
      }
      switch(mode){
        case 0:
          sent_pkt1 = driver1.send(fake_general, sizeof(fake_general));
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
        Serial.println("Sent radio 2: " + String(sent_pkt2) + "\tmode: " + String(mode));
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
        Serial.println("Sent radio 3: " + String(sent_pkt3) + "\tmode: " + String(mode));
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
          sent_pkt4 = driver4.send(suspension.packet.data(), suspension.groupNum * suspension.offset);
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
        Serial.println("Sent radio 4: " + String(sent_pkt4) + "\tmode: " + String(mode));
      }
      break;
  }
  
  if(radio < num_radios){
    delay(_delay);
  }
  radio++;
}