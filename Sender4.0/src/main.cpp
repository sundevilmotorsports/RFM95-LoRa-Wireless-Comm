#include <Arduino.h>
#include <RH_RF95.h>
#include <FlexCAN_T4.h>
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

#define MODEM_CONFIG RH_RF95::ModemConfigChoice::Bw500Cr45Sf128

//Packet send time is a linear equation
//time = scalar based on config * number of indexies in the packet + base send time
  
  // Testing proved that message send length increases linearly with sent packet length

  // Equations are found by sending a packet of length 0 and using that as the base send time, 
  // find the send time of a full packet (251 indexies) find the delta between the base time, 
  // divide that by 251 and that is the scalar
std::map<RH_RF95::ModemConfigChoice, std::pair<float, int>> config = {
	{RH_RF95::ModemConfigChoice::Bw500Cr45Sf128,  {0.37,8}},    //short & fast,     0.37x + 5 ms
  {RH_RF95::ModemConfigChoice::Bw125Cr45Sf128,  {1.47,31}},   // medium & medium, 1.47x + 31 ms
	{RH_RF95::ModemConfigChoice::Bw125Cr45Sf2048, {15.01, 414}},// long & slow,     15.01x + 414 ms
	{RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512,{28.72,594}}, // long & sloww,    28.72x + 594 ms
	{RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096, {52.22,926}}  // long & slowwww,  52.22x + 926 ms
};

//Declare packet
//Max length 251 (RH_RF95_MAX_MESSAGE_LEN), longer the message the longer send time
//General is the master packet that is updated then by canSniff then parsed through to update other packets

uint8_t general[87];
uint8_t fake_general[251];

//Declaring LoRa instances, 1 for each radio
// from https://github.com/jecrespo/RadioHead
RH_RF95 driver1(CS0, G00);
RH_RF95 driver2(CS1, G01);
RH_RF95 driver3(CS2, G02);
RH_RF95 driver4(CS3, G03);

//Declaring CAN instance
// from https://github.com/tonton81/FlexCAN_T4
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

//stores if the radio wasn initialized
bool radio1 = false;
bool radio2 = false;
bool radio3 = false;
bool radio4 = false;

float num_radios = 0;

// Declare function
void canSniff(const CAN_message_t &msg);
void testPacket();

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
    // driver1.setModemConfig(MODEM_CONFIG); //Bandwidth of 125, Cognitive Radio 4/5, Spreading Factor 2048
    driver1.setSpreadingFactor(7);
    driver1.setSignalBandwidth(500000);
    driver1.setCodingRate4(6);
    num_radios++;
  }

  radio2 = driver2.init();
  if (!radio2)
    Serial.println("init 2 failed"); 
  else{
    Serial.println("init 2 succeded");
    driver2.setFrequency(915.0);
    driver2.setTxPower(RH_RF95_MAX_POWER, false);
    //driver2.setModemConfig(MODEM_CONFIG);
    driver2.setSpreadingFactor(7);
    driver2.setSignalBandwidth(500000);
    driver2.setCodingRate4(6);
    num_radios++;
  }

  radio3 = driver3.init();
  if (!radio3)
    Serial.println("init 3 failed"); 
  else{
    Serial.println("init 3 succeded");
    driver3.setFrequency(915.0);
    driver3.setTxPower(RH_RF95_MAX_POWER, false);
    //driver3.setModemConfig(MODEM_CONFIG);
    driver3.setSpreadingFactor(7);
    driver3.setSignalBandwidth(500000);
    driver3.setCodingRate4(6);
    num_radios++;
  }

  radio4 = driver4.init();
  if (!radio4)
    Serial.println("init 4 failed"); 
  else {
    Serial.println("init 4 succeded");
    driver4.setFrequency(915.0);
    driver4.setTxPower(RH_RF95_MAX_POWER, false);
    //driver4.setModemConfig(MODEM_CONFIG);
    driver4.setSpreadingFactor(7);
    driver4.setSignalBandwidth(500000);
    driver4.setCodingRate4(6);
    num_radios++;
  }
  
  Can.begin();
  Can.setBaudRate(1000000); // Our CAN loop's Baud rate
  Can.enableMBInterrupts(); // CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
  Can.onReceive(canSniff); // Calls can sniff when it recives a can message
}

//Counter that increments every loop, radio % 4 is what radio is chosen that loop and when radio is less than number of radios that initializes it sleeps the teensy for that delay 
int radio = 0;
int timing = 0;
void loop() {  
  bool sent_pkt1 = false;
  bool sent_pkt2 = false;
  bool sent_pkt3 = false;
  bool sent_pkt4 = false;

  // if(radio % 100000 == 0){
  //   Serial.println("anti-hang tech " + String(radio));
  // }

  if(TESTING_CAN){
    Serial.print("general: " + String(general[0]) + ", ");
    for(int i = 1; i < sizeof(general); i++){
      Serial.print(String(general[i]) + ", ");
    }
    Serial.println("");
  }

  
  switch (radio % 4){
    case  0:
      if(!radio1){
        //Serial.println("Driver 1 unresponsive");
        break;
      }
      sent_pkt1 = driver1.send(general, sizeof(general));
      if (Serial){
        Serial.println("Sent radio 1: " + String(sent_pkt1) + "\tmode: " + String(mode));
        int temp = millis();
        //Serial.println("\tA milli A milli A milli 1: " + String(temp - timing));
        timing = temp;
      }
      break;

    case 1:
      if(!radio2){
        //Serial.println("Driver 2 unresponsive");
        break;
      }
      sent_pkt2 = driver2.send(general, sizeof(general));
      if (Serial){
        Serial.println("Sent radio 2: " + String(sent_pkt2) + "\tmode: " + String(mode));
        int temp = millis();
        //Serial.println("\tA milli A milli A milli 2: " + String(temp - timing));
        timing = temp;
      }
      break;
    case 2:
      if(!radio3){
        //Serial.println("Driver 3 unresponsive");
        break;
      }
      sent_pkt3 = driver3.send(general, sizeof(general));
      if (Serial){
        Serial.println("Sent radio 3: " + String(sent_pkt3) + "\tmode: " + String(mode));
        int temp = millis();
        //Serial.println("\tA milli A milli A milli 3: " + String(temp - timing));
        timing = temp;
      }
      break;
    case 3:
      if(!radio4){
        //Serial.println("Driver 4 unresponsive");
        break;
      }
      sent_pkt4 = driver4.send(general, sizeof(general));
      if (Serial){
        Serial.println("Sent radio 4: " + String(sent_pkt4) + "\tmode: " + String(mode));
        int temp = millis();
        //Serial.println("\tA milli A milli A milli 4: " + String(temp - timing));
        timing = temp;
      }
      break;
  }
  
  if(radio < num_radios){
    delay(_delay);
  }
  radio++;
}

//The can messages are sent as a CAN messgae struct saved into msg, for us the important parts of the struct is
  // timestamp - the FlexCAN time when the message arrived - BETA: Was millis(), testing how it changes the response
  // buf - uint8_t array that holds our data
  // id - identifier that tells us what message we recived

void canSniff(const CAN_message_t &msg)
{
  //Serial.println("cansiff detect");
  //Grabing current millisecond, shifting right logicical to place the selected 8 bits into the 8 least significant bits,
  //then performing a logical and with 0xFF to mask all the more significant bits
  //Serial.println("In can");
  unsigned long currentMillis =  millis();
  general[0] = 0;
  general[1] = (currentMillis >> 24) & 0xFF;
  general[2] = (currentMillis >> 16) & 0xFF;
  general[3] = (currentMillis >> 8) & 0xFF;
  general[4] = currentMillis & 0xFF;

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
    case 0x2EE:
      //Will eventually DTC codes, not currently implemented
      break;
    case 0x360:
      //xAccel, yAccel
      // xAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // yAccel = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];  

      general[5] = msg.buf[0];
      general[6] = msg.buf[1];
      general[7] = msg.buf[2];
      general[8] = msg.buf[3];
      general[9] = msg.buf[4];
      general[10] = msg.buf[5];
      general[11] = msg.buf[6];
      general[12] = msg.buf[7];
      break;
    case 0x361:
      //zAccel, xGyro
      // zAccel = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // xGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[13] = msg.buf[0];
      general[14] = msg.buf[1];
      general[15] = msg.buf[2];
      general[16] = msg.buf[3];
      general[17] = msg.buf[4];
      general[18] = msg.buf[5];
      general[19] = msg.buf[6];
      general[20] = msg.buf[7];
      break;
    case 0x362:
      // yGyro, zGyro
      // yGyro = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // zGyro = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[21] = msg.buf[0];
      general[22] = msg.buf[1];
      general[23] = msg.buf[2];
      general[24] = msg.buf[3];
      general[25] = msg.buf[4];
      general[26] = msg.buf[5];
      general[27] = msg.buf[6];
      general[28] = msg.buf[7];
      break;
    case 0x363:
      //fl speed, fl temp, fl ambiant temp
      // fl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // fl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // fl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[29] = msg.buf[0];
      general[30] = msg.buf[1];
      general[31] = msg.buf[2];
      general[32] = msg.buf[3];
      general[33] = msg.buf[4];
      general[34] = msg.buf[5];
      break;
    case 0x364:
      //fr speed, fr temp, fr ambiant temp
      // fr_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // fr_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // fr_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[35] = msg.buf[0];
      general[36] = msg.buf[1];
      general[37] = msg.buf[2];
      general[38] = msg.buf[3];
      general[39] = msg.buf[4];
      general[40] = msg.buf[5];
      break;
    case 0x365:
      //rl speed, rl temp, rl ambiant temp
      // bl_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // bl_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // bl_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[41] = msg.buf[0];
      general[42] = msg.buf[1];
      general[43] = msg.buf[2];
      general[44] = msg.buf[3];
      general[45] = msg.buf[4];
      general[46] = msg.buf[5];
      break;
    case 0x366:
      //rr speed, rr temp, rr ambiant temp
      // br_speed = (msg.buf[0]) | msg.buf[1] << 8;
      // br_brakeTemp = (msg.buf[2]) | msg.buf[3] << 8;
      // br_ambTemp = (msg.buf[4]) | msg.buf[5] << 8;

      general[47] = msg.buf[0];
      general[48] = msg.buf[1];
      general[49] = msg.buf[2];
      general[50] = msg.buf[3];
      general[51] = msg.buf[4];
      general[52] = msg.buf[5];
      break;
    case 0x367:
      //DRS = msg.buf[0];

      general[55] = msg.buf[0] ? 1 : 0;
      break;
    case 0x368:
      // steeringAngle = (msg.buf[0]) | msg.buf[1] << 8;
      // throttleInput = (msg.buf[2]) | msg.buf[3] << 8;
      // rearBrakePressure = (msg.buf[6]) | msg.buf[7] << 8;
      // frontBrakePressure = (msg.buf[4]) | msg.buf[5] << 8;

      general[56] = msg.buf[0];
      general[57] = msg.buf[1];
      general[58] = msg.buf[2];
      general[59] = msg.buf[3];
      general[60] = msg.buf[4];
      general[61] = msg.buf[5];
      general[62] = msg.buf[6];
      general[63] = msg.buf[7];
      break;
    case 0x369:
      // gps_lat = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // gps_long = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[64] = msg.buf[0];
      general[65] = msg.buf[1];
      general[66] = msg.buf[2];
      general[67] = msg.buf[3];
      general[68] = msg.buf[4];
      general[69] = msg.buf[5];
      general[70] = msg.buf[6];
      general[71] = msg.buf[7];
      break;
    case 0x36A:
      // batteryVoltage = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
      // daqCurrentDraw = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

      general[72] = msg.buf[0];
      general[73] = msg.buf[1];
      general[74] = msg.buf[2];
      general[75] = msg.buf[3];
      general[76] = msg.buf[4];
      general[77] = msg.buf[5];
      general[78] = msg.buf[6];
      general[79] = msg.buf[7];
      break;
    default:
      Serial.print("default: " + String(msg.buf[0]) + ", ");
      for(int i = 1; i < msg.len; i++){
        Serial.print(String(msg.buf[i]) + ", ");
      }
    break;
  }
}