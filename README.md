# RFM95-LoRa-Wireless-Comm
Sender and Receiver code for the RFM95 LoRa wireless communication.

## LoRa Documentation
[RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html)

## Heads up
If wanting to use more than 3 radios you have to modify 

lib\RadioHead-1.130\RadioHead\{radio class}.cpp 

and 

lib\RadioHead-1.130\RadioHead\{radio class}.h

Also, you might have to find where it is saved locally on your system and modify it there, I haven't been able to test if that is required

  In {radio class}.cpp file you add another index to _deviceForInterrupt, where N is the number after the previous, and add
  ```cpp
  else if(_myInterruptIndex == N)
    attachInterrupt(interruptNumber, isrN, RISING);
  ```
  ```cpp
  void RH_INTERRUPT_ATTR RH_RF95::isrN()
  {
      if (_deviceForInterrupt[N])
  	_deviceForInterrupt[N]->handleInterrupt();
  }
  ```
  In {radio class}.h declare and add 1 to RH_RF95_NUM_INTERRUPTS
  ```cpp
  #define RH_RF95_NUM_INTERRUPTS = N
  static void         isrN();
  ```
Supposedly they had it set up like this because most Arduinos can only handle 2 and MEGAs can only handle 3, but we move different

## Pinout for Arduino Leonardo

**!!!Leonardo is not currently on a board!!!**

**CS pins MUST go into INT pins (i.e 0-4, 7), PCINT didn't work in testing but we're also somewhat dumb and might work now**

Leonardo ----> LoRa 0 ----> LoRa 1 ----> LoRa 2 ----> LoRa 3

5v ----> VIN

ICSP 3 (SCK) ----> SCK

ICSP 4 (MOSI) ----> MOSI

ICSP 1 (MISO) ----> MISO

GND ----> GND

0 ----> (LoRa 0) G0

1 ----> (LoRa 1) G0

2 ----> (LoRa 2) G0

3 ----> (LoRa 3) G0

4 ----> (LoRa 0) CS

5 ----> (LoRa 1) CS

6 ----> (LoRa 2) CS

7 ----> (LoRa 3) CS

## Receiver Spec Sheet
![Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Arduino_Loenardo_pinOUT2.png)
<!--![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Receiver_LoRa.jpg)-->
<!--![Leonardo](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Leonardo.jpg)-->

## Pinout for Teensy 4.1 on the breadboard
-- CS pins can be on any logic pin, currently using pins defined as CS pins by Teensy
-- Chained pins can theoretically go infinitely, just limited by logic pins but [check warning above]([##-Heads-up]) to implement in code

Teensy ----> LoRa 0 ----> LoRa 1 ----> LoRa 2 ----> LoRa 3

5v ----> power rail ----> (LoRa 0) VIN ----> (LoRa 1) VIN ----> (LoRa 2) VIN ----> (LoRa 3) VIN

13 ----> (LoRa 0) SCK ----> (LoRa 1) SCK ----> (LoRa 2) SCK ----> (LoRa 3) SCK

GND ----> (LoRa 0) GND ----> (LoRa 1) GND ----> (LoRa 2) GND ----> (LoRa 3) GND

38 ----> (LoRa 0) CS

37 ----> (LoRa 1) CS

36 ----> (LoRa 2) CS

10 ----> (LoRa 3) CS

21 ----> (LoRa 0) G0

20 ----> (LoRa 1) G0

19 ----> (LoRa 2) G0

18 ----> (LoRa 3) GO

11 ----> (LoRa 0) MOSI ----> (LoRa 1) MOSI ----> (LoRa 2) MOSI ----> (LoRa 3) MOSI

12 ----> (LoRa 0) MISO ----> (LoRa 1) MISO ----> (LoRa 2) MISO ----> (LoRa 3) MISO

CAN ----> 5v !!!Only connect when powering through USB, not the car!!!

CAN ----> GND

CAN HIGH ----> 23

CAN LOW ----> 22

## Sender Spec Sheet
<!--![Teensy](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensy_4.1.PNG)-->
<!--![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/lora_radios.PNG)-->
![Simple Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensyPinout.jpg)
![In-depth Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensyPinout2.jpg)
