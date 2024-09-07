# RFM95-LoRa-Wireless-Comm
Sender and Receiver code for the RFM95 LoRa wireless communication.

## Pinout for Arduino Leonardo
Leonardo -----> LoRa

5v ----> VIN

ICSP 3 (SCK) ----> SCK

ICSP 4 (MOSI) ----> MOSI

ICSP 1 (MISO) ----> MISO

GND ----> GND

4 ----> (lora 0) CS

0 ----> (lora 0) G0

5 ----> (lora 1) CS

1 ----> (lora 1) G0

6 ----> (lora 2) CS

2 ----> (lora 2) G0

7 ----> (lora 3) CS

3 ----> (lora 3) G0

## LoRa Documentation
[RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab9605810c11c025758ea91b2813666e3)


## Receiver Spec Sheet
![Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Arduino_Leonardo_pinOUT.png)
![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Receiver_LoRa.jpg)
![Leonardo](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Leonardo.jpg)

## Pinout for Teensy 4.1
Teensy ----> LoRa

5v ----> (lora 0) VIN ----> (lora 0) VIN

13 ----> (lora 0) SCK ----> (lora 1) SCK

GND ----> (lora 0) GND ----> (lora 1) GND

38 ----> (lora 0) CS

37 ----> (lora 1) CS

36 ----> (lora 2) CS

10 ----> (lora 3) CS

21 ----> (lora 0) G0

20 ----> (lora 1) G0

19 ----> (lora 2) G0

18 ----> (lora 3) GO

11 ----> (lora 0) MOSI ----> (lora 1) MOSI ----> (lora 2) MOSI ----> (lora 3) MOSI

12 ----> (lora 0) MISO ----> (lora 1) MISO ----> (lora 2) MISO ----> (lora 3) MISO

CAN ----> 5v

CAN ----> GND

CAN HIGH ----> 23

CAN LOW ----> 22

## Sender Spec Sheet
![Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensy_4.1.PNG)
![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/lora_radios.PNG)
![Teensy](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensyPinout.jpg)
