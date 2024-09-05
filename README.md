# RFM95-LoRa-Wireless-Comm
Sender and Receiver code for the RFM95 LoRa wireless communication.

## Pinout for Arduino Leonardo
Leonardo -----> LoRa

5v ----> VIN

ICSP (SCK) ----> SCK

ICSP (MOSI) ----> MOSI

ICSP (MISO) ----> MISO

GND ----> GND

4 ----> CS

3 ---> G0

2 ---> RST

## LoRa Documentation
[RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab9605810c11c025758ea91b2813666e3)


## Receiver Spec Sheet
![Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Arduino_Leonardo_pinOUT.png)
![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Receiver_LoRa.jpg)
![Leonardo](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Receiver/Leonardo.jpg)

## Pinout for Teensy 4.1
Teensy ----> LoRa

5v ----> (lora 1) VIN ----> (lora 2) VIN

13 ----> (lora 1) SCK ----> (lora 2) SCK

GND ----> (lora 1) GND ----> (lora 2) GND

38 ----> (lora 1) CS

37 ----> (lora 2) CS

11 ----> (lora 1) MOSI ----> (lora 2) MOSI

12 ----> (lora 1) MISO ----> (lora 2) MISO

21 ----> (lora 1) G0

20 ----> (lora 2) G0

CAN ----> 5v

CAN ----> GND

CAN HIGH ----> 23

CAN LOW ----> 22

## Sender Spec Sheet
![Pinout](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensy_4.1.PNG)
![LoRa](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/lora_radios.PNG)
![Teensy](https://github.com/sundevilmotorsports/RFM95-LoRa-Wireless-Comm/blob/main/Spec_Sheet/Sender/teensyPinout.jpg)
