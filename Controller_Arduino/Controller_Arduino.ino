#include <NewTone.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SoftwareSerial.h>
#include "RTClib.h"

SoftwareSerial CANSerial(3, 2); // RX, TX
RTC_DS1307 rtc;

tCAN setMessage(int id, int headerLength, ...) {
  va_list argList;
  va_start(argList, headerLength);
  tCAN message;
  
  for (int i = 0; i < headerLength; i++) {
    int value = va_arg(argList, int);
    message.data[i] = value;
  }
  
  message.id = id;
  message.header.rtr = 0;
  message.header.length = headerLength;
  
  return message;
}

void sendMsg(tCAN message) {
  String arduinoData = "S ";
  String id = String(message.id, HEX);
  id.toUpperCase();
  arduinoData.concat(id);
  arduinoData.concat(" ");
  for (int i = 0; i < message.header.length; i++) {
    String data = String(message.data[i], HEX);
    data.toUpperCase();
    arduinoData.concat(data);
    if (i != message.header.length - 1) {
      arduinoData.concat(" ");
    }
  }
  CANSerial.print(arduinoData);
  while (!CANSerial.available()) { }
  while (CANSerial.available()) {
    char in = CANSerial.read();
    delay(2);
  }
}

void sendToArduino(String send) {
  CANSerial.print(send);
  while (!CANSerial.available()) {}
  while (CANSerial.available()) {
    CANSerial.read();
    delay(2);
  }
  Serial.println(send);
}

// splits the given string by the given separator and sends the split string to the given destination String array
void split(String src, char sep, String dest[]) { 
  int arrIdx = 0; // used to keep track of how many strings have been split from the source
  int strIndex[] = {0, -1}; // used to keep track of where to split the string {start, end}
  int maxIndex = src.length() - 1;  // records the highest index of the source string (avoids out-of-bounds)
  int maxSize = sizeof(&dest) / sizeof(String);  // records the size of the destination array
  
  for (int i = 0; i <= maxIndex; i++){
    if (src.charAt(i) == sep || i == maxIndex && arrIdx != maxSize) {
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
      dest[arrIdx++] = src.substring(strIndex[0], strIndex[1]);
    }
  }
}

int hexCharToInt(char c) {
  if (c >= 0x30 && c <= 0x39) {
    return c - 0x30;
  } else if (c >= 'A' && c <= 'F') {  // 0x41 to 0x46
    return 15 - (70 - c);
  }
}

tCAN msg0x23  = setMessage(0x23, 8, 0x00, 0x00, 0x80, 0x00, 0xFA, 0xFA, 0x02, 0x54); //0x88
tCAN transMsg = setMessage(0x22, 8, 0x00, 0x00, 0x04, 0x25, 0x00, 0x00, 0x00, 0x00);
int secondsInterval = 15;
void setup() {
  Serial.begin(9600);
  CANSerial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1) {}
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC is not running");
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //current time from computer
  //rtc.adjust(DateTime(2019, 1, 8, 17, 15, 20)); // year, month, day, hour (24), minute, second

  DateTime now = rtc.now();
  int seconds = now.second();
  msg0x23.data[7] += seconds;
  if (seconds <= 15) {
    secondsInterval = 15;
    msg0x23.data[7] += 1;
  } else if (seconds <= 30) {
    secondsInterval = 30;
    msg0x23.data[7] += 2;
  } else if (seconds <= 45) {
    secondsInterval = 45;
    msg0x23.data[7] += 3;
  }
  sendMsg(msg0x23);
  Serial.println("Setup done");

  //sendToArduino("speed 1000");
}

int calcSpeedoFreq(int minutes, int lessFreq, int lessInc, int greaterFreq, int greaterInc) {
  if (minutes < 30) {
    return lessFreq + (minutes / lessInc);
  } else {
    return greaterFreq + ((minutes - 30) / lessInc);
  }
}

int charToInt(String src, int sLength, int from, int to, int base) {
  char s[sLength];
  char *pEnd;
  src.substring(from, to).toCharArray(s, sLength);
  return strtol(s, &pEnd, base);
}

int oldSec = -1;
int oldMin = -1;
bool upArrowLit = false;
bool dnArrowLit = false;
bool brightMode = false;
bool trunkOn = false;
void loop() {
  DateTime now = rtc.now();

  if (Serial.available()) {
    String data = "";
    while (Serial.available()) {
      char in = Serial.read();
      data.concat(in);
      delay(2);
    }
    String splitData[2];
    split(data, ' ', splitData);
    if (splitData[0] == "brightness") {
      msg0x23.data[4] = splitData[1].toInt();
      sendMsg(msg0x23);
    } else if (splitData[0] == "bright") {
      int onOff = splitData[1].toInt();
      if (!brightMode && onOff == 1) {
        msg0x23.data[3] += 0x20;
        brightMode = true;
      } else if (brightMode && onOff == 0) {
        msg0x23.data[3] -= 0x20;
        brightMode = false;
      }
      sendMsg(msg0x23);
    } else if (splitData[0] == "trunk") {
      int onOff = splitData[1].toInt();
      if (!trunkOn && onOff == 1) {
        msg0x23.data[3] += 0x10;
        trunkOn = true;
      } else if (trunkOn && onOff == 0) {
        msg0x23.data[3] -= 0x10;
        trunkOn = false;
      }
      sendMsg(msg0x23);
    }
  }
  
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  Serial.println(" ");
  if (oldSec == -1) {
    oldSec = now.second();
  }

  while (rtc.now().second() == oldSec) {}
  now = rtc.now();
  int newS = now.second();

  msg0x23.data[7] += newS - oldSec;
  sendMsg(msg0x23);
  if (msg0x23.data[7] == 0x7C) {
    msg0x23.data[7] = 0x90;
  }
  if (newS >= secondsInterval) {
    msg0x23.data[7] += 1;
    secondsInterval += 15;
  }
  oldSec = now.second();

  String hex = String(transMsg.data[1], HEX);
  if (oldSec % 2 == 0) {
    upArrowLit = false;
    dnArrowLit = true;
  } else {
    upArrowLit = true;
    dnArrowLit = false;
  }
  int tenSec = oldSec / 10;
  if (upArrowLit) {
    transMsg.data[1] = (tenSec == 0 ? 0x0F : tenSec + 0x08);
  } else {
    transMsg.data[1] = (tenSec == 0 ? 0x17 : tenSec + 0x10);
  }
  String min = String(now.minute(), DEC);
  if (min[1] == '0' || min[1] == '1') {
    transMsg.data[2] = 0x03;
  } else if (min[1] == '2' || min[1] == '3') {
    transMsg.data[2] = 0x04;
  } else if (min[1] == '4' || min[1] == '5') {
    transMsg.data[2] = 0x05;
  } else if (min[1] == '6' || min[1] == '7') {
    transMsg.data[2] = 0x06;
  } else if (min[1] == '7' || min[1] == '8') {
    transMsg.data[2] = 0x07;
  }
  sendMsg(transMsg);

  if (oldMin != now.minute()) {
    secondsInterval = 0;
    msg0x23.data[7] = 0x54;
    sendMsg(msg0x23);
    oldMin = now.minute();
    int hours = now.hour() - 12;
    int minutes = now.minute();
    int tFreq = 0;
    String send = "speed ";
    int h = now.hour() > 12 ? (now.hour() - 12) : now.hour();
    double displaySpeed = h * 10 + (now.minute() / 60.0) * 10;
    Serial.println(displaySpeed);
    send.concat(displaySpeed);
    sendToArduino(send);

      // startFreq + (steps to next increment / minutes between increments) * (current minutes - lowest number of minutes in this range)
      if (minutes < 3) {
        tFreq = 0 + (15 / 3) * minutes;
      } else if (minutes >= 3 && minutes < 5) {
        tFreq = 15 + (11 / 2) * (minutes - 3);
      } else if (minutes >= 5 && minutes < 10) {
        tFreq = 26 + (24 / 5) * (minutes - 5);
      } else if (minutes >= 10 && minutes < 15) {
        tFreq = 50 + (25 / 5) * (minutes - 10);
      } else if (minutes >= 15 && minutes < 20) {
        tFreq = 75 + (25 / 5) * (minutes - 15);
      } else if (minutes >= 20 && minutes < 25) {
        tFreq = 100 + (25 / 5) * (minutes - 20);
      } else if (minutes >= 25 && minutes < 30) {
        tFreq = 125 + (25 / 5) * (minutes - 25);
      } else if (minutes >= 30 && minutes < 35) {
        tFreq = 150 + (24 / 5) * (minutes - 30);
      } else if (minutes >= 35 && minutes < 40) {
        tFreq = 174 + (24 / 5) * (minutes - 35);
      } else if (minutes >= 40 && minutes < 45) {
        tFreq = 198 + (24 / 5) * (minutes - 40);
      } else if (minutes >= 45 && minutes < 50) {
        tFreq = 222 + (24 / 5) * (minutes - 45);
      } else if (minutes >= 50 && minutes < 55) {
        tFreq = 248 + (23 / 5) * (minutes - 50);
      } else if (minutes >= 55 && minutes < 60) {
        tFreq = 271 + (24 / 5) * (minutes - 55);
      }
    NewTone(9, tFreq);
  }
}
