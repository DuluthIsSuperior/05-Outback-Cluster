#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SPI.h>
#include <SD.h>
#include <NewTone.h>
#include <SoftwareSerial.h>

#define SPEEDO_PIN 6

SoftwareSerial ControllerSerial(2, 3);  // RX, TX
const int messageCount = 8;
tCAN messages[messageCount];

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

byte registerData = 0b00000000;

// writes the new data to the register connected to the given Arduino pin
void writeToRegister() {
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); // 0b876543210
  digitalWrite(4, LOW); // brings the select pin on the given shift register low to let it read in the new data
  SPI.transfer(registerData);
  digitalWrite(4, HIGH);  // brings the select pin on the given shift register high so that it ignores new data until needed
  SPI.endTransaction();
}

int ABS = 5;
int BRAKE = 9;
int BATTERY = 14;
int SEATBELT = 15;
int OIL = 16;
int CHECK_ENGINE = 17;
int BRIGHT = 18;
int SECURITY = 19;
void setup() {
  Serial.begin(9600);
  ControllerSerial.begin(9600);
  pinMode(4, OUTPUT);
  SPI.begin();
  digitalWrite(4, HIGH);
  writeToRegister();
  registerData = 0b10000000;
  writeToRegister();
  if (!Canbus.init(CANSPEED_125)) {
    while (1) {}
  }

  messages[0] = setMessage(0x080, 7, 0x85, 0x4A, 0x01, 0x4E, 0x00, 0x00, 0xDE);
  messages[1] = setMessage(0x022, 8, 0x00, 0x00, 0x04, 0x25, 0x00, 0x00, 0x00, 0x00);
  messages[2] = setMessage(0x040, 8, 0x80, 0xA9, 0x00, 0xB9, 0x00, 0x00, 0x00, 0x08);
  messages[3] = setMessage(0x020, 8, 0xFF, 0xBF, 0xFF, 0x7F, 0xFF, 0x7F, 0x00, 0x00);
  messages[4] = setMessage(0x023, 8, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xFA, 0x02, 0x88);
  messages[5] = setMessage(0x080, 7, 0x85, 0x4A, 0x01, 0x4E, 0x00, 0x00, 0xDE);
  messages[6] = setMessage(0x080, 7, 0x85, 0x48, 0x01, 0x4E, 0x00, 0x00, 0x47);
  messages[7] = setMessage(0x021, 8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00);

  pinMode(ABS, OUTPUT);
  digitalWrite(ABS, LOW);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, LOW);
  pinMode(BATTERY, OUTPUT);
  digitalWrite(BATTERY, LOW);
  pinMode(SEATBELT, OUTPUT);
  digitalWrite(SEATBELT, LOW);
  pinMode(OIL, OUTPUT);
  digitalWrite(OIL, LOW);
  pinMode(CHECK_ENGINE, OUTPUT);
  digitalWrite(CHECK_ENGINE, LOW);
  pinMode(BRIGHT, OUTPUT);
  digitalWrite(BRIGHT, LOW);
  pinMode(SECURITY, OUTPUT);
  digitalWrite(SECURITY, LOW);
  
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
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

void dataRecieved(Stream &serial) {
    String raw = "";
    while (serial.available()) {
      delay(5);
      char in = serial.read();
      raw.concat(in);
    }
    String splitData[10];
    split(raw, ' ', splitData);
    char s[4];
    char *pEnd;
    splitData[1].substring(0, 4).toCharArray(s, 4);
    int id = strtol(s, &pEnd, 16);
    if (splitData[0] == "R") {
      for (int i = 0; i < messageCount; i++) {
        if (messages[i].id == id) {
          serial.print(messages[i].id);
          serial.print(" ");
          for (int j = 0; j < messages[j].header.length; j++) {
            serial.print(messages[i].data[j], HEX);
            serial.print(" ");
          }
          serial.println();
        }
      }
    } else if (splitData[0] == "S") {
      for (int i = 0; i < messageCount; i++) {
        if (messages[i].id == id) {
          int newData[8];
          for (int j = 0; j < messages[i].header.length; j++) {
            if (id == 0x80 && j == 6) { // trips "Er IU" if this hex number is overwritten
              newData[j] = messages[i].data[j];
              continue;
            }
            char ss[2];
            splitData[j + 2].substring(0, 2).toCharArray(ss, 3);
            newData[j] = strtol(ss, &pEnd, 16);
          }
          messages[i] = setMessage(messages[i].id, messages[i].header.length, newData[0], newData[1], newData[2], newData[3], newData[4], newData[5], newData[6], newData[7], newData[8]);
          mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
          mcp2515_send_message(&messages[i]);
        }
      }
    } else if (splitData[0] == "speed") {
      adjustSpeedometer(splitData[1].toInt());
    } else if (splitData[0] == "airbag") {
      int request = id;
      int current = bitRead(registerData, 7) == 0 ? 1 : 0;
      if (request != current) {
        registerData = registerData ^ 0b10000000;
        writeToRegister();
      }
    } else if (splitData[0] == "left") {
      int request = id;
      int current = bitRead(registerData, 6) == 0 ? 0 : 1;
      if (request != current) {
        registerData = registerData ^ 0b01000000;
        writeToRegister();
      }
    } else if (splitData[0] == "right") {
      int request = id;
      int current = bitRead(registerData, 5) == 0 ? 0 : 1;
      if (request != current) {
        registerData = registerData ^ 0b00100000;
        writeToRegister();
      }
    } else if (splitData[0] == "lights") {
      int request = id;
      int current = bitRead(registerData, 4) == 0 ? 0 : 1;
      if (request != current) {
        registerData = registerData ^ 0b00010000;
        writeToRegister();
      }
    } else if (splitData[0] == "brake") {
      int request = id;
      int current = bitRead(registerData, 3) == 0 ? 0 : 1;
      if (request != current) {
        registerData = registerData ^ 0b00001000;
        writeToRegister();
      }
    } else if (splitData[0] == "abs") {
      int request = id;
      int current = bitRead(registerData, 2) == 0 ? 0 : 1;
      if (request != current) {
        registerData = registerData ^ 0b00000100;
        writeToRegister();
      }
    } else if (splitData[0] == "battery" || splitData[0] == "seatbelt" || splitData[0] == "oil" || splitData[0] == "check_engine" || splitData[0] == "bright" || splitData[0] == "security") {
      int pinNumber = splitData[0] == "abs" ? ABS : splitData[0] == "brake" ? BRAKE : splitData[0] == "battery" ? BATTERY : splitData[0] == "seatbelt" ? SEATBELT : splitData[0] == "oil" ? OIL : splitData[0] == "check_engine" ? CHECK_ENGINE : 
        splitData[0] == "bright" ? BRIGHT : splitData[0] == "security" ? SECURITY : -1;
      int request = id;
      int current = digitalRead(pinNumber);
      if (request != current) {
        digitalWrite(pinNumber, !digitalRead(pinNumber));
      }
    } else {
      serial.println("fail");
    }
    serial.println("done");
}

void loop() {
  for (int i = 0; i < messageCount; i++) {
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
    mcp2515_send_message(&messages[i]);
  }

  if (ControllerSerial.available()) {
    dataRecieved(ControllerSerial);
  } else if (Serial.available()) {
    dataRecieved(Serial);
  }
}

int hexCharToInt(String src, int sLength, int from, int to) {
  char s[sLength];
  char *pEnd;
  src.substring(from, to).toCharArray(s, sLength);
  return strtol(s, &pEnd, 16);
}

void changeSpeedometer(int start, double increment, int newSpeed, int startFreq, int divisor) {
  start += increment * newSpeed;
  String hexStr = String(start, HEX);
  if (start < 0x100) {
    hexStr = "0" + hexStr;
  }
  //Serial.println(start, HEX);
  int hex5 = -1;
  int hex4 = -1;
  if (start > 0xFFF) {
    hex5 = hexCharToInt(hexStr, 3, 0, 2);
    hex4 = hexCharToInt(hexStr, 3, 2, 4);
  } else {
    hex5 = hexCharToInt(hexStr, 2, 0, 1);
    hex4 = hexCharToInt(hexStr, 3, 1, 3);
  }
  messages[7].data[4] = hex4;
  messages[7].data[5] = hex5;

  NewTone(SPEEDO_PIN, startFreq + newSpeed + (newSpeed / divisor));
}

void adjustSpeedometer(int newSpeed) {
  if (newSpeed >= 0 && newSpeed <= 10) {
    int start = 0x000;
    start += 26.7 * newSpeed;
    String hexStr = String(start, HEX);
    if (start < 0x100) {
      hexStr = "0" + hexStr;
    }
    //Serial.println(start, HEX);
    int hex5 = hexCharToInt(hexStr, 2, 0, 1);
    int hex4 = hexCharToInt(hexStr, 3, 1, 3);
    //Serial.println(hex4);
    //Serial.println(hex5);
    messages[7].data[4] = hex4;
    messages[7].data[5] = hex5;

    if (newSpeed <= 5) {
      NewTone(SPEEDO_PIN, newSpeed);
    } else {
      int temp = newSpeed - 5;
      NewTone(SPEEDO_PIN, 5 + temp + (temp / 5)); // 1.2 - for every 5 mph, add one to the frequency
    }
  } else if (newSpeed >= 10 && newSpeed <= 20) {
    changeSpeedometer(0x10B, (0x239 - 0x10B) / 10, newSpeed - 10, 11, 5); // 23 - 11 = 12 / 10 mph = 1.2
  } else if (newSpeed >= 20 && newSpeed <= 30) {
    changeSpeedometer(0x239, (0x356 - 0x239) / 10, newSpeed - 20, 23, 5); // 35 - 23 = 12 / 10 = 1.2
  } else if (newSpeed >= 30 && newSpeed <= 40) {
    changeSpeedometer(0x356, (0x460 - 0x356) / 10, newSpeed - 30, 35, 11);  // 45 - 35 = 10 / 10 = 0; 11 is used to avoid divide by 0 error
  } else if (newSpeed >= 40 && newSpeed <= 50) {
    changeSpeedometer(0x460, (0x58F - 0x460) / 10, newSpeed - 40, 45, 5); // 57 - 45 = 12 / 10 = 1.2
  } else if (newSpeed >= 50 && newSpeed <= 60) {
    changeSpeedometer(0x58F, (0x6AB - 0x58F) / 10, newSpeed - 50, 57, 10);  // 68 - 57 = 11 / 10 = 1.1
  } else if (newSpeed >= 60 && newSpeed <= 70) {
    changeSpeedometer(0x6AB, (0x7B6 - 0x6AB) / 10, newSpeed - 60, 68, 10);  // 79 - 68 = 11 / 10 = 1.1
  } else if (newSpeed >= 70 && newSpeed <= 80) {
    changeSpeedometer(0x7B6, (0x8D2 - 0x7B6) / 10, newSpeed - 70, 79, 10);  // 90 - 79 = 11 / 10 = 1.1
  } else if (newSpeed >= 80 && newSpeed <= 90) {
    changeSpeedometer(0x8D2, (0x9DD - 0x8D2) / 10, newSpeed - 80, 90, 10);  // 101 - 90 = 11 / 10 = 1.1
  } else if (newSpeed >= 90 && newSpeed <= 100) {
    changeSpeedometer(0x9DD, (0xAF9 - 0x9DD) / 10, newSpeed - 90, 101, 10); // 112 - 101 = 11 / 10 = 1.1
  } else if (newSpeed >= 100 && newSpeed <= 110) {
    changeSpeedometer(0xAF9, (0xC16 - 0xAF9) / 10, newSpeed - 100, 112, 10);  // 123 - 112 = 11 / 10 = 1.1
  } else if (newSpeed >= 110 && newSpeed <= 120) {
    changeSpeedometer(0xC16, (0xD20 - 0xC16) / 10, newSpeed - 110, 123, 10);  // 134 - 123 = 11 / 10 = 1.1
  } else if (newSpeed >= 120 && newSpeed <= 130) {
    changeSpeedometer(0xD20, (0xE3D - 0xD20) / 10, newSpeed - 120, 134, 10);  // 145 - 134 = 11 / 10 = 1.1
  } else if (newSpeed >= 130 && newSpeed <= 140) {
    changeSpeedometer(0xE3D, (0xF6B - 0xE3D) / 10, newSpeed - 130, 145, 10);  // 156 - 145 = 11 / 10 = 1.1
  } else if (newSpeed >= 140 /*&& newSpeed <= 150*/) {
    changeSpeedometer(0xF6B, (0x1076 - 0xF6B) / 10, newSpeed - 140, 156, 10); // 167 - 156 = 11 / 10 = 1.1
  }
}
