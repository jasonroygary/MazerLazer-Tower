//////////////////////////////////////////
//LIBRARIES
//////////////////////////////////////////
#include <EEPROM.h>
#include <XBee.h>
#include <SoftwareSerial.h>
//////////////////////////////////////////
//XBEE SETUP
//////////////////////////////////////////
XBee xbee = XBee();
unsigned long start = millis();
//PAYLOAD SETUP
uint8_t payload[] = {0, 0, 0, 0, 0, 0, 0 }; //COMMAND CODE + 6 PARAMETERS
Tx16Request tx = Tx16Request(0x1000, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
//PAYLOAD DATA COMPONENTS
uint8_t option = 0;
uint8_t data = 0;
uint8_t rssi = 0;
uint16_t remoteAddress = 0;
uint8_t frameDataSize = 0;
///////////////////////////////////////////
//SETUP OF PINS
///////////////////////////////////////////
int laserPins[] = {2,3,4,5,6,7};
int sensorPins[] = {0,1,2,3,4,5};
int sensorStatus[] = {0,0,0,0,0,0};
int sensorStatusPrevious[] = {0,0,0,0,0,0};
int sensorStatusSent[] = {0,0,0,0,0,0};
int sensorStatusMonitored[] = {1,1,1,1,1,1}; //BY DEFAULT WE CARE ABOUT ALL THE SENSORS 0=OFF
//SENSOR STATUS AND DELAY
int sensorThreshold = 1000; //THRESHOLD FOR DETECTING AN ON STATE
int sensorDelay = 900; //LOOPS THAT IT WAITS FOR STATE CHANGE TO BE SENT
int sensorDelayState = 0; //CURRENT STATE OF THE LOOP
int sensorStateChange = 0; //IS THERE A SENSOR STATE CHANGE
int sensorMonitoring = 0; //DEFAULTS TO OFF FOR SENSOR DETECTION (1 = ON)

int buttonPin = 9;
int buttonLightPin = 10;
int buttonState = 0; //OFF
int buttonStateCurrent = 0;
int buttonDetection=1; //DEFAULT IS TO DETECT BUTTON PUSH (0=OFF)

int relayPin1 = 11;
int relayPin2 = 12;
///////////////////////////////////////////
//SETUP VARIABLES
///////////////////////////////////////////
//RSSI
int transAddress=1000; //ADDRESS FOR XBEE DEVICE TO SEND TO
long intervalRSSI = 5000; //HOW OFTEN TO SEND RSSI
long previousMillisRSSI = 0; // will store last time was updated
int RSSIOnOff = 0; //1 = ON, 0 = OFF
//COMMAND CODES AND PARAMETERS
int cCode;
int cPara1;
int cPara2;
int cPara3;
int cPara4;
int cPara5;
int cPara6;

int musicID; //ID FOR MP3 TO PLAY
int musicType = 0; //MP3 OR WAV 0=MP3, 1=WAV
int OnOff = 0; //1 = ON, 0 = OFF (FOR ALL DEVICES IN UNIT)
///////////////////////////////////////////
//SOFTWARE SERIAL SETUP
///////////////////////////////////////////
//#define rxPin 10
//#define txPin 13
#define VMUSIC_RX 8
#define VMUSIC_TX 13
SoftwareSerial mySerial =  SoftwareSerial(VMUSIC_RX, VMUSIC_TX);
///////////////////////////////////////////
//SYSTEM SETUP
///////////////////////////////////////////
void setup() {
  //PIN MODES
  pinMode(buttonPin, INPUT); 
  pinMode(buttonLightPin, OUTPUT); 
  pinMode(relayPin1, OUTPUT); 
  pinMode(relayPin2, OUTPUT);
  //SENSOR PINS
  pinMode(sensorPins[0], INPUT);
  pinMode(sensorPins[1], INPUT);    
  pinMode(sensorPins[2], INPUT);
  pinMode(sensorPins[3], INPUT);
  pinMode(sensorPins[4], INPUT);  
  pinMode(sensorPins[5], INPUT); 
  
  //SOFTWARE SERIAL DATA RATE
  mySerial.begin(2400);
  pinMode(VMUSIC_RX, INPUT);
  pinMode(VMUSIC_TX, OUTPUT);
  xbee.begin(38400);
  //SETUP LASER PINS
  int i;
  for (i = 0; i < 6; i = i + 1) {
    pinMode(laserPins[i],OUTPUT);
    digitalWrite(laserPins[i],LOW);
  }
  eepromReadAll();
}

void loop() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
        xBeeReceive();
      }
    }
  }
  checkSensors();
  checkButton();
  if (RSSIOnOff==1) {
  rssiSend();
  }
}

void commandProcessing() {
  int i = 0;
  switch (cCode) {
  case 10: //TURN ON - OFF EVERYTHING ON DEVICE
    OnOff = cPara1; //0 = OFF, 1=ON
    if (cPara1==0) {
      allLasersOff();
      pinMode(relayPin1,OUTPUT);
      digitalWrite(relayPin1,LOW);
      pinMode(relayPin2,OUTPUT);
      digitalWrite(relayPin2,LOW);
      digitalWrite(buttonLightPin,LOW);
      
    }
    else {
      allLasersOn();
      pinMode(relayPin1,OUTPUT);
      digitalWrite(relayPin1,HIGH);
      pinMode(relayPin2,OUTPUT);
      digitalWrite(relayPin2,HIGH);
      digitalWrite(buttonLightPin,HIGH);
    }
    eepromWriteAll();
    break;
  case 11: //ECHO BACK TO SOURCE
    payload[0] = 0xA & 0xff;
    payload[1] = cPara1 & 0xff;
    payload[2] = cPara2 & 0xff;
    payload[3] = cPara3 & 0xff;
    payload[4] = cPara4 & 0xff;
    payload[5] = cPara5 & 0xff;
    payload[6] = cPara6 & 0xff;
    xbee.send(tx);  
    break;
  case 12: //Change Address for XBee Transmission
    transAddress = cPara1 * pow(16, 3) + cPara2 * pow(16, 2) + cPara3 * pow(16, 1) + cPara4 * pow(16, 0);
    tx.setAddress16 (transAddress); //CPARA1 - CPARA4
    eepromWriteAll();
    break;
  case 13: //Set RSSI Interval in ms
    intervalRSSI = cPara1 * 100;
    eepromWriteAll();
    break;
  case 14: //Turn ON/OFF RSSI Transmit
    RSSIOnOff = cPara1;
    eepromWriteAll();
    break;
  case 15: //SEND ONE RSSI PACKET
    rssiSend();
    break;
  case 16: //Give me specific EEPROM Value  
    payload[0] = 0xB & 0xff;
    payload[1] = cPara1 & 0xff;
    payload[2] = EEPROM.read(cPara1) & 0xff;
    payload[3] = 0x0 & 0xff;
    payload[4] = 0x0 & 0xff;
    payload[5] = 0x0 & 0xff;
    payload[6] = 0x0 & 0xff;
    xbee.send(tx);
    break;
  case 17: //WRITE TO EEPROM (THERE IS A DELAY IN WRITTING EEPROM OF 3.3MS PER WRITE)
    EEPROM.write(cPara1,cPara2);
    break; 
  case 18: //GIVE ALL EEPROM VALUES
    eepromWriteAll();
    break;    
  case 19: //GIVE AMBIENT SENSOR THRESHOLD
    getSensorThreshold();
    break;  
  case 20: //SET SENSOR DELAY LOOP FOR STATE CHANGE
    sensorDelay = cPara1*1000 + cPara2*100 + cPara3*10 + cPara4;
    eepromWriteAll();
    break;
  case 21: //SET SENSOR THRESHOLD
    sensorThreshold = cPara1 * 100;
    eepromWriteAll();
     break;  
  case 22: //GET STATUS OF SENSORS
    payload[0] = 0x14 & 0xff;
    payload[1] = sensorStatus[0] & 0xff;
    payload[2] = sensorStatus[1] & 0xff;
    payload[3] = sensorStatus[2] & 0xff;
    payload[4] = sensorStatus[3] & 0xff;
    payload[5] = sensorStatus[4] & 0xff;
    payload[6] = sensorStatus[5] & 0xff;
    xbee.send(tx);
    break;
  case 23: //TURN ON/OFF SENSOR MONITORING (INTERUPT MODE)
  if (cPara1==1) {
      sensorMonitoring=1;
    }
    else {
      sensorMonitoring=0;
    }
     eepromWriteAll();
    break;
  case 24: //SENSOR MONITOR STATE (WHAT SENSORS TO PAY ATTENTION TO)
    sensorStatusMonitored[0] = cPara1;
    sensorStatusMonitored[1] = cPara2;
    sensorStatusMonitored[2] = cPara3;
    sensorStatusMonitored[3] = cPara4;
    sensorStatusMonitored[4] = cPara5;
    sensorStatusMonitored[5] = cPara6;
    break;
  case 25: //RESET SENSOR MESSAGE SENT STATE
    sensorStatusSent[0] = cPara1;
    sensorStatusSent[1] = cPara2;
    sensorStatusSent[2] = cPara3;
    sensorStatusSent[3] = cPara4;
    sensorStatusSent[4] = cPara5;
    sensorStatusSent[5] = cPara6;
   case 26: //SYNC SENSOR PREVIOUS STATE WITH CURRENT STATE
     for (i = 0; i <= 5; i = i + 1) {
          sensorStatusPrevious[i] = sensorStatus[i];
        }
   break;
  case 30: //TURN ON ALL RELAYS
    pinMode(relayPin1,OUTPUT);
    digitalWrite(relayPin1,HIGH);
    pinMode(relayPin2,OUTPUT);
    digitalWrite(relayPin2,HIGH);
    break;
  case 31: //TURN OFF ALL RELAYS
    pinMode(relayPin1,OUTPUT);
    digitalWrite(relayPin1,LOW);
    pinMode(relayPin2,OUTPUT);
    digitalWrite(relayPin2,LOW);
    break;
  case 32: //TURN ON SPECIFIC RELAY
    if (cPara1==1) {
      pinMode(relayPin1,OUTPUT);
      digitalWrite(relayPin1,HIGH);
    }
    else {
      pinMode(relayPin2,OUTPUT);
      digitalWrite(relayPin2,HIGH);
    }
    break;
  case 33: //TURN OFF SPECIFIC RELAY
    if (cPara1==1) {
      pinMode(relayPin1,OUTPUT);
      digitalWrite(relayPin1,LOW);
    }
    else {
      pinMode(relayPin2,OUTPUT);
      digitalWrite(relayPin2,LOW);
    }
    break;
  case 35: //PLAY SOUND
    musicID = cPara1;
    musicType = cPara2;
    playSound();
    break;
   case 36: //STOP MUSIC
      mySerial.print("VST"); //#stop playing
      mySerial.print(13,BYTE);
      break;
  case 40: //BUTTON LIGHT ON/OFF
    if (cPara1==1) {
      digitalWrite(buttonLightPin,HIGH);
    }
    else {
      digitalWrite(buttonLightPin,LOW);
    }
    break;
    case 41: //TURN ON/OFF SENSOR DETECTION DIRECT SEND
    if (cPara1==1) {
      buttonDetection=1;
    }
    else {
      buttonDetection=0;
    }
    eepromWriteAll();
  break; 
  case 45: //TURN ON ALL LASERS
    allLasersOn();
    break;
  case 46: //TURN OFF ALL LASERS
    allLasersOff();
    break;
  case 47: //TURN ON SPECIFIC LASERS (0 - 5)
    pinMode(laserPins[cPara1],OUTPUT);
    digitalWrite(laserPins[cPara1],HIGH);
    if (cPara2<=5) {
      pinMode(laserPins[cPara2],OUTPUT);
      digitalWrite(laserPins[cPara2],HIGH);
    }
    if (cPara3<=5) {
      pinMode(laserPins[cPara3],OUTPUT);
      digitalWrite(laserPins[cPara3],HIGH);
    }
    if (cPara4<=5) {
      pinMode(laserPins[cPara4],OUTPUT);
      digitalWrite(laserPins[cPara4],HIGH);
    }
    if (cPara5<=5) {
      pinMode(laserPins[cPara5],OUTPUT);
      digitalWrite(laserPins[cPara5],HIGH);
    }
    if (cPara6<=5) {
      pinMode(laserPins[cPara6],OUTPUT);
      digitalWrite(laserPins[cPara6],HIGH);
    }
    break;    
  case 48: //TURN OFF SPECIFIC LASERS (0 - 6)
    pinMode(laserPins[cPara1],OUTPUT);
    digitalWrite(laserPins[cPara1],LOW);
    if (cPara2<=6) {
      pinMode(laserPins[cPara2],OUTPUT);
      digitalWrite(laserPins[cPara2],LOW);
    }
    if (cPara3<=6) {
      pinMode(laserPins[cPara3],OUTPUT);
      digitalWrite(laserPins[cPara3],LOW);
    }
    if (cPara4<=6) {
      pinMode(laserPins[cPara4],OUTPUT);
      digitalWrite(laserPins[cPara4],LOW);
    }
    if (cPara5<=6) {
      pinMode(laserPins[cPara5],OUTPUT);
      digitalWrite(laserPins[cPara5],LOW);
    }
    if (cPara6<=6) {
      pinMode(laserPins[cPara6],OUTPUT);
      digitalWrite(laserPins[cPara6],LOW);
    }
    break;  
  }
  cCode = 0;
  cPara1 = 0;
  cPara2 = 0;
  cPara3 = 0;
  cPara4 = 0;
  cPara5 = 0;
  cPara6 = 0;
}

void rssiSend() {
  if (millis() - previousMillisRSSI > intervalRSSI) {
    //Save the last time RSSI was Sent
    previousMillisRSSI = millis();   
    //Send RSSI
    payload[0] = 0xC & 0xff;
    payload[1] = 0x42 & 0xff;
    payload[2] = 0x43 & 0xff;
    payload[3] = 0x44 & 0xff;
    payload[4] = 0x45 & 0xff;
    payload[5] = 0x46 & 0xff;
    payload[6] = 0x47 & 0xff;
    xbee.send(tx);
  }
}
void xBeeReceive() {
  xbee.getResponse().getRx16Response(rx16);
  option = rx16.getOption();
  frameDataSize = rx16.getFrameDataLength() - 4;
  data = rx16.getData(0);
  rssi = rx16.getRssi();
  remoteAddress = rx16.getRemoteAddress16();
  cCode = rx16.getData(0);
  cPara1 = rx16.getData(1);
  cPara2 = rx16.getData(2);
  cPara3 = rx16.getData(3);
  cPara4 = rx16.getData(4);
  cPara5 = rx16.getData(5);
  cPara6 = rx16.getData(6);
  //transAddress = cPara1 * pow(16, 3) + cPara2 * pow(16, 2) + cPara3 * pow(16, 1) + cPara4 * pow(16, 0);
  commandProcessing();
}  

void monitorSensors() {
  //WHAT IT WAS LAST TIME
  //WHAT IT IS THIS TIME
  //HAVE IT SENT A CHANGE MESSAGE ON THIS SENSOR YET
  //WHAT SENSORS I CARE ABOUT
  int i = 0;
  for (i = 0; i <= 5; i = i + 1) {
    if (sensorStatusPrevious[i] != sensorStatus[i]) {
      //DO I CARE ABOUT THIS SENSOR?
      //HAVE I ALREADY INFORMED ON THIS SENSOR CHANGE?
        if (sensorStatusMonitored[i]==1) { //DO I CARE ABOUT THIS SENSOR?
          if (sensorStatusSent[i] == 0) { //I HAVE NOT SENT A MESSAGE ABOUT THIS SENSOR
          payload[0] = 0x14 & 0xff;
          payload[1] = sensorStatus[0] & 0xff;
          payload[2] = sensorStatus[1] & 0xff;
          payload[3] = sensorStatus[2] & 0xff;
          payload[4] = sensorStatus[3] & 0xff;
          payload[5] = sensorStatus[4] & 0xff;
          payload[6] = sensorStatus[5] & 0xff;
          xbee.send(tx);
          //LOG THAT A MESSAGE HAS BEEN SENT
          sensorStatusSent[i] = 1;
          break; //ONLY SEND THE MESSAGE ONCE NO MATTER HOW MANY HAVE CHANGED
          }
        }
    }
  }
  //ASSIGN PREVIOUS ARRAY
  for (i = 0; i <= 5; i = i + 1) {
    sensorStatusPrevious[i] = sensorStatus[i];
  }
}  

void checkSensors() {
  //Checks the sensor status. If there is a change of status in any sensor it sends a message to the MazerLazer server
  //sensorStatus
  if (sensorDelayState == 0) {
  int i;
  int sensorCheck;
  
  for (i = 0; i <= 5; i = i + 1) {
    sensorCheck = analogRead(i);
    if (sensorCheck >= sensorThreshold) {
      //SENSOR IS ON
       sensorStatus[i]=1;
    }
      else {
      //SENSOR IS OFF
      sensorStatus[i]=0; 
    }
  }
  sensorDelayState = sensorDelay;
    if (sensorMonitoring==1) {
        monitorSensors();
    }
    else {
       //NEED TO KEEP PREVIOUS SENSOR ARRAY IN SYNC IF WE ARE NOT MONITORING  
       for (i = 0; i <= 5; i = i + 1) {
          sensorStatusPrevious[i] = sensorStatus[i];
        }
        }
      }
  else {
  sensorDelayState = sensorDelayState - 1;
  }
}  

void playSound() {
  mySerial.print("VST"); //#stop playing
  mySerial.print(13,BYTE);
  mySerial.print("VPF ");
  //mySerial.print(incomingByte, BYTE);
  mySerial.print(musicID);
  if (musicType==0) {
  mySerial.print(".mp3");
  }
  else {
  mySerial.print(".wav");
  }
  mySerial.print(13,BYTE); 
}  

void eepromWriteAll() {
  EEPROM.write(0,OnOff);
  EEPROM.write(1,transAddress); 
  EEPROM.write(2,intervalRSSI); 
  EEPROM.write(3,RSSIOnOff); 
  EEPROM.write(4,sensorThreshold);
  EEPROM.write(5,sensorDelay);
  EEPROM.write(6,sensorMonitoring);  
  EEPROM.write(7,buttonDetection);
}

void eepromReadAll() {
  OnOff = EEPROM.read(0);
  transAddress = EEPROM.read(1);
  intervalRSSI = EEPROM.read(2);
  RSSIOnOff = EEPROM.read(3);
  sensorThreshold = EEPROM.read(4);
  sensorDelay = EEPROM.read(5);
  sensorMonitoring = EEPROM.read(6);
  buttonDetection = EEPROM.read(7);
}

void allLasersOn(){
  int i;
  for (i = 0; i <= 5; i = i + 1) {
    pinMode(laserPins[i],OUTPUT);
    digitalWrite(laserPins[i],HIGH);
  }    
}  

void allLasersOff(){
  int i;
  for (i = 0; i <= 5; i = i + 1) {
    pinMode(laserPins[i],OUTPUT);
    digitalWrite(laserPins[i],LOW);
  }    
}

void allSensorsOn(){
  int i;
  for (i = 0; i <= 5; i = i + 1) {
    pinMode(laserPins[i],OUTPUT);
    digitalWrite(laserPins[i],HIGH);
  }    
}  

void checkButton() {
  buttonStateCurrent = digitalRead(buttonPin);
  if (buttonStateCurrent==HIGH) {
    //CHECK TO SEE IF IT WAS ALREADY HIGH
    if(buttonState==1) {
    //IT WAS ALREADY HIGH
    }
    else {
     //SEND MESSAGE
      if (buttonDetection==1) {
      payload[0] = 0x1E & 0xff;
      payload[1] = 0 & 0xff;
      payload[2] = 0 & 0xff;
      payload[3] = 0 & 0xff;
      payload[4] = 0 & 0xff;
      payload[5] = 0 & 0xff;
      payload[6] = 0 & 0xff;
      xbee.send(tx);
      }
      buttonState=1; //SET BUTTON STATE TO ON
     }
    }
    else {
      buttonState=0; //SET BUTTON STATE TO OFF
  }
}
void getSensorThreshold() {
 int sensorAnalogReading[] = {0,0,0,0,0,0};
 int sensorAverageArray[49];
 int sensorAverage;
 int i = 0;
 int j = 0;
 int k = 0;
 for (j = 0; j <= 49; j = j + 1) {
 for (i = 0; i <= 5; i = i + 1) {
   sensorAnalogReading[i] = analogRead(i);
 }
 for (k = 0; k <= 5; k = k + 1) {
   sensorAverage = sensorAverage + sensorAnalogReading[k]; 
 }
 sensorAverageArray[j] = sensorAverage / 6;
 sensorAverage = 0;
 delay(100);
 }
for (j = 0; j <= 20; j = j + 1) {
 sensorAverage = sensorAverage + sensorAverageArray[j];
}
sensorAverage = sensorAverage / 50;
//SEND SENSOR AVERAGE TO SERVER
payload[0] = 0x14 & 0xff;
payload[1] = sensorAverage & 0xff;
payload[2] = 0x0 & 0xff;
payload[3] = 0x0 & 0xff;
payload[4] = 0x0 & 0xff;
payload[5] = 0x0 & 0xff;
payload[6] = 0x0 & 0xff;
xbee.send(tx);
}
