#include "esp_wifi.h"

uint8_t data[16384];
uint8_t* buffer;
int bufferIndex = 0;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//fromAutoSteerData FA 250 - sensor values etc
uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

int packetSize;

unsigned int AOGNtripPort = 2233;                 // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;             // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;              // Port of AOG that listens
IPAddress ipDes = IPAddress(255, 255, 255, 255);  //AOG IP
boolean ipDesIsSet = false;

AsyncUDP udp;
AsyncUDP ntrip;


void initUDP() {
  // Supress Debug information
  wfm.setDebugOutput(false);

  if (!wfm.autoConnect("ESP32AGOpenGPS")) {
    // Did not connect, print error message
    Serial.println("failed to connect and hit timeout");

    // Reset and try again
    ESP.restart();
    delay(1000);
  }

  WiFi.useStaticBuffers(true);
  esp_wifi_set_ps(WIFI_PS_NONE);

  myip = WiFi.localIP();
  // Connected!
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  if (udp.listen(AOGAutoSteerPort)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      autoSteerPacketPerser(packet);
    });

    if (ntrip.listen(AOGNtripPort)) {
      Serial.print("UDP Listening on IP: ");
      Serial.println(WiFi.localIP());
      ntrip.onPacket([](AsyncUDPPacket packet) {
        ntripPacketProxy(packet);
      });
    }
  }
}

void ntripPacketProxy(AsyncUDPPacket packet) {
  if (packet.length() > 0) {
    Serial2.write(packet.data(), packet.length());
  }
}


void autoSteerPacketPerser(AsyncUDPPacket udpPacket) {
  if (ipDesIsSet && udpPacket.remoteIP() != ipDes) {
    return;
  }
  if (udpPacket.length() <= 0)
    return;
  buffer = udpPacket.data();
  for (int i = 0; i < udpPacket.length(); i++) {
    data[bufferIndex] = buffer[i];
    bufferIndex++;
  }
  if (bufferIndex < 4) {
    return;
  }
  for (int i = 0; i < bufferIndex; i++) {
    if (data[i] == 128 && data[i + 1] == 129) {
      int lenght = data[i + 4] + 6;
      if (lenght > i + bufferIndex + 1) {
        break;
      }
      for (int x = 0; x < lenght; x++) {
        data[x] = data[i + x];
      }


      bufferIndex -= lenght;
      i = -1;
      byte CK_A = 0;
      for (int j = 2; j < lenght - 1; j++) {
        CK_A += data[j];
      }

      if (data[lenght - 1] != (byte)CK_A) {
        Serial.print("Packet: CRC error: ");
        Serial.print(CK_A);
        printLnByteArray(data, lenght);
        break;
      }

      parsePacket(data, lenght, udpPacket);
    }
  }
}


void parsePacket(uint8_t* packet, int size, AsyncUDPPacket udpPacket) {

  if (packet[0] == 0x80 && packet[1] == 0x81 && packet[2] == 0x7F)  //Data
  {
    switch (packet[3]) {
      case 0xFE:
        {
          gpsSpeed = ((float)(packet[5] | packet[6] << 8)) * 0.1;
          gpsSpeedUpdateTimer = 0;

          prevGuidanceStatus = guidanceStatus;

          guidanceStatus = packet[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(packet[8] | ((int8_t)packet[9]) << 8)) * 0.01;  //high low bytes

          if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1)) {
            watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
          } else                                   //valid conditions to turn on autosteer
          {
            watchdogTimer = 0;  //reset watchdog
          }

          //Bit 10 Tram
          tram = packet[10];

          //Bit 11
          relay = packet[11];

          //Bit 12
          relayHi = packet[12];

          //----------------------------------------------------------------------------
          //Serial Send to agopenGPS

          int16_t sa = (int16_t)(steerAngleActual * 100);

          PGN_253[5] = (uint8_t)sa;
          PGN_253[6] = sa >> 8;

          // heading
          PGN_253[7] = (uint8_t)9999;
          PGN_253[8] = 9999 >> 8;

          // roll
          PGN_253[9] = (uint8_t)8888;
          PGN_253[10] = 8888 >> 8;

          PGN_253[11] = switchByte;
          PGN_253[12] = (uint8_t)pwmDisplay;

          sendData(PGN_253, sizeof(PGN_253));

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              sendData(PGN_250, sizeof(PGN_250));
              aog2Count = 0;
            }
          }
          break;
        }
      //steer settings
      case 252:
        {  //0xFC
          //PID values
          steerSettings.Kp = ((float)packet[5]);    // read Kp from AgOpenGPS
          steerSettings.highPWM = packet[6];        // read high pwm
          steerSettings.lowPWM = (float)packet[7];  // read lowPWM from AgOpenGPS
          steerSettings.minPWM = packet[8];         //read the minimum amount of PWM for instant on
          float temp = (float)steerSettings.minPWM * 1.2;
          steerSettings.lowPWM = (byte)temp;
          steerSettings.steerSensorCounts = packet[9];   //sent as setting displayed in AOG
          steerSettings.wasOffset = (packet[10]);        //read was zero offset Lo
          steerSettings.wasOffset |= (packet[11] << 8);  //read was zero offset Hi
          steerSettings.AckermanFix = (float)packet[12] * 0.01;

          //crc
          //autoSteerUdpData[13];

          //store in EEPROM
          EEPROM.put(10, steerSettings);
          EEPROM.commit();
          // Re-Init steer settings
          steerSettingsInit();
          break;
        }
      case 251:  //251 FB - SteerConfig
        {
          uint8_t sett = packet[5];  //setting0

          if (bitRead(sett, 0)) steerConfig.InvertWAS = 1;
          else steerConfig.InvertWAS = 0;
          if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1;
          else steerConfig.IsRelayActiveHigh = 0;
          if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1;
          else steerConfig.MotorDriveDirection = 0;
          if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1;
          else steerConfig.SingleInputWAS = 0;
          if (bitRead(sett, 4)) steerConfig.CytronDriver = 1;
          else steerConfig.CytronDriver = 0;
          if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1;
          else steerConfig.SteerSwitch = 0;
          if (bitRead(sett, 6)) steerConfig.SteerButton = 1;
          else steerConfig.SteerButton = 0;
          if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1;
          else steerConfig.ShaftEncoder = 0;

          steerConfig.PulseCountMax = packet[6];

          //was speed
          //autoSteerUdpData[7];

          sett = packet[8];  //setting1 - Danfoss valve etc

          if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1;
          else steerConfig.IsDanfoss = 0;
          if (bitRead(sett, 1)) steerConfig.PressureSensor = 1;
          else steerConfig.PressureSensor = 0;
          if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1;
          else steerConfig.CurrentSensor = 0;
          if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1;
          else steerConfig.IsUseY_Axis = 0;

          //crc
          //autoSteerUdpData[13];

          EEPROM.put(40, steerConfig);
          // Re-Init
        }
      case 200:
        {  // Hello from AgIO

          int16_t sa = (int16_t)(steerAngleActual * 100);

          helloFromAutoSteer[5] = (uint8_t)sa;
          helloFromAutoSteer[6] = sa >> 8;

          helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
          helloFromAutoSteer[8] = helloSteerPosition >> 8;
          helloFromAutoSteer[9] = switchByte;

          sendData(helloFromAutoSteer, sizeof(helloFromAutoSteer));
          if (useBNO08x) {
            sendData(helloFromIMU, sizeof(helloFromIMU));
          }
        }
      case 202:
        {
          //make really sure this is the reply pgn
          if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202) {
            ipDes = udpPacket.remoteIP();
            ipDesIsSet = true;
            //hello from AgIO
            uint8_t scanReply[] = { 128, 129, 126, 203, 7,
                                    myip[0], myip[1], myip[2], myip[3],
                                    myip[0], myip[1], myip[2], 23 };

            sendData(scanReply, sizeof(scanReply));
          }
        }
    }
  } else {
    Serial.print("Unknown packet!!!");
    printLnByteArray(packet, size);
  }
}

void sendData(uint8_t* data, uint8_t datalen) {

  AsyncUDPMessage m = AsyncUDPMessage(datalen);
  int16_t CK_A = 0;
  for (uint8_t i = 2; i < datalen - 1; i++) {
    CK_A = (CK_A + data[i]);
  }
  data[datalen - 1] = CK_A;

  m.write(data, datalen);
  udp.sendTo(m, ipDes, portDestination);
}

void printLnByteArray(uint8_t* data, uint8_t datalen) {
  for (int i = 0; i < datalen; i++) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}
