#include <Adafruit_BNO08x.h>


#define RAD_TO_DEG_X_10 572.95779513082320876798154814105
// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

struct IMUVector {
  uint32_t time;
  float qr;
  float qi;
  float qj;
  float qk;
} imuVector;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// booleans to see if we are using BNO08x
bool useBNO08x = false;
uint8_t error;

Adafruit_BNO08x bno08x(-1);
// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;

sh2_SensorValue_t sensorValue;

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = {};
char speedKnots[10] = {};

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

void initHandler() {
  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
}
// If odd characters showed up.
void errorHandler() {
  //nothing at the moment
}

void GGA_Handler()  //Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);

  // latitude
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);

  // longitude
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);

  // fix quality
  parser.getArg(5, fixQuality);

  // satellite #
  parser.getArg(6, numSats);

  // HDOP
  parser.getArg(7, HDOP);

  // altitude
  parser.getArg(8, altitude);

  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  GGA_Available = true;

  if (useBNO08x) {
    calculateIMU();
  } else {
    itoa(65535, imuHeading, 10);
  }             //Get IMU data ready
  BuildNmea();  //Build & send data GPS data to AgIO (Both Dual & Single)
}

void gpsStream() {
  while (Serial2.available()) {
    parser << Serial2.read();
  }
}

void readBNO(float qr, float qi, float qj, float qk) {
  imuVector.time = millis();
  imuVector.qr = qr;
  imuVector.qi = qi;
  imuVector.qj = qj;
  imuVector.qk = qk;
}

void calculateIMU() {
  quaternionToEuler(imuVector.qr, imuVector.qi, imuVector.qj, imuVector.qk);
  int16_t temp = 0;

  //BNO is reading in its own timer
  // Fill rest of Panda Sentence - Heading
  temp = ypr.yaw;
  itoa(temp, imuHeading, 10);

  // the pitch x10
  temp = (int16_t)ypr.pitch;
  itoa(temp, imuPitch, 10);

  // the roll x10
  temp = (int16_t)ypr.roll;
  itoa(temp, imuRoll, 10);

  // YawRate - 0 for now
  itoa(0, imuYawRate, 10);
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  if (steerConfig.IsUseY_Axis) {
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  } else {
    ypr.roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  }

  ypr.yaw *= -RAD_TO_DEG_X_10;
  if (ypr.yaw < 0) {
    ypr.yaw += 3600;
  }
  ypr.pitch *= RAD_TO_DEG_X_10;
  ypr.roll *= RAD_TO_DEG_X_10;

  if (invertRoll) {
    ypr.roll *= -1;
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("Could not enable stabilized remote vector");
    return;
  }
}

void imuTask() {
  if (!useBNO08x) {
    return;
  }
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

    case SH2_GAME_ROTATION_VECTOR:
      readBNO(sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
      break;
  }
}


void initIMU() {
  //set up communication
  Wire.begin();
  for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
    bno08xAddress = bno08xAddresses[i];

    //Serial.print("\r\nChecking for BNO08X on ");
    //Serial.println(bno08xAddress, HEX);
    Wire.beginTransmission(bno08xAddress);
    error = Wire.endTransmission();

    if (error == 0) {
      //Serial.println("Error = 0");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X Ok.");
      // Initialize BNO080 lib
      if (bno08x.begin_I2C((int32_t)bno08xAddress))  //??? Passing NULL to non pointer argument, remove maybe ???
      {
        setReports();
        useBNO08x = true;
      } else {
        Serial.println("BNO080 not detected at given I2C address.");
      }
    } else {
      //Serial.println("Error = 4");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X not Connected or Found");
    }
    if (useBNO08x) {
      imuTS.enable();
      break;
    }
  }
}

void BuildNmea(void) {

  strcpy(nmea, "");

  strcat(nmea, "$PANDA,");

  strcat(nmea, fixTime);
  strcat(nmea, ",");

  strcat(nmea, latitude);
  strcat(nmea, ",");

  strcat(nmea, latNS);
  strcat(nmea, ",");

  strcat(nmea, longitude);
  strcat(nmea, ",");

  strcat(nmea, lonEW);
  strcat(nmea, ",");

  // 6
  strcat(nmea, fixQuality);
  strcat(nmea, ",");

  strcat(nmea, numSats);
  strcat(nmea, ",");

  strcat(nmea, HDOP);
  strcat(nmea, ",");

  strcat(nmea, altitude);
  strcat(nmea, ",");

  //10
  strcat(nmea, ageDGPS);
  strcat(nmea, ",");

  //11
  strcat(nmea, speedKnots);
  strcat(nmea, ",");

  //12
  strcat(nmea, imuHeading);
  strcat(nmea, ",");

  //13
  strcat(nmea, imuRoll);
  strcat(nmea, ",");

  //14
  strcat(nmea, imuPitch);
  strcat(nmea, ",");

  //15
  strcat(nmea, imuYawRate);

  strcat(nmea, "*");

  CalculateChecksum();

  strcat(nmea, "\r\n");

  int len = strlen(nmea);
  sendData((uint8_t*)nmea, len);
}

void CalculateChecksum(void) {
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++) {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*') {
      break;
    }

    sum ^= tmp;  // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix
  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E
  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots
  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)
  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second
  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with
  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV
  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with
  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/

void VTG_Handler() {
  // vtg heading
  parser.getArg(0, vtgHeading);

  // vtg Speed knots
  parser.getArg(4, speedKnots);
}