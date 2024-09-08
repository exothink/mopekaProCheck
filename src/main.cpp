#include <Arduino.h>

/*
C:\Users\EliteDesk\Documents\PlatformIO\Projects\mopekaProCheck\src\main.cpp            je 9/6/24

from:  'Projects\bleExplorer\src\main.cpp'             je 8/30/24
WORKS!!!

peripheral.address() = c7:df:3b:63:7f:41  The app only shows the last three bytes. c7:df:3b must be common.(?)
Advertise interval averages 4 seconds.

 */

#include <ArduinoBLE.h>

// #define DEBUG
#define VERS "\nMopekaProCheck "
#define AD_UUID "fee5"
#define DEV_ADR "c7:df:3b:63:7f:41"

#define SCAN_ITERVAL 1000

void printVal(int, uint8_t, int);
void decodeMopeka(uint8_t *); // takes a pointer to an array
void printResults();

static const double
    AIR[] = {0.153096, 0.000327, -0.000000294},
    PROPANE[] = {0.573045, -0.002822, -0.00000535},
    WATER[] = {0.600592, 0.003124, -0.00001368},
    GASOLINE[] = {0.7373417462, -0.001978229885, 0.00000202162}; // same as: DIESEL, LNG, OIL, HYDRAULIC_OIL

static const double *MOPEKA_COEF = PROPANE; // PROPANE WATER
uint8_t manuDataBuffer[29];
uint8_t *pManuDataBuffer = &manuDataBuffer[2]; // skips the first two manufacturer ID bytes
// even though manufacturerDataLength() = 10, the buffer can't be less than 29 or esp32 will throw exceptions

enum SensorReadQuality
{
  QUALITY_HIGH = 0x3,
  QUALITY_MED = 0x2,
  QUALITY_LOW = 0x1,
  QUALITY_NONE = 0x0
};

struct sensData
{
  double rxInterval;
  int levelMM;
  int levelInches;
  SensorReadQuality quality_value;
  float v;
  float tc;
  float tf;
} mResults;

void setup()
{
  delay(2000);
  Serial.begin(9600); // 115200
  Serial.print(VERS);
  Serial.println(__TIMESTAMP__);

  if (!BLE.begin()) // begin initialization
  {
    Serial.println("Can't hear 'Mopeka Pro Check'.");
    while (1)
      ;
  }
  // BLE.scan(); // start scanning for peripherals
  BLE.scanForAddress(DEV_ADR); // start scanning for Mopeka only
} // end of 'setup'

unsigned long lastTime = 0, lastTimeFound = 0;
void loop()
{
  if ((millis() - lastTime) > SCAN_ITERVAL)
  {
    // Serial.println(millis() - lastTime);
    lastTime = millis();
    BLEDevice peripheral = BLE.available(); // check if a peripheral has been discovered

    if (peripheral)
    {
      mResults.rxInterval = (millis() - lastTimeFound) / 1000.0;
      Serial.print("\n  Advertise interval: ");
      Serial.println(mResults.rxInterval, 1);
      lastTimeFound = millis();
      // discovered a peripheral, print out address, local name, and advertised service
#ifdef DEBUG
      Serial.print("\nFound ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
#endif

      bool mDataAvailble = peripheral.hasManufacturerData();
      if (mDataAvailble)
      {
        int ManuDataLen = peripheral.manufacturerDataLength();
        int retLen = peripheral.manufacturerData(manuDataBuffer, ManuDataLen); // populates "manuDataBuffer"  retLen =12
#ifdef DEBUG
        if (retLen) // debug help
        {
          Serial.println("Manufacturer Data: ");
          printVal(0, manuDataBuffer, ManuDataLen);
          Serial.println();
          uint8_t *manu_data = &manuDataBuffer[2]; // remove manufacturer id
          printVal(0, manu_data, ManuDataLen - 2);
        }
#endif
      }

      // if (peripheral.advertisedServiceUuid() == AD_UUID)
      if (peripheral.address() == DEV_ADR)
      {
        BLE.stopScan();
        Serial.println("  Westy Propane: ");
        Serial.print("\taddr: \t\t");
        Serial.println(peripheral.address());

        decodeMopeka(pManuDataBuffer); // points to the third, [2], location of "manuDataBuffer"
        printResults();

        Serial.print("\tRSSI: \t\t");
        Serial.print(peripheral.rssi());
        Serial.println("dBm");
        Serial.println();
        // while (1) // peripheral disconnected, finished
        // {
        // }
      }
    }

    // BLE.scan(); // start scanning for any
    BLE.scanForAddress(DEV_ADR); // start scanning for mopeka
  }
} // end of 'loop'

void printVal(int col, u_int8_t data[], int length)
{
  int asciiCnt = 0;
  // int pos = 0;
  for (int i = 0; i < length; i++) // print hex
  {
    unsigned char b = data[i];
    if (b < 16)
    {
      Serial.print("0");
    }
    Serial.print(b, HEX);
    if (b > 0x1f && b < 0x80) // check for embedded ascii
      asciiCnt++;
    if (((i + 1) % 2 == 0) && ((i + 1) < length)) // seperater every 2 bytes (i > 0) &&
      Serial.print('-');
  }
  Serial.print(", len: ");
  Serial.print(length);

  if (asciiCnt) // ascii vals exist
  {
    Serial.print('\n');
    for (int i = 0; i < col; i++) // print ascii at hex vals column
      Serial.print(' ');

    for (int i = 0; i < length; i++) // print vals again in ascii
    {
      unsigned char b = data[i];
      if (b > 0x1f && b < 0x80)
      {
        Serial.print(' '); // sp+char because ascii is two hex digits
        Serial.print(char(b));
      }
      else
        Serial.print("  ");                         // non ascii, print two spaces
      if (((i + 1) % 2 == 0) && ((i + 1) < length)) // seperater every 2 bytes (i > 0) &&
        Serial.print(' ');
    }
  }
}
enum encodedBytes
{                 // byte# val
  sensType,       // 0     0c
  batt,           // 1     59
  depthAndTemp,   // 2     32
  depth,          // 3     00
  depthAndQuality // 4     00
};

void decodeMopeka(u_int8_t *manuData)
{
  uint16_t raw = (manuData[depthAndQuality] * 256) + manuData[depth];
  double raw_level = raw & 0x3FFF;                // mm
  double raw_t = (manuData[depthAndTemp] & 0x7F); // used in level calc
  mResults.levelMM = (raw_level * (MOPEKA_COEF[0] + MOPEKA_COEF[1] * raw_t + MOPEKA_COEF[2] * raw_t * raw_t));
  mResults.levelInches = mResults.levelMM / 24.5;
  mResults.quality_value = static_cast<SensorReadQuality>(manuData[depthAndQuality] >> 6);
  mResults.v = (float)((manuData[batt] & 0x7F) / 32.0f);
  mResults.tc = (float)(raw_t - 40);
  mResults.tf = (9.0 / 5 * mResults.tc) + 32;
}

void printResults()
{
  Serial.print("\tLevel:\t\t");
  Serial.print(mResults.levelInches, 1);
  Serial.print('\"');

  if (mResults.quality_value < QUALITY_HIGH)
  {
    Serial.print("  <-- Questionable reading");
    if (mResults.quality_value < QUALITY_MED)
    {
      Serial.print(", not to be trusted");
    }
    Serial.println('.');
  }
  else
    Serial.println();

  Serial.print("\tBattery: \t");
  Serial.print(mResults.v);
  Serial.println('V');

  Serial.print("\tTemperature: \t");
  Serial.print(mResults.tf, 1); // with one digit right of decimal pt.
  Serial.println('F');
}
