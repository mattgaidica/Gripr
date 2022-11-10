#include <ATSAMD21_ADC.h>
#include <LinearRegression.h>
#include <RunningAverage.h>
#include <FlashStorage_SAMD.h>
#include <ArduinoBLE.h>

BLEService griprService("A200");
BLEIntCharacteristic loadChar("A201", BLERead | BLENotify);

LinearRegression lr = LinearRegression();
double linReg[2] = { 0.172, -1.533 };
double actualLoad = 0;
uint16_t eeAddress = 0;

const int nSamples = 50;
RunningAverage myRA(nSamples);
int sampleCount = 0;

int16_t adcVal = 0;
int samplesCount = 0;

const int neg_pin = A0;  //A6;
const int pos_pin = A3;  //A5;

const int DISPLAY_MS = 50;
long int displayTime;

void setup() {
  Serial.begin(9600);
  // while (!Serial) {}

  // analogReset();
  analogGain(ADC_GAIN_16);
  analogReadExtended(15);  // 16-bit samples @ 108ms
  analogReference2(ADC_REF_INTVCC1);
  analogCalibrate();

  double eeprom_linReg[2] = { 0, 0 };
  EEPROM.get(eeAddress, eeprom_linReg[0]);
  EEPROM.get(eeAddress + sizeof(double), eeprom_linReg[1]);
  if (!isnan(eeprom_linReg[0]) & !isnan(eeprom_linReg[1])) {
    linReg[0] = eeprom_linReg[0];
    linReg[1] = eeprom_linReg[1];
  }
  lr.reset();
  lr.learn(0.0, linReg[0]);
  lr.learn(200.0, (200 * linReg[0]) + linReg[1]);  // y = m*x + b

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("BLE failed");
    // while (1) {}
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Gripr");
  BLE.setDeviceName("Gripr");
  BLE.setAdvertisedService(griprService);
  griprService.addCharacteristic(loadChar);
  BLE.addService(griprService);

  // init
  loadChar.writeValue(0);
  BLE.advertise();

  displayTime = millis();
}

// !! add adVal back into serial loop
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    while (central.connected()) {
      if (loadChar.subscribed()) {
        adcVal = analogDifferential(pos_pin, neg_pin);
        actualLoad = lr.calculate(adcVal);
        loadChar.writeValue(actualLoad);
      }
    }
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }

  if (millis() > displayTime + DISPLAY_MS) {  // !! split these up
    displayTime = millis();
    Serial.print("load:");
    Serial.print(actualLoad, 3);
    Serial.print(", ");
    Serial.println("lowLim:-30, upLim:300");
    adcVal = analogDifferential(pos_pin, neg_pin);
    actualLoad = lr.calculate(adcVal);
  }

  if (Serial.available()) {
    calibrateLoad();
  }
}

void calibrateLoad() {
  float avg;
  String str = Serial.readString();

  if (str.indexOf("cal") >= 0) {
    Serial.println("Relieve load and press enter...");
    while (Serial.available() == 0) {}
    Serial.readString();  // flush input

    lr.reset();

    myRA.clear();
    for (sampleCount = 0; sampleCount < nSamples; sampleCount++) {
      adcVal = analogDifferential(pos_pin, neg_pin);
      myRA.addValue(float(adcVal));
    }
    avg = myRA.getAverage();
    myRA.clear();
    lr.learn(avg, 0);

    Serial.println("Apply load - how many grams?");
    while (Serial.available() == 0) {}
    long loadGrams = Serial.parseInt();
    Serial.println("entered: " + String(loadGrams));

    myRA.clear();
    for (sampleCount = 0; sampleCount < nSamples; sampleCount++) {
      adcVal = analogDifferential(pos_pin, neg_pin);
      myRA.addValue(float(adcVal));
    }
    avg = myRA.getAverage();
    myRA.clear();
    lr.learn(avg, loadGrams);

    lr.parameters(linReg);
    char buffer[30];
    sprintf(buffer, "y = %1.3fx + %1.3f", linReg[0], linReg[1]);
    Serial.println(buffer);

    EEPROM.put(eeAddress, linReg[0]);
    EEPROM.put(eeAddress + sizeof(double), linReg[1]);
    EEPROM.commit();
    Serial.println("SAVED TO FLASH");

    delay(2000);
  }
}