#include <ATSAMD21_ADC.h>
#include <LinearRegression.h>
#include "RunningAverage.h"

LinearRegression lr = LinearRegression();
double linReg[2];
double actualLoad = 0;

const int nSamples = 50;
RunningAverage myRA(nSamples);
int sampleCount = 0;

int16_t adcVal = 0;
float b;
int samplesCount = 0;

const int neg_pin = A0;  //A6;
const int pos_pin = A3;  //A5;

const int DISPLAY_MS = 50;
long int displayTime;

void setup() {
  Serial.begin(9600);
  delay(1000);
  // analogReset();
  analogGain(ADC_GAIN_16);
  analogReadExtended(15);  // 16-bit samples @ 108ms
  analogReference2(ADC_REF_INTVCC1);
  analogCalibrate();

  // calibrate load sensor
  // myRA.clear();
  // for (sampleCount = 0; sampleCount < nSamples; sampleCount++) {
  //   adcVal = analogDifferential(pos_pin, neg_pin);
  //   myRA.addValue(float(adcVal));
  // }
  // b = myRA.getAverage();
  // myRA.clear();

  // calibrateLoad();

  displayTime = millis();
}

void loop() {
  adcVal = analogDifferential(pos_pin, neg_pin);
  myRA.addValue(float(adcVal));
  if (sampleCount > nSamples) {
    actualLoad = lr.calculate(myRA.getAverage());
    myRA.clear();
  }
  sampleCount++;

  if (millis() > displayTime + DISPLAY_MS) { // !! split these up
    displayTime = millis();
    Serial.print("300, -300, ");
    // Serial.print(String(myRA.getAverage()) + ", ");
    Serial.println(actualLoad, 3);
  }

  if (Serial.available()) {
    calibrateLoad();
  }
}

void calibrateLoad() {
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
    b = myRA.getAverage();
    myRA.clear();
    Serial.println(b);
    lr.learn(b, 0);

    Serial.println("Apply load - how many grams?");
    while (Serial.available() == 0) {}
    long loadGrams = Serial.parseInt();
    Serial.println("entered: " + String(loadGrams));

    myRA.clear();
    for (sampleCount = 0; sampleCount < nSamples; sampleCount++) {
      adcVal = analogDifferential(pos_pin, neg_pin);
      myRA.addValue(float(adcVal));
    }
    b = myRA.getAverage();
    myRA.clear();
    Serial.println(b);
    lr.learn(b, loadGrams);

    lr.parameters(linReg);
    char buffer[30];
    sprintf(buffer, "y = %1.1fx + %1.1f", linReg[0], linReg[1]);
    Serial.println(buffer);

    delay(2000);
  }
}