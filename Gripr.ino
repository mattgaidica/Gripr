#include <ATSAMD21_ADC.h>
// #include <LinearRegression.h>
#include "RunningAverage.h"

const int nSamples = 50;
RunningAverage myRA(nSamples);
int sampleCount = 0;

int16_t adcVal = 0;
float mx, b;
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
  analogReadExtended(15); // 16-bit samples @ 108ms
  analogReference2(ADC_REF_INTVCC1);
  analogCalibrate();

  // calibrate load sensor
  myRA.clear();
  for (sampleCount = 0; sampleCount < nSamples; sampleCount++) {
    adcVal = analogDifferential(pos_pin, neg_pin);
    myRA.addValue(float(adcVal));
  }
  b = myRA.getAverage();
  myRA.clear();

  displayTime = millis();
}

void loop() {
  adcVal = analogDifferential(pos_pin, neg_pin);
  myRA.addValue(float(adcVal) - b);
  sampleCount++;

  if (millis() > displayTime + DISPLAY_MS & sampleCount > nSamples) {
    displayTime = millis();
    Serial.print("1000, -1000, ");
    Serial.println(myRA.getAverage(), 3);
    myRA.clear();
  }
}