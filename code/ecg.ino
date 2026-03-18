#include "WiFi.h"
#include "WebServer.h"
#include <OpenAI.h>

#define AD_REF 3.3
#define AD_SPAN 4096
#define SAMPLE_RATE 200 // 200 Hz
#define SAMPLE_TIME 5 // 5 seconds


const char* ssid = "";
const char* password = "";
const char* api_key = "";

OpenAI openai(api_key);
OpenAI_ChatCompletion chat(openai);

long int T = 1000000 / SAMPLE_RATE;
float AD_K = float(AD_REF / AD_SPAN);
long int old_time, new_time = 0;

// Filter coefficients
float a0L, a0H, A1L, A1H, A2L, A2H, B0_lp, B0_hp, B1_lp, B1_hp, B2_lp, B2_hp;
float xn, xn_1, xn_2 = 0;
float ynL, ynL_1, ynL_2 = 0;
float ynH, ynH_1, ynH_2 = 0;

// Notch filter variables
float b0n = 1, b1n = 0, b2n = 1, a1n = 0, a2n = 0.8783;
float y2n = 0, y1n = 0, x2n = 0, x1n = 0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Calculate filter coefficients
  float omegaL = 2 * PI * 15.0 / SAMPLE_RATE;
  float alphaL = sin(omegaL) / (2 * (1 / sqrt(2)));
  a0L = 1 + alphaL;
  A1L = -(2 * cos(omegaL)) / a0L;
  A2L = (1 - alphaL) / a0L;
  B1_lp = (1 - cos(omegaL)) / a0L;
  B0_lp = B1_lp / 2;
  B2_lp = B0_lp;

  float omegaH = 2 * PI * 0.5 / SAMPLE_RATE;
  float alphaH = sin(omegaH) / (2 * (1 / sqrt(2)));
  a0H = 1 + alphaH;
  A1H = -(2 * cos(omegaH)) / a0H;
  A2H = (1 - alphaH) / a0H;
  B1_hp = -(1 + cos(omegaH)) / a0H;
  B0_hp = -B1_hp / 2;
  B2_hp = B0_hp;
}

float notch_2(float x) {
  float y = b0n * x + b1n * x1n + b2n * x2n - a1n * y1n - a2n * y2n;
  y2n = y1n; y1n = y; x2n = x1n; x1n = x;
  return y;
}

void acquire_and_analyze() {
  String ecg_data = "";
  for (int i = 0; i < SAMPLE_RATE * SAMPLE_TIME; i++) {
    new_time = micros();
    while ((new_time - old_time) < T) new_time = micros();
    old_time = new_time;
    float sensorValue0 = analogRead(34);
    float voltage0 = float(sensorValue0) * AD_K;

    xn = notch_2(sensorValue0);
    ynL = B0_lp * xn + B1_lp * xn_1 + B2_lp * xn_2 - A1L * ynL_1 - A2L * ynL_2;
    ynH = B0_hp * ynL + B1_hp * ynL_1 + B2_hp * ynL_2 - A1H * ynH_1 - A2H * ynH_2;

    xn_2 = xn_1; xn_1 = xn;
    ynL_2 = ynL_1; ynL_1 = ynL;
    ynH_2 = ynH_1; ynH_1 = ynH;

    if (i > 0) ecg_data += ",";
    ecg_data += String(ynH * AD_K);
  }
 if (ecg_data.length() > 1000) {
    ecg_data = ecg_data.substring(0, 1000);
  }
  String prompt = "Analyze the following ECG data and provide a diagnosis: " + ecg_data;
  chat.setModel("gpt-4o-mini");
  chat.setSystem("Cardiologist AI Assistant");
  chat.setMaxTokens(1000);
  chat.setTemperature(0.3);
  chat.setStop("\r");
  chat.setPresencePenalty(0);
  chat.setFrequencyPenalty(0);
  chat.setUser("OpenAI-ESP32");

  Serial.println("Sending ECG data to OpenAI for diagnosis...");
  OpenAI_StringResponse result = chat.message(prompt);

  if (result.length() > 0) {
    for (unsigned int i = 0; i < result.length(); ++i) {
      Serial.printf("Diagnosis[%u]:\n%s\n", i, result.getAt(i));
    }
  } else if (result.error()) {
    Serial.print("Error! ");
    Serial.println(result.error());
  } else {
    Serial.println("Unknown error!");
  }
}
void read_signal() {
  long int start_time = millis();
  long int duration = 5000; // Čitanje signala 1 sekundu
  int cc=0;
  while (millis() - start_time < duration) {
    new_time = micros();
    if ((new_time - old_time) >= T) {
      old_time = new_time;
      float sensorValue0 = analogRead(34);
      float voltage0 = float(sensorValue0) * AD_K;

      xn = notch_2(sensorValue0);  // notch filteread
      // lowpass
      ynL = B0_lp * xn + B1_lp * xn_1 + B2_lp * xn_2 - A1L * ynL_1 - A2L * ynL_2;

      // highpass
      ynH = B0_hp * ynL + B1_hp * ynL_1 + B2_hp * ynL_2 - A1H * ynH_1 - A2H * ynH_2;

      xn_2 = xn_1;
      xn_1 = xn;
      ynL_2 = ynL_1;
      ynL_1 = ynL;
      ynH_2 = ynH_1;
      ynH_1 = ynH;

      Serial.print(voltage0);
      Serial.print(", ");
      Serial.println(ynH*AD_K);
    }
  }
}




void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "read") {
      read_signal();
    } else if (command == "show") {
      read_signal();
    } else if (command == "analyze") {
      acquire_and_analyze();
    } else {
      Serial.println("Unknown command!");
    }
  }
}
