#include <Arduino.h>
#include <HX711.h>
#include <OneWire.h>
#include <DallasTemperature.h>

constexpr int HX711_DT_PIN = 3;
constexpr int HX711_SCK_PIN = 2;
constexpr int DS18B20_PIN = A0;

constexpr float CALIBRATION_FACTOR = 431.23f;
constexpr float LOAD_CELL_CAPACITY_G = 5000.0f;
constexpr float REFERENCE_TEMPERATURE_C = 25.0f;
constexpr float SPAN_TEMPERATURE_COEFFICIENT = 0.00003f;
constexpr float ZERO_TEMPERATURE_COEFFICIENT = 0.0002f;

constexpr unsigned long SAMPLE_INTERVAL_MS = 1000;
constexpr byte HX711_SAMPLES = 10;
constexpr byte TEMPERATURE_RESOLUTION = 9;

HX711 scale;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

unsigned long last_sample_time = 0;
float current_temperature = NAN;
float tare_temperature = NAN;
long tare_raw = 0;
bool tare_ready = false;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  t = tare lai khi ban can trong"));
  Serial.println(F("  h = hien thi huong dan"));
  Serial.println(F("Quy trinh: tare trong -> dat cung mot vat -> dung may say"));
  Serial.println(F("Khong huong may say qua gan hoac qua lau vao loadcell."));
}

void tareEmptyScale() {
  if (!scale.is_ready()) {
    Serial.println(F("ERROR,HX711_NOT_READY"));
    return;
  }

  tare_raw = scale.read_average(20);
  scale.set_offset(tare_raw);
  scale.set_scale(CALIBRATION_FACTOR);
  tare_ready = true;
  tare_temperature = current_temperature;

  Serial.print(F("EVENT,TARE,temp_c="));
  Serial.print(tare_temperature, 2);
  Serial.print(F(",raw="));
  Serial.println(tare_raw);
}

void updateTemperature() {
  sensors.requestTemperatures();
  const float temperature = sensors.getTempCByIndex(0);
  if (temperature != DEVICE_DISCONNECTED_C) {
    current_temperature = temperature;
  }
}

void printSample() {
  if (!tare_ready || !scale.is_ready()) {
    return;
  }

  const long raw_average = scale.read_average(HX711_SAMPLES);
  const long raw_delta = raw_average - tare_raw;
  const float measured_weight =
    static_cast<float>(raw_delta) / CALIBRATION_FACTOR;

  const float temperature_difference =
    current_temperature - REFERENCE_TEMPERATURE_C;
  const float span_factor =
    1.0f + SPAN_TEMPERATURE_COEFFICIENT * temperature_difference;
  const float zero_drift =
    LOAD_CELL_CAPACITY_G *
    ZERO_TEMPERATURE_COEFFICIENT *
    temperature_difference;

  const float span_only_weight =
    span_factor > 0.0f ? measured_weight / span_factor : NAN;
  const float zero_minus_weight =
    span_factor > 0.0f
      ? (measured_weight - zero_drift) / span_factor
      : NAN;
  const float zero_plus_weight =
    span_factor > 0.0f
      ? (measured_weight + zero_drift) / span_factor
      : NAN;

  Serial.print(F("DATA,time_ms="));
  Serial.print(millis());
  Serial.print(F(",temp_c="));
  Serial.print(current_temperature, 2);
  Serial.print(F(",raw_avg="));
  Serial.print(raw_average);
  Serial.print(F(",raw_delta="));
  Serial.print(raw_delta);
  Serial.print(F(",measured_g="));
  Serial.print(measured_weight, 2);
  Serial.print(F(",span_only_g="));
  Serial.print(span_only_weight, 2);
  Serial.print(F(",zero_minus_g="));
  Serial.print(zero_minus_weight, 2);
  Serial.print(F(",zero_plus_g="));
  Serial.println(zero_plus_weight, 2);
}

void setup() {
  Serial.begin(115200);
  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);

  sensors.begin();
  sensors.setResolution(TEMPERATURE_RESOLUTION);
  updateTemperature();

  Serial.println(F("Temperature compensation test"));
  Serial.println(F("Keep the scale empty and stable for 5 seconds."));
  delay(5000);
  tareEmptyScale();
  printHelp();
  Serial.println(F("CSV: DATA,time_ms,temp_c,raw_avg,raw_delta,measured_g,span_only_g,zero_minus_g,zero_plus_g"));
}

void loop() {
  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 't' || command == 'T') {
      tareEmptyScale();
    } else if (command == 'h' || command == 'H') {
      printHelp();
    }
  }

  const unsigned long current_time = millis();
  if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
    last_sample_time = current_time;
    updateTemperature();
    printSample();
  }
}
