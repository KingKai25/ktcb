#include <Arduino.h>
#include <EEPROM.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//khởi tạo chân HX711, cảm biến nhiệt độ DS18B20 và màn hình LCD, button, buzzer
constexpr int HX711_DT_PIN = 3;
constexpr int HX711_SCK_PIN = 2;
constexpr int DS18B20_PIN = A0;
constexpr int BUZZER_PIN = 7;
constexpr int UNIT_BUTTON_PIN = A1; // Chân A1 được sử dụng cho chức năng chuyển đổi đơn vị cân (gram, kg, lb, jin)
constexpr int  TZ_BUTTON_PIN = A2; // Chân A2 được sử dụng cho cả chức năng trừ bì(tare) lẫn chức năng đưa về mức 0 khi có sai lệch mặc dù không có vật đặt lên cân 
constexpr int CAL_BUTTON_PIN = A3; // Chân A3 được sử dụng cho chức năng hiệu chuẩn cân
constexpr int DEFAULT_FACTOR_ADDRESS = 0; // Địa chỉ lưu trữ hệ số hiệu chuẩn mặc định trong EEPROM các ô 0,1,2,3
constexpr int CALIB_FACTOR_ADDRESS = 4; // Địa chỉ lưu trữ hệ số hiệu chuẩn mới trong EEPROM các ô 4,5,6,7
constexpr int CALIB_FLAG_ADDRESS = 8; // Địa chỉ lưu trữ cờ hiệu chuẩn trong EEPROM ô 8, nếu giá trị là 1 thì có nghĩa là đã hiệu chuẩn, nếu giá trị là 0 thì có nghĩa là chưa hiệu chuẩn

// Khởi tạo các biến liên quan đến thông số kỹ thuật của cân: lấy mẫu 10 lần để cho ra một kết quả chứ không phải 10 hz (lấy kết quả đo 10 lần liên tiếp và tính trung bình rồi ép thành giá trị nguyên để tạo bước nhảy 1g), mức cân tối đa là 5kg nếu vượt quá thì buzzer kêu, khoảng nhiệt độ mà cảm biến loadcell bị sai số cần thuật toán bù nhiệt độ, vùng chết là những điểm dưới 1g nếu dưới 1g thì sẽ bằng 0
constexpr float DEFAULT_CALIBRATION_FACTOR = 431.23f; // Hệ số hiệu chuẩn mặc định
constexpr int MAX_WEIGHT = 5000; // Trọng lượng tối đa 5kg (5000g)  
constexpr int DEAD_ZONE = 1; // Vùng chết (g) - nếu trọng lượng của vật trong khoảng từ -1g đến 1g thì sẽ được coi là 0g để tránh việc hiển thị các giá trị nhỏ hơn 1g, đồng thời hiển thị giá trị âm khi chức năng trừ bì(tare) được sử dụng, ví dụ: nếu đặt vật 50g lên cân và nhấn nút trừ bì(tare) thì cân sẽ hiển thị -50g, nếu đặt vật 0.5g lên cân và nhấn nút trừ bì(tare) thì cân sẽ hiển thị 0g do vùng chết là 1g
constexpr float LOAD_CELL_CAPACITY_G = 5000.0f;
constexpr float ZERO_TEMPERATURE_COEFFICIENT = 0.0002f; // 0.02%RO/°C theo datasheet
constexpr float SPAN_TEMPERATURE_COEFFICIENT = 0.00003f; // 0.003%RO/°C theo datasheet
// Khởi tạo khoảng thời gian cập nhật cho màn hình, nhiệt độ, button và bước nhảy của nhiệt độ (lấy mẫu 10 lần liên tiếp trong 1s và tính trung bình rồi ép thành giá trị nguyên để tạo bước nhảy 1°C và để 1s có thể lấy mẫu 10 lần đo thì phải giảm độ phân giải của cảm biến nhiệt độ xuống 9 bit để lấy mẫu nhanh hơn, nếu để 12 bit thì phải chờ 750ms mới có kết quả đo nên không thể lấy mẫu 10 lần trong 1s được
constexpr int LCD_UPDATE_INTERVAL = 250; // Khoảng thời gian cập nhật màn hình (ms)
constexpr int TEMP_UPDATE_INTERVAL = 1000; // Khoảng thời gian cập nhật nhiệt độ (ms)
constexpr int BUTTON_DEBOUNCE_DELAY = 50; // Khoảng thời gian chống rung cho button (ms)
constexpr int TEMP_RESOLUTION = 9; // Độ phân giải của cảm biến nhiệt độ (bit) - giảm xuống 9 bit để lấy mẫu nhanh hơn

// Khởi tạo các đối tượng cho HX711, màn hình LCD, cảm biến nhiệt độ và button
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Địa chỉ I2C của màn hình LCD là 0x27, kích thước 16x2
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// Khởi tạo các biến toàn cục để lưu trữ trạng thái của cân, đơn vị đo, nhiệt độ và thời gian cập nhật
float calibration_factor = DEFAULT_CALIBRATION_FACTOR; // Hệ số hiệu chuẩn
float current_weight = 0.0f; // Trọng lượng hiện tại (g) (f thể hiện float)
float current_temperature = NAN; // Nhiệt độ hiện tại (°C)
float tare_temperature = NAN; // Nhiệt độ tại thời điểm xác lập mốc 0
unsigned long last_lcd_update = 0; // Thời gian cập nhật màn hình lần cuối (ms)
unsigned long last_temp_update = 0; // Thời gian cập nhật nhiệt độ lần cuối (ms)
unsigned long last_button_update = 0; // Thời gian đọc button lần cuối (ms)

// Biến để lưu trạng thái của các button
bool unit_button_pressed = false; // Sự kiện nhấn button chuyển đổi đơn vị cân
bool tz_button_pressed = false; // Sự kiện nhấn button trừ bì/tare
bool cal_button_pressed = false; // Sự kiện nhấn button hiệu chuẩn cân
bool unit_button_state = HIGH; // Trạng thái hiện tại của button chuyển đổi đơn vị cân
bool tz_button_state = HIGH; // Trạng thái hiện tại của button trừ bì/tare
bool cal_button_state = HIGH; // Trạng thái hiện tại của button hiệu chuẩn cân

// Biến để lưu trạng thái đơn vị đo hiện tại (gram, kg, lb, jin)
enum Unit { GRAM, KILOGRAM, POUND, JIN };
Unit current_unit = GRAM; // Đơn vị đo hiện tại, mặc định là gram

//Hàm xử lý chống nhiễu cho tất cả button để tránh việc đọc nhiều lần khi nhấn button, đảm bảo rằng chỉ có một lần nhấn được ghi nhận trong khoảng thời gian BUTTON_DEBOUNCE_DELAY
void debounceButton() {
  const unsigned long current_time = millis();
  if (current_time - last_button_update < BUTTON_DEBOUNCE_DELAY) {
    return;
  }
  last_button_update = current_time;

  const bool new_unit_state = digitalRead(UNIT_BUTTON_PIN);
  const bool new_tz_state = digitalRead(TZ_BUTTON_PIN);
  const bool new_cal_state = digitalRead(CAL_BUTTON_PIN);

  unit_button_pressed = unit_button_state == HIGH && new_unit_state == LOW;
  tz_button_pressed = tz_button_state == HIGH && new_tz_state == LOW;
  cal_button_pressed = cal_button_state == HIGH && new_cal_state == LOW;

  unit_button_state = new_unit_state;
  tz_button_state = new_tz_state;
  cal_button_state = new_cal_state;
}

//Hàm để lấy thông tin khối lượng từ cảm biến loadcell và áp dụng thuật toán bù nhiệt độ nếu nhiệt độ vượt quá ngưỡng TEMP_COMPENSATION_THRESHOLD, đồng thời áp dụng vùng chết DEAD_ZONE để tránh việc hiển thị các giá trị nhỏ hơn 1g
void updateWeight() {
  if (scale.is_ready()) {
    current_weight = scale.get_units(10); // Lấy giá trị trung bình từ 10 lần đo liên tiếp
    if (isfinite(tare_temperature) && isfinite(current_temperature)) {
      const float temperature_difference =
        current_temperature - tare_temperature;
      const float zero_drift =
        LOAD_CELL_CAPACITY_G *
        ZERO_TEMPERATURE_COEFFICIENT *
        temperature_difference;
      const float span_factor =
        1.0f + SPAN_TEMPERATURE_COEFFICIENT * temperature_difference;

      if (span_factor > 0.0f) {
        // Thử nghiệm cho thấy nhiệt độ tăng làm số đo giảm, nên cộng bù điểm 0.
        current_weight =
          (current_weight + zero_drift) / span_factor;
      }
    }
    if (fabs(current_weight) < DEAD_ZONE) {
      current_weight = 0.0f; // Loại bỏ nhiễu trong khoảng -1 g đến 1 g
    }
    current_weight = round(current_weight);
  }
}

// Hàm chuyển đổi đơn vị đo từ gram sang các đơn vị khác
float convertWeight(float weight, Unit unit) {
  switch (unit) {
    case GRAM:
      return weight; // Trọng lượng hiện tại (g)
    case KILOGRAM:
      return weight / 1000.0f; // Trọng lượng hiện tại (kg)
    case POUND:
      return weight * 0.00220462; // Trọng lượng hiện tại (lb)
    case JIN:
      return weight * 0.002; // Trọng lượng hiện tại (jin)
    default:
      return weight; // Mặc định là gram
  }
}

// Hàm cập nhật màn hình LCD với trọng lượng và nhiệt độ hiện tại
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("W: ");
  const float displayed_weight = convertWeight(current_weight, current_unit);
  lcd.print(displayed_weight, current_unit == GRAM ? 0 : 2);
  switch (current_unit) {
    case GRAM:
      lcd.print(" g");
      break;
    case KILOGRAM:
      lcd.print(" kg");
      break;
    case POUND:
      lcd.print(" lb");
      break;
    case JIN:
      lcd.print(" jin");
      break;
  }
  lcd.print("      ");
  lcd.setCursor(0, 1);
  lcd.print("T: ");
  lcd.print(current_temperature, 1); // Hiển thị nhiệt độ với 1 chữ số thập phân
  lcd.print((char)223); // Ký tự độ (°)
  lcd.print("C");
  lcd.print("      ");
}

// Hàm cập nhật nhiệt độ từ cảm biến DS18B20
void updateTemperature() {
  sensors.requestTemperatures(); // Yêu cầu cảm biến đo nhiệt độ
  const float temperature = sensors.getTempCByIndex(0);
  if (temperature != DEVICE_DISCONNECTED_C) {
    current_temperature = temperature; // Giữ giá trị trước đó khi cảm biến mất kết nối
  }
}

void tareScale() {
  if (!isfinite(current_temperature) ||
      current_temperature == DEVICE_DISCONNECTED_C) {
    return;
  }

  scale.tare();
  tare_temperature = current_temperature;
}

// Hàm để kiểm tra trạng thái cờ hiệu chuẩn trong EPROM, nếu giá trị là 1 thì có nghĩa là đã hiệu chuẩn sẽ dùng giá trị mới còn là 0 thì có nghĩa là chưa hiệu chuẩn sẽ dùng giá trị hệ số mặc định
void loadCalibrationFactor() {
  float stored_factor = DEFAULT_CALIBRATION_FACTOR;
  EEPROM.get(DEFAULT_FACTOR_ADDRESS, stored_factor);

  // EEPROM mới có thể chứa 0xFF hoặc giá trị 0, không được dùng làm hệ số chia.
  if (isnan(stored_factor) || stored_factor == 0.0f) {
    stored_factor = DEFAULT_CALIBRATION_FACTOR;
  }

  if (EEPROM.read(CALIB_FLAG_ADDRESS) == 1) {
    // Đã hiệu chuẩn, sử dụng hệ số hiệu chuẩn mới từ EEPROM
    EEPROM.get(CALIB_FACTOR_ADDRESS, calibration_factor);
    if (isnan(calibration_factor) || calibration_factor == 0.0f) {
      calibration_factor = stored_factor;
    }
  } else {
    // Chưa hiệu chuẩn, sử dụng hệ số hiệu chuẩn mặc định
    calibration_factor = stored_factor;
  }
}

// Hàm setup() để khởi tạo các thiết bị và cấu hình ban đầu
void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2); // Khởi tạo màn hình LCD 16x2
  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  loadCalibrationFactor();
  // Đọc hệ số hiệu chuẩn từ EEPROM
  scale.set_scale(calibration_factor); // Đặt hệ số hiệu chuẩn ban đầu

  lcd.backlight(); // Bật đèn nền LCD
  lcd.clear();

  sensors.begin(); // Khởi tạo cảm biến nhiệt độ DS18B20
  sensors.setResolution(TEMP_RESOLUTION); // Đặt độ phân giải của cảm biến
  do {
    updateTemperature();
    if (!isfinite(current_temperature) ||
        current_temperature == DEVICE_DISCONNECTED_C) {
      delay(100);
    }
  } while (!isfinite(current_temperature) ||
           current_temperature == DEVICE_DISCONNECTED_C);
  tareScale(); // Đặt cân về 0 và lưu nhiệt độ tham chiếu nhiệt độ

  pinMode(BUZZER_PIN, OUTPUT); // Cấu hình chân buzzer là OUTPUT
  pinMode(UNIT_BUTTON_PIN, INPUT_PULLUP); // Cấu hình chân button chuyển đổi đơn vị cân là INPUT_PULLUP
  pinMode(TZ_BUTTON_PIN, INPUT_PULLUP); // Cấu hình chân button trừ bì/tare là INPUT_PULLUP
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP); // Cấu hình chân button hiệu chuẩn cân là INPUT_PULLUP
  unit_button_state = digitalRead(UNIT_BUTTON_PIN);
  tz_button_state = digitalRead(TZ_BUTTON_PIN);
  cal_button_state = digitalRead(CAL_BUTTON_PIN);
}

// Hàm loop() để thực hiện các chức năng chính của cân
void loop() {
  unsigned long current_time = millis();

  // Cập nhật trọng lượng và nhiệt độ theo khoảng thời gian đã định
  if (current_time - last_temp_update >= TEMP_UPDATE_INTERVAL) {
    updateTemperature();
    last_temp_update = current_time;
  }

  if (current_time - last_lcd_update >= LCD_UPDATE_INTERVAL) {
    updateWeight();
    updateLCD();
    last_lcd_update = current_time;
  }

  // Xử lý các button
  debounceButton();

  // Chuyển đổi đơn vị cân khi nhấn button UNIT_BUTTON_PIN
  if (unit_button_pressed) {
    switch (current_unit) {
      case GRAM:
        current_unit = KILOGRAM;
        break;
      case KILOGRAM:
        current_unit = POUND;
        break;
      case POUND:
        current_unit = JIN;
        break;
      case JIN:
        current_unit = GRAM;
        break;
    }
    unit_button_pressed = false; // Reset trạng thái button
  }

  // Trừ bì/tare khi nhấn button TZ_BUTTON_PIN
  if (tz_button_pressed) {
    tareScale(); // Đặt cân về 0 và cập nhật nhiệt độ tham chiếu
    tz_button_pressed = false; // Reset trạng thái button
  }

  // Hiệu chuẩn cân khi nhấn button CAL_BUTTON_PIN. Người dùng đặt quả cân 100g lên bàn cân và đợi 5 giây để hệ thống tính toán ra hệ số hiệu chuẩn mới sau đó hệ số này sẽ được lưu vào bộ nhớ eprom của Arduino để sử dụng cho các lần cân tiếp theo. Nếu muốn reset về lại hệ số hiệu chuẩn mặc định thì người dùng cần nhấn giữ nút CAL_BUTTON_PIN trong 5 giây để reset về hệ số hiệu chuẩn mặc định. 
  if (cal_button_pressed) {
    const unsigned long press_start_time = millis();
    while (digitalRead(CAL_BUTTON_PIN) == LOW) {
      // Nếu nhấn giữ button trong 5 giây thì reset về hệ số hiệu chuẩn mặc định
      if (millis() - press_start_time >= 5000) {
        calibration_factor = DEFAULT_CALIBRATION_FACTOR;
        EEPROM.put(CALIB_FACTOR_ADDRESS, calibration_factor);
        EEPROM.write(CALIB_FLAG_ADDRESS, 0); // Đặt cờ hiệu chuẩn về 0 (chưa hiệu chuẩn)
        scale.set_scale(calibration_factor);
        tareScale();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Reset...");
        delay(1000); // Hiển thị thông báo trong 1 giây
        lcd.clear();
        break;
      }
    }
    // Nếu nhấn button trong thời gian ngắn hơn 5 giây thì thực hiện hiệu chuẩn cân
    if (millis() - press_start_time < 5000) {
      constexpr float known_weight = 100.0f; // Trọng lượng quả cân chuẩn là 100g
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dat qua can 100g");
      lcd.setCursor(0, 1);
      lcd.print("len ban can");
      delay(5000); // Chờ 5 giây để người dùng đặt quả cân chuẩn lên bàn cân
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dang hieu chuan...");
      const double raw_average = scale.is_ready()
        ? scale.get_value(10)
        : 0.0; // Lấy giá trị raw trung bình sau khi đã đặt quả cân
      const float new_calibration_factor =
        static_cast<float>(raw_average / known_weight);
      const float measured_weight =
        calibration_factor != 0.0f
          ? static_cast<float>(raw_average / calibration_factor)
          : 0.0f;
      const float minimum_calibration_weight = known_weight * 0.5f;
      if (isfinite(raw_average) &&
          isfinite(new_calibration_factor) &&
          fabs(measured_weight) >= minimum_calibration_weight) {
        calibration_factor = new_calibration_factor;
        // Factor = raw trung bình / khối lượng chuẩn.
        EEPROM.put(CALIB_FACTOR_ADDRESS, calibration_factor);
        EEPROM.write(CALIB_FLAG_ADDRESS, 1); // Đặt cờ hiệu chuẩn về 1 (đã hiệu chuẩn)
        scale.set_scale(calibration_factor);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Hieu chuan xong");
        delay(2000); // Hiển thị thông báo trong 2 giây
        lcd.clear();
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Loi: Khong co "); // Hiển thị thông báo lỗi nếu không có quả cân chuẩn
        lcd.setCursor(0, 1);
        lcd.print("qua can 100g");
        delay(2000); // Hiển thị thông báo lỗi trong 2 giây
        lcd.clear();
      }
    }
    cal_button_pressed = false; // Reset trạng thái button
  }

  // Kiểm tra trọng lượng vượt quá mức tối đa và kích hoạt buzzer nếu cần
  if (current_weight > MAX_WEIGHT) {
    digitalWrite(BUZZER_PIN, HIGH); // Bật buzzer
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Tắt buzzer
  }
}
