#include "HX711.h"

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

HX711 scale;
#include "Arduino.h"
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 16, 2); // Tạo đối tượng LCD với địa chỉ 0x27 và kích thước 16x2

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  lcd.init(); // Khởi tạo màn hình LCD
  lcd.backlight(); // Bật đèn nền LCD
  lcd.clear(); // Xóa màn hình
  lcd.setCursor(0, 0);
  lcd.print("=== BAT DAU QUA TRINH TIM HE SO HIEU CHUAN ===");
  
  // Bước 1: Đưa cân về 0 (Chưa đặt gì lên bàn cân)
  Serial.println("1. De trong ban can de lay diem 0 (Tare)...");
  delay(3000); // Chờ 3 giây để bạn bỏ hết đồ ra
  scale.set_scale(); // Đặt hệ số chia mặc định là 1
  scale.tare();      // Reset cân về 0
  Serial.println("-> Da lay xong diem 0!");
  Serial.println("----------------------------------------------");

  // Bước 2: Đặt quả cân chuẩn lên
  Serial.println("2. BAY GIO, hay dat qua can chuan (100g) len ban can.");
  Serial.println("He thong se cho 5 giay de ban dat qua can va on dinh...");
  delay(5000); // Chờ 5 giây để bạn đặt quả cân lên phao cân

  // Bước 3: Thuật toán dùng biến f để gom dữ liệu 10 lần liên tiếp nhằm tìm giá trị chuẩn
  Serial.println("-> Dang lay mau 10 lan lien tiep...");
  
  float f = 0; // Biến f dùng để tích lũy giá trị thô
  int samples = 0;

  // Vòng lặp trong setup thu thập đủ 10 mẫu thực tế
  while (samples < 10) {
    if (scale.is_ready()) {
      f += scale.get_value(1); // Lấy giá trị thô (chưa chia) cộng dồn vào f
      samples++;
      delay(100); // Giãn cách nhẹ giữa các lần đọc
    }
  }

  // Tính trung bình cộng của 10 lần đọc thô liên tiếp
  float gia_tri_raw_trung_binh = f / 10.0;

  // Bước 4: Tính toán hệ số hiệu chuẩn (Calibration Factor)
  // Công thức: Factor = Giá trị thô trung bình / Khối lượng quả cân mẫu (100g)
  float khoi_luong_qua_can = 226.0; 
  float he_so_chuan_tim_duoc = gia_tri_raw_trung_binh / khoi_luong_qua_can;

  // Xuất con số cuối cùng ra màn hình máy tính
  Serial.println("----------------------------------------------");
  Serial.print("SUCCESS! He so hieu chuan (Calibration Factor) cua ban la: ");
  Serial.println(he_so_chuan_tim_duoc);
  Serial.println("Hay copy con so nay de dien vao code chinh thuc cua ban.");
  Serial.println("=== KET THUC ===");
}

void loop() {
  // Không cần viết gì ở đây, ta chỉ dùng setup() để tìm con số cố định
}
