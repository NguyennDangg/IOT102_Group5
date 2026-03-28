#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/i2s.h>
#include <IRremote.h>
// #include <IRremote.hpp>
// #define IR_RECEIVE_PIN 17

// --- Cấu hình chân Pin ---
// - remote -
const int RECV_PIN = 17;
IRrecv irrecv(RECV_PIN);
decode_results results;

const int PIN_R = 5, PIN_G = 18, PIN_B = 19;
const int LED_R = 27, LED_G = 16, LED_B = 14;
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int I2S_SD = 33, I2S_WS = 25, I2S_SCK = 26;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// chỉnh threshold thực tế
const long ENVIRONMENT_THRESHOLD = 25000;
const long THRESHOLD_AMP = 90000;
const long THRESHOLD_DELTA = 90000;

// --- Biến điều khiển hệ thống ---
volatile int currentMode = 0; 
int brightness = 255;
int lastR = 255, lastG = 255, lastB = 255;

// --- Biến cho Mode 3 (Clap Control) ---
unsigned long lastClapDetected = 0; 
int clapCount = 0;                  
int colorState = 0;                
//  4 màu cơ bản (đỏ, xanh lá, xanh dương, tắt, có 8 màu nhưng LED dỏm)
const int colors[6][3] = {
  {100,0,0},    // red
  {0,100,0},    // green
  {0,0,100},    // blue
  {127,0,127},       // purple
  {127,127,127},       // white
  {0,127,127},       // cyan
};
const unsigned long CLAP_WINDOW = 500;   
const unsigned long DEBOUNCE_TIME = 200; 

// --- Hàm bổ trợ phần cứng ---
void forceRGB(int r, int g, int b) {
  ledcWrite(0, r);
  ledcWrite(1, g);
  ledcWrite(2, b);
}

// --- Bluetooth Callback ---
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        char cmd = value[0];
        // Cập nhật điều kiện để nhận được Mode '4'
        if (cmd >= '0' && cmd <= '4') { 
          currentMode = cmd - '0';
          clapCount = 0; // Reset lại bộ đếm khi đổi mode
          Serial.printf("\n>>> CHUYEN MODE: %d\n", currentMode);
        }
      }
    }
};

// --- Cấu hình I2S (Microphone) ---
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // FIX - Ban đầu I2S_COMM_FORMAT_I2S
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  esp_err_t err;

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  Serial.println(err == ESP_OK ? "I2S driver OK" : "I2S driver FAIL");

  err = i2s_set_pin(I2S_PORT, &pin_config);
  Serial.println(err == ESP_OK ? "I2S pin OK" : "I2S pin FAIL");
}

long getAmplitude() {
  int32_t samples[64];
  size_t bytes_read;

  i2s_read(I2S_PORT, &samples, sizeof(samples), &bytes_read, portMAX_DELAY);

  long maxAmp = 0;

  for (int i = 0; i < bytes_read / 4; i++) {
    long s = abs(samples[i] >> 11);  // scale hợp lý
    if (s > maxAmp) maxAmp = s;
  }

  return maxAmp;
}

// --- Hue - Sat - V ... thành RGB - hiệu ứng thở ---
/**
void hsvToRgb(float h, float s, float v, int &r, int &g, int &b) {
  float c = v * s;
  float x = c * (1 - abs(fmod(h / 60.0, 2) - 1));
  float m = v - c;

  float r1, g1, b1;

  if (h < 60) { r1 = c; g1 = x; b1 = 0; }
  else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
  else { r1 = c; g1 = 0; b1 = x; }

  r = (r1 + m) * 255;
  g = (g1 + m) * 255;
  b = (b1 + m) * 255;
}

void modeRainbow() {
  static float hue = 0;
  static unsigned long lastUpdate = 0;

  int r, g, b;

  // tốc độ đổi màu (có thể chỉnh)
  if (millis() - lastUpdate > 30) {
    lastUpdate = millis();

    hsvToRgb(hue, 1.0, 1.0, r, g, b);
    forceRGB(255 - r, 255 - g, 255 - b);

    hue += 0.5;   // tốc độ chạy màu
    if (hue >= 360) hue = 0;
  }
}
*/

// --- Logic Mode 1: cầu vồng ---
void modeRainbow() {
  static float t = 0;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate < 55) return;
  lastUpdate = millis();

  int r = (sin(t) + 1) * 127;
  int g = (sin(t + 2) + 1) * 127;
  int b = (sin(t + 4) + 1) * 127;

  forceRGB(r, g, b);

  t += 0.1;
}

// --- Logic Mode 2: chưa có ---
void modeSoundReactive(){
    // long amp = getAmplitude();
    // // sáng từ 1 → 3 led theo độ lớn âm thanh: chưa gắn 3 led vào mạch điện / board
    // if (amp < 200) forceRGB(0, 15, 0);        
    // else if (amp < 1000) forceRGB(0, 0, 255);       
    // else forceRGB(255, 0, 0);

  static long smoothAmp = 0;

  long amp = getAmplitude();

  // làm mượt tín hiệu (VERY IMPORTANT)
  smoothAmp = 0.7 * smoothAmp + 0.3 * amp;

  // debug
  Serial.printf("amp=%ld smooth=%ld\n", amp, smoothAmp);

  if (smoothAmp < ENVIRONMENT_THRESHOLD * 0.6) {
    // GREEN
    forceRGB(0, 127, 0);
  }
  else if (smoothAmp < ENVIRONMENT_THRESHOLD) {
    // CYAN 
    forceRGB(0, 127, 127);
  }
  else {
    // RED
    forceRGB(127, 0, 0);
  }
}
    
// --- Logic Mode 3: Vỗ tay đổi màu ---
void modeClapControl() {
  long amp = getAmplitude();
  unsigned long now = millis();

  static long lastAmp = 0;
  long delta = abs(amp - lastAmp);

  if (
    amp > THRESHOLD_AMP &&         // chỉnh lại theo môi trường
    delta > THRESHOLD_DELTA &&       // spike mạnh
    (now - lastClapDetected > 250)
  ) {
    clapCount++;
    lastClapDetected = now;

    Serial.printf("👏 Clap detected! amp=%ld delta=%ld\n", amp, delta);
  }

  lastAmp = amp;

  // xử lý sau window
  if (clapCount > 0 && (now - lastClapDetected > 400)) {

    if (clapCount == 1) {
      colorState = (colorState + 1) % 6;
      forceRGB(colors[colorState][0], colors[colorState][1], colors[colorState][2]);
    }

    else if (clapCount == 2) {
      forceRGB(0,0,0);
    }

    clapCount = 0;
  }
}

// --- Logic Mode 4: Mô phỏng ... ---
void modePulse() {
  static float t = 0;

  int brightness = (sin(t) + 1) * 127;

  forceRGB(brightness, 0, brightness);

  t += 0.05;
}

// --- Khởi tạo hệ thống ---
void setup() {
  Serial.begin(115200);

  // cho remote
  //IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  irrecv.enableIRIn();

  //
  // Setup PWM channels
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PIN_R, 0);

  ledcSetup(1, 5000, 8);
  ledcAttachPin(PIN_G, 1);

  ledcSetup(2, 5000, 8);
  ledcAttachPin(PIN_B, 2);

  // Cấu hình BLE
  BLEDevice::init("ESP32_Smart_LED");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEDevice::getAdvertising()->start();

  Serial.println("STEP 1");

  setupI2S();

  Serial.println("STEP 2");  // Nếu không thấy dòng này → lỗi I2S
  Serial.println("HE THONG DA SAN SANG!");
}


// --- Test IrReceiver (Remote) ---
// void handleIR() {
//   //   cũ
//   if (irrecv.decode(&results)) {
//     if (results.value != 0 && results.value != 0xFFFFFFFF) {
//       Serial.print("IR: ");
//       Serial.print(results.value, HEX);
//       Serial.println(results.value);
//     }
//     irrecv.resume();
//   }

//   // code mới (library từ 3. trở lên)
//   // if (IrReceiver.decode()) {
//   //   unsigned long code = IrReceiver.decodedIRData.decodedRawData;

//   //   Serial.print("IR: ");
//   //   Serial.println(code, HEX);
//   //   Serial.println(code, DEC);
//   //   Serial.println(code);

//   //   handleRemote(code);  // 👈 tách ra xử lý riêng

//   //   IrReceiver.resume();
//   // }
// }

void handleRemote() { //unsigned long code
  if (irrecv.decode(&results)) {
    if (results.value != 0 && results.value != 0xFFFFFFFF) {
      Serial.print("IR: ");
      Serial.println(results.value, HEX);
      //Serial.println(results.value);
    
    switch(results.decode_type) { // code

      // ===== HÀNG 1 =====
      case 0xF7C03F16236607: // ON
        forceRGB(lastR, lastG, lastB);
        break;

      case 0xF740BF16203967: // OFF
        forceRGB(0,0,0);
        break;

      case 0xF700FF16187647: // Bright +
        brightness = min(255, brightness + 20);
        break;

      case 0xF7807F16220287: // Bright -
        brightness = max(0, brightness - 20);
        break;

      // ===== HÀNG 2 =====
      case 0xF720DF16195807: forceRGB(255,0,0); break; // Red
      case 0xF7A05F16228447: forceRGB(0,255,0); break; // Green
      case 0xF7609F16212127: forceRGB(0,0,255); break; // Blue
      case 0xF7E01F16244767: forceRGB(127,127,127); break; // White

      // ===== HÀNG 4 =====
      case 0xF730CF16199887: forceRGB(127,127,0); break; // Orange
      case 0xF7B04F16232527: forceRGB(0,127,127); break; // Cyan
      case 0xF7708F16216207: forceRGB(127,0,127); break; // Purple

      // ===== MODE =====
      case 0xF7D02F16240687: currentMode = 2; break; // FLASH
      case 0xF7F00F16248847: currentMode = 3; break; // STROBE
      case 0xF7C83716238647: currentMode = 4; break; // FADE
      case 0xF7E81716246807: currentMode = 1; break; // SMOOTH

      default:
        // các nút khác
        forceRGB(0,0,0);
        delay(1000);
        forceRGB(lastR, lastG, lastB);
        break;
    }
    irrecv.resume();
  }
}}

// --- Vòng lặp chính ---
void loop() {
  handleRemote();

  switch(currentMode) {
    case 1:
      modeRainbow();
      break;

    case 2:
      modeSoundReactive();
      break;

    case 3:
      modeClapControl();
      break;

    case 4:
      modePulse();
      break;

    default:
      forceRGB(0,127,127);
  }
  yield(); 
}