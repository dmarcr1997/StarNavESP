#include <TJpg_Decoder.h>
#include <Arduino.h>

#include <SPI.h>

#define TFT_SCLK D8
#define TFT_MISO D9
#define TFT_MOSI D10
#define TFT_CS   D1
#define TFT_DC   D3
#define TFT_BL   D6
#define TFT_RST  -1
#define TFT_WIDTH  240
#define TFT_HEIGHT 240

#include "esp_camera.h"
#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

enum DeviceState {
  LIVE_FEED,
  CAPTURING,
  INFERENCING,
  SHOW_RESULTS
};

volatile DeviceState state = LIVE_FEED;
volatile int frames_captured = 0;

static const int STAR_W = 96;
static const int STAR_H = 96;
float accum_buffer[STAR_W * STAR_H];
uint8_t ei_input[STAR_W * STAR_H]; 
const int LONG_EXP_FRAMES = 48;
String last_constellation = "";
float last_confidence = 0.0f;

bool init_camera_rgb565_240() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;   // note: sccb, not sscb
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  // *** THIS IS THE KEY CHANGE FROM THE WEB EXAMPLE ***
  config.pixel_format = PIXFORMAT_RGB565;    // we want RGB565, not JPEG
  config.frame_size   = FRAMESIZE_QQVGA; //FRAMESIZE_UXGA;      // start big, example does this too
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 50;                 // ignored for RGB565
  config.fb_count     = 1;

  // This block is straight from the example, just with pixel_format changed above
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count     = 2;
      config.grab_mode    = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size   = FRAMESIZE_SVGA;
      config.fb_location  = CAMERA_FB_IN_DRAM;
    }
  } else {
    // *** For non-JPEG (RGB565 / grayscale), the example forces 240x240 here ***
    config.frame_size = FRAMESIZE_240X240;
  #if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
  #endif
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // tweak as needed to orient correctly
    s->set_vflip(s, 1);     // try 0/1
    s->set_hmirror(s, 1);   // try 0/1
  }

  return true;
}
bool init_camera_jpeg_240() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size   = FRAMESIZE_240X240;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 2;
  config.jpeg_quality = 10;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  }

  return true;
}
bool init_camera_gray_96() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_96X96;       // matches STAR_W/H
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Gray camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  }
  return true;
}

bool switchToGrayCam() {
  esp_camera_deinit();
  delay(50);
  return init_camera_gray_96();
}

bool switchToJpegCam() {
  esp_camera_deinit();
  delay(50);
  return init_camera_jpeg_240();
}

inline void lcd_select()   { digitalWrite(TFT_CS, LOW); }
inline void lcd_deselect() { digitalWrite(TFT_CS, HIGH); }

void lcd_cmd(uint8_t c) {
  digitalWrite(TFT_DC, LOW);
  lcd_select();
  SPI.transfer(c);
  lcd_deselect();
}

void lcd_data(const uint8_t *data, size_t len) {
  digitalWrite(TFT_DC, HIGH);
  lcd_select();
  while (len--) {
    SPI.transfer(*data++);
  }
  lcd_deselect();
}

void lcd_data8(uint8_t d) {
  lcd_data(&d, 1);
}

void lcd_reset() {
  if (TFT_RST < 0) {
    delay(50);
    return;
  }
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(20);
}

void lcd_init_gc9a01() {
  lcd_reset();

  lcd_cmd(0xFE);
  lcd_cmd(0xEF);

  lcd_cmd(0xEB); lcd_data8(0x14);
  lcd_cmd(0xFE);
  lcd_cmd(0xEF);

  lcd_cmd(0x84); lcd_data8(0x40);
  lcd_cmd(0x85); lcd_data8(0xFF);
  lcd_cmd(0x86); lcd_data8(0xFF);
  lcd_cmd(0x87); lcd_data8(0xFF);

  lcd_cmd(0x88); lcd_data8(0x0A);
  lcd_cmd(0x89); lcd_data8(0x21);
  lcd_cmd(0x8A); lcd_data8(0x00);
  lcd_cmd(0x8B); lcd_data8(0x80);
  lcd_cmd(0x8C); lcd_data8(0x01);
  lcd_cmd(0x8D); lcd_data8(0x01);

  lcd_cmd(0x8E); lcd_data8(0xFF);
  lcd_cmd(0x8F); lcd_data8(0xFF);

  lcd_cmd(0xB6); lcd_data8(0x00); lcd_data8(0x00);

  // ---- COLOR ORDER / ROTATION ----
  lcd_cmd(0x36); 
  lcd_data8(0x68); 

  // ---- PIXEL FORMAT ----
  lcd_cmd(0x3A);
  lcd_data8(0x55);  // 16-bit 565, standard format

  lcd_cmd(0x90);
  { uint8_t d[] = {0x08, 0x08, 0x08, 0x08}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xBD); lcd_data8(0x06);
  lcd_cmd(0xBC); lcd_data8(0x00);

  lcd_cmd(0xFF);
  { uint8_t d[] = {0x60, 0x01, 0x04}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xC3); lcd_data8(0x13);
  lcd_cmd(0xC4); lcd_data8(0x13);

  lcd_cmd(0xC9); lcd_data8(0x22);

  lcd_cmd(0xBE); lcd_data8(0x11);

  lcd_cmd(0xE1); // VCOM
  lcd_data8(0x10); 
  lcd_data8(0x0E);

  lcd_cmd(0xDF);
  { uint8_t d[] = {0x21, 0x0C, 0x02}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xF0);
  { uint8_t d[] = {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xF1);
  { uint8_t d[] = {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xF2);
  { uint8_t d[] = {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xF3);
  { uint8_t d[] = {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xED);
  { uint8_t d[] = {0x1B, 0x0B}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xAE); lcd_data8(0x77);
  lcd_cmd(0xCD); lcd_data8(0x63);

  lcd_cmd(0x70);
  { uint8_t d[] = {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0xE8); lcd_data8(0x34);

  lcd_cmd(0x62);
  { uint8_t d[] = {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x63);
  { uint8_t d[] = {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x64);
  { uint8_t d[] = {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x66);
  { uint8_t d[] = {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x67);
  { uint8_t d[] = {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x74);
  { uint8_t d[] = {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x98);
  { uint8_t d[] = {0x3E, 0x07}; lcd_data(d, sizeof(d)); }

  lcd_cmd(0x21); // INVOFF
  delay(120);

  // Sleep out, display on
  lcd_cmd(0x11);  // SLP OUT
  delay(120);
  lcd_cmd(0x29);  // DISP ON
  delay(20);
}

void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t data[4];

  lcd_cmd(0x2A);  // column
  data[0] = x0 >> 8; data[1] = x0 & 0xFF;
  data[2] = x1 >> 8; data[3] = x1 & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(0x2B);  // row
  data[0] = y0 >> 8; data[1] = y0 & 0xFF;
  data[2] = y1 >> 8; data[3] = y1 & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(0x2C);  // RAMWR
}

void lcd_fill_color(uint16_t color565) {
  lcd_set_addr_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

  uint8_t hi = color565 >> 8;
  uint8_t lo = color565 & 0xFF;

  digitalWrite(TFT_DC, HIGH);
  lcd_select();

  for (uint32_t i = 0; i < (uint32_t)TFT_WIDTH * TFT_HEIGHT; i++) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }

  lcd_deselect();
}

void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color565) {
  if(x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT) return;

  lcd_set_addr_window(x, y, x, y);

  uint8_t hi = color565 >> 8;
  uint8_t lo = color565 & 0xFF;

  digitalWrite(TFT_DC, HIGH);
  lcd_select();
  SPI.transfer(hi);
  SPI.transfer(lo);
  lcd_deselect();
}

void lcd_push_image(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *data) {
  if( w <= 0 || h <= 0) return;
  lcd_set_addr_window(x, y, x + w - 1, y + h - 1);

  digitalWrite(TFT_DC, HIGH);
  lcd_select();
  int32_t count = (int32_t)w * (int32_t)h;
  while (count--) {
    uint16_t color = *data++;
    SPI.transfer(color >> 8);
    SPI.transfer(color & 0xFF);
  }
  lcd_deselect();
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  lcd_push_image(x, y, w, h, (const uint16_t*)bitmap);
  return true;
}

bool screenPressed() {
  return false;
}

void startLongExposureCapture() {
  if (!switchToGrayCam()) {
    Serial.println("Failed to switch to gray mode");
    return;
  }
  for(int i = 0; i< STAR_W * STAR_H; i++) {
    accum_buffer[i] = 0.0f;
  }
  frames_captured = 0;

  lcd_fill_color(0x0000);
  Serial.println("Starting long exposure capture");
}
void finalizeLongExposure() {
  float maxVal = 0.0f;
  for (int i = 0; i < STAR_W * STAR_H; i++) {
    if (accum_buffer[i] > maxVal) maxVal = accum_buffer[i];
  }
  if (maxVal < 1e-3f) maxVal = 1.0f;
  for (int i = 0; i < STAR_W * STAR_H; i++) {
    float norm = accum_buffer[i] / maxVal;
    if (norm > 1.0f) norm = 1.0f;
    ei_input[i] = (uint8_t)(norm * 255.0f);
  }
}

void accumulateFromGray(camera_fb_t *fb) {
  int w = fb->width;
  int h = fb->height;
  if (w != STAR_W || h != STAR_H) {
    Serial.printf("Unexpected gray frame size: %dx%d\n", w, h);
    return;
  }
  const uint8_t *src = fb->buf; // each byte is 0–255
  for (int i = 0; i < STAR_W * STAR_H; i++) {
    accum_buffer[i] += (float)src[i];
  }
}
void handleCapture() {
  if(frames_captured >= LONG_EXP_FRAMES) {
    finalizeLongExposure();
    state = INFERENCING;
    return;
    
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed during long exposure");
    return;
  }
  accumulateFromGray(fb);

  esp_camera_fb_return(fb);

  frames_captured++;
}
void handleLiveFeed() {
  if(screenPressed()) {
    startLongExposureCapture();
    state = CAPTURING;
    return;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  TJpgDec.drawJpg(0, 0, fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}
void handleInference() {
  //TODO: NEED TO TRAIN EDGE IMPULSE MODEL
  last_constellation = "Orion";
  last_confidence = 0.87f;
  state = SHOW_RESULTS;
}
void showResults() {
//beige screen
//left hand corner draw [last_constellation]
//middle center draw Confidence: num
//TODO add directionality, north, ect based on this
  if (screenPressed()) {
      frames_captured = 0;
      if (!switchToJpegCam()) {
        Serial.println("Failed to switch back to JPEG mode");
        return;
      }
      state = LIVE_FEED;
  }
}
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nXIAO S3 Sense + Round Display: Camera Preview");

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_BL, OUTPUT);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_BL, HIGH);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  SPI.setFrequency(40000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  lcd_init_gc9a01();
  lcd_fill_color(0x0000); // black
  TJpgDec.setCallback(tft_output);
  if (!init_camera_jpeg_240()) {
    lcd_fill_color(0xF800); // red screen if camera failed
    while (true) {
      delay(1000);
    }
  }

  // Brief test flash
  lcd_fill_color(0x07E0); // green
  delay(300);
  lcd_fill_color(0x0000); // black
}

void loop() {
  switch(state) {
    case LIVE_FEED:
      handleLiveFeed();
      break;
    case CAPTURING:
      handleCapture();
      break;
    case INFERENCING:
      handleInference();
      break;
    case SHOW_RESULTS:
      showResults();
      break;
  }
}