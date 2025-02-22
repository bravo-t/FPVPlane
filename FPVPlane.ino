#include "esp_camera.h"
#include "driver/temperature_sensor.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include "basic_define.h"

#if USE_ESP32_PWM_API == 1
#include "driver/ledc.h"
#endif

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "FPVPlane";
const char *password = "tang1783";

extern int gpLm = D2; // Motor 1
extern int gpRm = D3; // Motor 2
extern String WiFiAddr = "";

#if USE_ESP32_PWM_API == 1
static void 
pwm_init(void)
{
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = LEDC_FREQUENCY, 
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t left_channel = {
    .speed_mode     = LEDC_MODE,
    .channel        = LEFT_MOTOR_PWM_CHANNEL,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = gpLm,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&left_channel));
  ledc_channel_config_t right_channel = {
    .speed_mode     = LEDC_MODE,
    .channel        = RIGHT_MOTOR_PWM_CHANNEL,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = gpRm,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&right_channel));
}
#endif

void startCameraServer();
void setupLedFlash(int pin);
WiFiServer server(80);

temperature_sensor_handle_t temp_handle = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

#if USE_ESP32_PWM_API == 1
  pwm_init();
#endif

  pinMode(gpLm, OUTPUT); //Left Motor
  pinMode(gpRm, OUTPUT); //Right Motor
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, 1, 0, 2);
  delay(100);
  IPAddress ip(192, 168, 1, 1);
  IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, mask);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: http://");
  Serial.println(IP);

  startCameraServer();

  WiFiAddr = IP.toString();

  ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}

int idleCounter = 0;
bool camera_active = true;
void loop() {
  // Do nothing. Everything is done in another task by the web server
  ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
  float temperature;
  ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &temperature));
  ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
  Serial.printf("Current temp: %f C\n", temperature);

  if (WiFi.softAPgetStationNum() <= 0) {
    ++idleCounter;
  } else {
    idleCounter = 0;
  }

  if (idleCounter >= 60 && camera_active) {
    Serial.printf("Put camera to sleep\n");
    esp_camera_deinit();
    camera_active = false;
  }

  delay(1000);
}
