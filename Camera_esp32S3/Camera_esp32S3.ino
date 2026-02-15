#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <husarnet.h>
#include <MAVLink.h>

#include "board_config.h"

const char *ssid = "2.4";
const char *password = "password";

const int tcpPort = 8888;
const int baudRate = 115200;

#define RX2 3
#define TX2 4

WiFiServer server(tcpPort);
WiFiClient client;


// Husarnet credentials
#define HOSTNAME "rovercam"
#define JOIN_CODE "************"

HusarnetClient husarnet;

unsigned long previousMillis1 = 0;
long LOOP1 = (1 * 1000);




void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);

  Serial2.begin(baudRate, SERIAL_8N1, RX2, TX2);






  Serial.setDebugOutput(true);
  Serial.println();


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
  config.frame_size = FRAMESIZE_UXGA;
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
  setupLedFlash();
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
    delay(1000);

  husarnet.join(HOSTNAME, JOIN_CODE);
  while (!husarnet.isJoined()) {
    Serial.println("Waiting for Husarnet network...");
    delay(1000);
  }
  Serial.println("Husarnet network joined");

  Serial.print("Husarnet IP: ");
  Serial.println(husarnet.getIpAddress().c_str());




  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
    // Check for new client connection
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New TCP Client connected!");
    }
  }

  // Case 1: Data from TCP -> Serial
  if (client.connected() && client.available()) {
    while (client.available()) {
      Serial2.write(client.read());
      Serial.print("+");
    }
  }

  // Case 2: Data from Serial -> TCP
  if (Serial2.available()) {
    while (Serial2.available()) {
      if (client.connected()) {
        client.write(Serial2.read());
      } else {
        // Clear buffer if no client is connected
        Serial2.read();
      }
    }
  }


  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= LOOP1) {
    previousMillis1 = currentMillis1;
    MAVLINK_HB();
  }
}





void MAVLINK_HB() {
  //if (FCHB > 1) {
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 1;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_ONBOARD_CONTROLLER;
  // Pack the message

  mavlink_msg_heartbeat_pack(1, 241, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  client.write(buf, len);
  Serial2.write(buf, len);
   //Serial.print("hb");
  //}
}
