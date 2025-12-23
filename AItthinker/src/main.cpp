#include "esp_camera.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // Add ArduinoJson library
#include <time.h>        // NTP time functions

// Edge Impulse model header
#include <Road-Defect-ESP32CAM_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
// Select your camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#define LED_PIN 4 // onboard flash LED

// Wi-Fi credentials
const char *ssid = "Aroosh 1st Floor";
const char *password = "aroosh0000";

// MQTT broker details
const char *mqtt_server = "n12a20fa.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;

// CA certificate from EMQX broker
const char *ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

WiFiClientSecure net;
PubSubClient client(net);

AsyncWebServer server(80);
AsyncEventSource events("/events");

unsigned long lastTime = 0;
unsigned long timerDelay = 10000; // 10 seconds

uint8_t *snapshot_buf; // buffer for inference
static bool is_initialised = false;
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;

// Capture handler for webserver
void handleCapture(AsyncWebServerRequest *request)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    request->send(500, "text/plain", "Camera capture failed");
    return;
  }
  AsyncWebServerResponse *response = request->beginResponse_P(200, "image/jpeg", fb->buf, fb->len);
  response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
  request->send(response);
  esp_camera_fb_return(fb);
}

// Simple HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head><title>ESP32CAM</title></head>
<body>
  <h2>ESP32CAM Inference Demo</h2>
  <img id="photo" src="/capture" style="max-width:100%;"/>
  <p id="status"></p>
<script>
if (!!window.EventSource) {
  var source = new EventSource('/events');
  source.addEventListener('photo', function(e) {
    var img = document.getElementById("photo");
    img.src = "/capture?rand=" + new Date().getTime();
    document.getElementById("status").innerHTML = "Last updated: " + new Date().toLocaleTimeString();
  }, false);
}
</script>
</body></html>)rawliteral";

void initCamera()
{
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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  if (psramFound())
  {
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    is_initialised=false;
  }
  is_initialised=true;
}

void reconnectMQTT()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32CAMClient", "test", "1234"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}


static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

void setup()
{
  Serial.begin(115200);
  initCamera();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  net.setCACert(ca_cert);
  client.setServer(mqtt_server, mqtt_port);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });
  server.on("/capture", HTTP_GET, handleCapture);
  events.onConnect([](AsyncEventSourceClient *client)
                   { Serial.println("Client connected to /events"); });
  server.addHandler(&events);
  server.begin();

  // pinMode(LED_PIN, OUTPUT);
  Serial.print("Camera Ready! Open http://"); 
  Serial.print(WiFi.localIP()); 
  Serial.println(" in your browser to see the capture page");

  configTime(5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

void loop()
{
  if (!client.connected())
  {
    reconnectMQTT();
  }
  client.loop();

  if ((millis() - lastTime) > timerDelay)
  {
    events.send("update", "photo", millis());
    lastTime = millis();

    // Capture frame for inference
    snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf)
    {
      Serial.println("Failed to allocate buffer");
      return;
    }

    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf))
    {
      Serial.println("Capture failed");
      free(snapshot_buf);
      return;
    }

    ei::signal_t eiSignal; eiSignal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT; 
    eiSignal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&eiSignal, &result, false);

    String defectStatus = "no defect";
    if (err == EI_IMPULSE_OK)
    {
      // Print timing info
      Serial.printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                    result.timing.dsp,
                    result.timing.classification,
                    result.timing.anomaly);

      // Print classification results
      for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
      {
        Serial.printf("%s: %.5f\n",
                      ei_classifier_inferencing_categories[i],
                      result.classification[i].value);

        if (String(ei_classifier_inferencing_categories[i]) == "crack" &&
            result.classification[i].value > 0.6)
        {
          defectStatus = "defect";
        }
      }
    }
    else
    {
      Serial.printf("ERR: Failed to run classifier (%d)\n", err);
    }

    free(snapshot_buf);

    // Build JSON payload
    StaticJsonDocument<200> doc;
    doc["road_defect"] = defectStatus;
    JsonObject gps = doc.createNestedObject("GPS_coordinates");
    gps["latitude"] = 33.636351;
    gps["longitude"] = 72.989005;
    JsonObject prob = doc.createNestedObject("Inference_Result");
    prob["crack_prob"] = result.classification[0].value;
    prob["no_crack_prob"] = result.classification[1].value;

    time_t now;
    time(&now);
    doc["timestamp"] = now;
    

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish("road_defect/esp32/out", buffer, n);

    Serial.println("Published JSON:");
    Serial.println(buffer);
  }
}
