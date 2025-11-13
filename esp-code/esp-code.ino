#include <WiFi.h>
#include <driver/i2s.h>
#include <driver/dac.h>
#include <WebSocketsClient.h>
#include "secrets.h"

#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0

#define RECORD_BUTTON 26
#define LED_BUILTIN 2

#define DAC_CHANNEL DAC_CHANNEL_1
  
#define SAMPLE_RATE 16000
#define BUFFER_SIZE 4096

WebSocketsClient webSocket;
volatile bool isRecording = false;
volatile bool isReceivingAudio = false;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR onTimer() {}  // kept for future use if needed

void setupWifi() {
  Serial.print("[ESP] WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" ✓");
}

void setupI2SMicrophone() {
  Serial.print("[ESP] Microphone...");

  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[ESP] Failed to install I2S driver: %d\n", err);
    return;
  }

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[ESP] Failed to set I2S pins: %d\n", err);
    return;
  }
  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println("✓");
}

void setupDACOutput() {
  Serial.print("[ESP] Speaker (DAC)...");

  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, 128);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / SAMPLE_RATE, true);
  timerAlarmEnable(timer);

  Serial.println(" ✓");
}

void send_audio_chunk() {
  const int samples = 1024;
  int32_t buffer32[samples];
  size_t bytes_read;

  esp_err_t result = i2s_read(I2S_PORT, buffer32, samples * sizeof(int32_t),
                              &bytes_read, portMAX_DELAY);

  if (result != ESP_OK) {
    Serial.printf("[ESP] I2S read error: %d\n", result);
    return;
  }

  int16_t pcm16[samples];
  for (int i = 0; i < samples; i++) {
    pcm16[i] = (int16_t)(buffer32[i] >> 16);
  }

  webSocket.sendBIN((uint8_t*)pcm16, sizeof(pcm16));
}

void playTestTone(int durationMs, int times) {
  Serial.println("[ESP] Playing 1kHz test tone...");
  int samples = (8000 * durationMs) / 1000;

  for (int i = 0; i < times; i++) {
    for (int i = 0; i < samples; i++) {
      float t = (float)i / 8000.0;
      float sine = sin(2.0 * PI * 1000.0 * t);
      uint8_t value = (uint8_t)((sine * 100) + 128);

      dac_output_voltage(DAC_CHANNEL, value);
      delayMicroseconds(62.5);
    }
    delay(50);
  }

  dac_output_voltage(DAC_CHANNEL, 128);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      webSocket.sendTXT("ping");
      break;

    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      isReceivingAudio = false;
      dac_output_voltage(DAC_CHANNEL, 128);
      break;

    case WStype_TEXT:
      if (strcmp((const char*)payload, "pong") == 0) {
        Serial.println("[WS] Connected to server");
      } else {
        Serial.print("[WS] ");
        Serial.println((const char*)payload);
      }
      break;

    case WStype_BIN:
      if (isReceivingAudio || length > 100) {
        if (!isReceivingAudio) {
          Serial.println("[ESP] Starting playback");
          isReceivingAudio = true;
        }

        for (size_t i = 0; i < length; i++) {
          dac_output_voltage(DAC_CHANNEL, payload[i]);
          delayMicroseconds(62.5);
        }

        static unsigned long lastDot = 0;
        if (millis() - lastDot > 500) {
          Serial.print(".");
          lastDot = millis();
        }
      }
      break;

    case WStype_ERROR:
      Serial.println("[WS] Error occurred");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RECORD_BUTTON, INPUT_PULLDOWN);

  Serial.println("\n\n╔═══════════════════════════════════╗");
  Serial.println("║     ESP32 Voice Assistant v1.0     ║");
  Serial.println("╚═══════════════════════════════════╝\n");

  setupWifi();
  setupI2SMicrophone();
  setupDACOutput();

  Serial.print("[ESP] WiFi addr: ");
  Serial.println(WiFi.localIP());
  Serial.print("[ESP] Server: ");
  Serial.print(WS_HOST);
  Serial.print(":");
  Serial.println(WS_PORT);

  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(3000);
  // digitalWrite(LED_BUILTIN, LOW);

  playTestTone(250, 2);

  Serial.println("[ESP] Setup complete! Press button to talk.");
}

void loop() {
  webSocket.loop();

  static bool lastButtonState = LOW;
  bool button = digitalRead(RECORD_BUTTON);

  if (button == HIGH && lastButtonState == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("[ESP] 🎤 Recording...");
    isRecording = true;
    isReceivingAudio = false;
    dac_output_voltage(DAC_CHANNEL, 128);
    webSocket.sendTXT("pause");
  } else if (button == LOW && lastButtonState == HIGH) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("[ESP] ⏹ Stopped. Processing...");
    isRecording = false;
    webSocket.sendTXT("stop");
  }
  lastButtonState = button;

  if (isRecording && webSocket.isConnected()) {
    send_audio_chunk();
  }

  delay(10);
}