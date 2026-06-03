/*
  ESP32-CAM TinyML Inference Skeleton
  - Captures frames from camera
  - Runs TFLite Micro model
  - Sends results over Serial

  Prereqs:
  - Install ESP32 board support in Arduino IDE
  - Install TensorFlowLite_ESP32 library (or use ESP-IDF + TFLite Micro)
  - Provide model_tflite.h (C array) generated from .tflite
*/

#include <Arduino.h>
#include "esp_camera.h"

// If using TensorFlow Lite Micro (Arduino lib), include appropriate headers.
// Example (library names may vary depending on your install):
#include "TensorFlowLite.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "model_tflite.h" // <-- generated with xxd -i model.tflite

// ================== CAMERA CONFIG ==================
// AI Thinker ESP32-CAM pinout (default). Adjust if your module differs.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Model input size (must match your model)
static const int kInputWidth = 96;
static const int kInputHeight = 96;
static const int kInputChannels = 1; // 1 for grayscale, 3 for RGB

// Class labels (update to match your model)
static const char *kClassNames[] = {"Blue", "Green", "Red"};
static const int kNumClasses = 3;
static const float kMinScore = 0.50f;

// ================== TFLM SETUP ==================
static tflite::MicroErrorReporter micro_error_reporter;
static tflite::ErrorReporter *error_reporter = &micro_error_reporter;
static const tflite::Model *model = nullptr;
static tflite::AllOpsResolver resolver;
static tflite::MicroInterpreter *interpreter = nullptr;
static TfLiteTensor *input = nullptr;
static TfLiteTensor *output = nullptr;

// Tensor arena for model (size depends on model complexity)
constexpr int kTensorArenaSize = 80 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

// ================== HELPERS ==================

bool init_camera()
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
  config.pixel_format = PIXFORMAT_GRAYSCALE; // or PIXFORMAT_RGB565
  config.frame_size = FRAMESIZE_96X96;       // matches model input
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  return (err == ESP_OK);
}

void setup_tflm()
{
  model = tflite::GetModel(g_model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    error_reporter->Report("Model schema mismatch");
    while (true)
    {
    }
  }

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk)
  {
    error_reporter->Report("AllocateTensors failed");
    while (true)
    {
    }
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
}

int get_output_classes(const TfLiteTensor *tensor)
{
  if (tensor == nullptr || tensor->dims == nullptr)
    return 0;
  // Assume output is [1, num_classes] or [num_classes]
  if (tensor->dims->size == 2)
    return tensor->dims->data[1];
  if (tensor->dims->size == 1)
    return tensor->dims->data[0];
  return 0;
}

bool get_top_class(int &best_idx, float &best_score)
{
  best_idx = -1;
  best_score = 0.0f;
  int classes = get_output_classes(output);
  if (classes <= 0)
    return false;

  if (output->type == kTfLiteFloat32)
  {
    for (int i = 0; i < classes; i++)
    {
      float score = output->data.f[i];
      if (score > best_score)
      {
        best_score = score;
        best_idx = i;
      }
    }
  }
  else if (output->type == kTfLiteUInt8)
  {
    float scale = output->params.scale;
    int32_t zero = output->params.zero_point;
    for (int i = 0; i < classes; i++)
    {
      float score = (output->data.uint8[i] - zero) * scale;
      if (score > best_score)
      {
        best_score = score;
        best_idx = i;
      }
    }
  }
  else
  {
    return false;
  }

  return best_idx >= 0;
}

// Convert camera frame into model input tensor
void fill_input_from_frame(camera_fb_t *fb)
{
  // Assumes GRAYSCALE frame. Normalize to [0,1] if model expects float.
  // Adjust depending on your model input type.
  if (input->type == kTfLiteFloat32)
  {
    float *dst = input->data.f;
    for (size_t i = 0; i < fb->len; i++)
    {
      dst[i] = fb->buf[i] / 255.0f;
    }
  }
  else if (input->type == kTfLiteUInt8)
  {
    memcpy(input->data.uint8, fb->buf, fb->len);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  if (!init_camera())
  {
    Serial.println("Camera init failed");
    while (true)
    {
    }
  }

  setup_tflm();
  Serial.println("ESP32-CAM ready");
}

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    delay(1000);
    return;
  }

  fill_input_from_frame(fb);

  if (interpreter->Invoke() != kTfLiteOk)
  {
    Serial.println("Inference failed");
    esp_camera_fb_return(fb);
    delay(1000);
    return;
  }

  // Parse output (classification)
  int best_idx = -1;
  float best_score = 0.0f;
  if (get_top_class(best_idx, best_score) && best_score >= kMinScore)
  {
    const char *label = (best_idx >= 0 && best_idx < kNumClasses) ? kClassNames[best_idx] : "Unknown";
    Serial.printf("class:%s id:%d score:%.3f\n", label, best_idx, best_score);
  }
  else
  {
    Serial.println("class:None score:0.000");
  }

  esp_camera_fb_return(fb);
  delay(200);
}
