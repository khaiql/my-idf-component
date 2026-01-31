#pragma once

#include <esp_camera.h>
#include <string>
#include "esp_log.h"
#include "esp_err.h"

#ifdef CONFIG_LITTER_ROBOT_MODEL_TFLITE
#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#elifdef CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ
#include "dl_model_base.hpp"
#include "dl_image_jpeg.hpp"
#include "dl_image.hpp"
#endif

// #ifndef CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ
// #define CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ = 1
// #endif

namespace litter_robot_detect
{
  typedef struct
  {
    uint8_t empty_score{0};
    uint8_t nachi_score{0};
    uint8_t ngao_score{0};
    std::string predicted_class{""};
    esp_err_t err{ESP_OK};
  } prediction_result_t;

  static const std::string CLASS_NAMES[] = {"empty", "nachi", "ngao"};

  class CatDetect
  {
  public:
    CatDetect();
    ~CatDetect();

    esp_err_t setup(size_t tensor_arena_size);
    prediction_result_t run_inference(const camera_fb_t *fb);
#ifdef CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ
    prediction_result_t run_inference(const dl::image::img_t &img);
#endif

    void test_model();

  private:
    static constexpr const char *TAG{"litter_robot_detect::CatDetect"};

#ifdef CONFIG_LITTER_ROBOT_MODEL_TFLITE
    uint8_t *tensor_arena_{nullptr};
    const tflite::Model *model{nullptr};
    tflite::MicroInterpreter *interpreter{nullptr};
    tflite::MicroMutableOpResolver<7> *resolver_{nullptr};
#elif defined CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ
    dl::Model *model{nullptr};
    dl::TensorBase *model_input{nullptr};
    dl::TensorBase *model_output{nullptr};
    dl::image::ImageTransformer *img_transformer{nullptr};
#endif

    void decode_result(prediction_result_t &result);
  };
} // namespace litter_robot_detect
