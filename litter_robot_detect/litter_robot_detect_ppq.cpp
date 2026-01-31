#include "dl_image_preprocessor.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "litter_robot_detect.hpp"
#include "model_data.h"
#include <stdio.h>

// #ifndef CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ
// #define CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ = 1
// #endif

#ifdef CONFIG_LITTER_ROBOT_MODEL_ESP_PPQ

extern const uint8_t
    model_espdl[] asm("_binary_lite_model_esp32s3_espdl_start");

litter_robot_detect::CatDetect::CatDetect()
{
  img_transformer = new dl::image::ImageTransformer();
}

litter_robot_detect::CatDetect::~CatDetect()
{
  if (model)
  {
    delete model;
    model = nullptr;
  }
  if (model_input)
  {
    delete model_input;
    model_input = nullptr;
  }
  if (model_output)
  {
    delete model_output;
    model_output = nullptr;
  }
  if (img_transformer)
  {
    delete img_transformer;
    img_transformer = nullptr;
  }
}

esp_err_t litter_robot_detect::CatDetect::setup(size_t tensor_arena_size)
{
  ESP_LOGI(TAG, "Loading ESP-DL PPQ model...");
  model = new dl::Model((const char *)model_espdl,
                        fbs::MODEL_LOCATION_IN_FLASH_RODATA);
  if (model->test() != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to load model");
    return ESP_FAIL;
  }

  std::map<std::string, dl::TensorBase *> model_inputs = model->get_inputs();
  model_input = model_inputs.begin()->second;
  std::map<std::string, dl::TensorBase *> model_outputs = model->get_outputs();
  model_output = model_outputs.begin()->second;

  ESP_LOGI(TAG, "Model loaded successfully.");
  ESP_LOGI(TAG, "input types %s, output types %s",
           model_input->get_dtype_string(), model_output->get_dtype_string());
  ESP_LOGI(TAG, "Model input shape: ");
  for (const auto &dim : model_input->get_shape())
  {
    ESP_LOGI(TAG, "%d ", dim);
  }
  ESP_LOGI(TAG, "\nModel output shape: ");
  for (const auto &dim : model_output->get_shape())
  {
    ESP_LOGI(TAG, "%d ", dim);
  }

  model->minimize();
  return ESP_OK;
}

litter_robot_detect::prediction_result_t
litter_robot_detect::CatDetect::run_inference(const camera_fb_t *fb)
{
  img_transformer->reset();
  dl::image::jpeg_img_t jpeg_img = {
      .data = fb->buf,
      .data_len = fb->len,
  };
  auto img =
      dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888, dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN);

  img_transformer->set_src_img(img);

  dl::image::img_t dst_img = {.data = model_input->data,
                              .width = (uint8_t)model_input->shape[2],
                              .height = (uint8_t)model_input->shape[1],
                              .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888};

  // Transform to RGB888
  // dl::image::NormQuantWrapper::quant_type_t quant_type = dl::image::NormQuantWrapper::;
  img_transformer->set_dst_img(dst_img)
      .set_caps(dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN);
  // .set_norm_quant_param(
  //     {0, 0, 0}, {255, 255, 255}, model_input->exponent, quant_type);
  esp_err_t tx_err = img_transformer->transform();

  if (tx_err != ESP_OK)
  {
    ESP_LOGE(TAG, "Image transform failed");
    heap_caps_free(img.data);
    return {.err = tx_err};
  }

  heap_caps_free(img.data);
  return run_inference(dst_img);
}

void litter_robot_detect::CatDetect::decode_result(
    prediction_result_t &result)
{
  if (result.err != ESP_OK)
  {
    return;
  }

  // Access data based on dtype. Most esp-dl PPQ models output int16_t or
  // int8_t.
  ESP_LOGI(TAG, "Model Output Info:");
  ESP_LOGI(TAG, "  Exponent: %d", model_output->exponent);
  ESP_LOGI(TAG, "  DType: %s", model_output->get_dtype_string());
  ESP_LOGI(TAG, "  Shape: %s",
           dl::vector_to_string(model_output->shape).c_str());
  model_output->print(true);

  float scores_raw[3];
  dl::TensorBase *output_tensor =
      new dl::TensorBase(model_output->get_shape(), nullptr,
                         model_output->exponent, dl::DATA_TYPE_FLOAT);
  output_tensor->assign(model_output);

  ESP_LOGI(TAG, "Output Tensor (converted to float) Info:");
  output_tensor->print(true);

  scores_raw[0] = output_tensor->get_element<float>(0);
  scores_raw[1] = output_tensor->get_element<float>(1);
  scores_raw[2] = output_tensor->get_element<float>(2);

  // Log the raw scores
  ESP_LOGI(TAG, "empty_score=%f nachi_score=%f ngao_score=%f", scores_raw[0],
           scores_raw[1], scores_raw[2]);

  // Map to uint8_t for the result struct.
  // We add 128 for signed 8-bit types to map [-128, 127] to [0, 255],
  // matching the TFLite implementation's behavior.
  uint8_t scores_uint8[3];
  if (model_output->get_dtype() == dl::DATA_TYPE_INT8)
  {
    scores_uint8[0] = (uint8_t)(scores_raw[0] + 128);
    scores_uint8[1] = (uint8_t)(scores_raw[1] + 128);
    scores_uint8[2] = (uint8_t)(scores_raw[2] + 128);
  }
  else
  {
    // Assume float output is probability [0, 1]
    scores_uint8[0] = (uint8_t)(scores_raw[0] * 255.0f);
    scores_uint8[1] = (uint8_t)(scores_raw[1] * 255.0f);
    scores_uint8[2] = (uint8_t)(scores_raw[2] * 255.0f);
  }

  int max_index = 0;
  for (int i = 1; i < 3; ++i)
  {
    if (scores_uint8[i] > scores_uint8[max_index])
    {
      max_index = i;
    }
  }

  result.empty_score = scores_uint8[0];
  result.nachi_score = scores_uint8[1];
  result.ngao_score = scores_uint8[2];
  result.predicted_class = CLASS_NAMES[max_index];
}

litter_robot_detect::prediction_result_t
litter_robot_detect::CatDetect::run_inference(const dl::image::img_t &img)
{
  uint32_t start_infer = esp_log_timestamp();

  ESP_LOGI(TAG, "Input Image Info:");
  ESP_LOGI(TAG, "  Width: %d, Height: %d", img.width, img.height);
  ESP_LOGI(TAG, "  Pix Type: %d", img.pix_type);

  if (img.data)
  {
    uint8_t *data_u8 = (uint8_t *)img.data;
    float *data_f = (float *)img.data;

    ESP_LOGI(TAG, "Sample Input Data (First 10 values):");
    ESP_LOGI(TAG, "  As uint8: %d %d %d %d %d %d %d %d %d %d", data_u8[0],
             data_u8[1], data_u8[2], data_u8[3], data_u8[4], data_u8[5],
             data_u8[6], data_u8[7], data_u8[8], data_u8[9]);
    ESP_LOGI(TAG, "  As float: %f %f %f %f %f %f %f %f %f %f", data_f[0],
             data_f[1], data_f[2], data_f[3], data_f[4], data_f[5], data_f[6],
             data_f[7], data_f[8], data_f[9]);
  }

  model_input->assign({1, img.height, img.width, 3}, img.data,
                      model_input->exponent, dl::DATA_TYPE_INT8);

  // ESP_LOGI(TAG, "Model Input Tensor Info (after assign):");
  // model_input->print(true);

  model->run();
  prediction_result_t result;
  result.err = ESP_OK;

  uint32_t end_infer = esp_log_timestamp();
  ESP_LOGI(TAG, "Inference took %lu ms", end_infer - start_infer);

  decode_result(result);
  return result;
}
#endif