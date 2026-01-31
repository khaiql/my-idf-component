#include "esp_heap_caps.h"
#include "esp_log.h"
#include "litter_robot_detect.hpp"
#include "model_data.h"
#include <esp_jpeg_common.h>
#include <esp_jpeg_dec.h>
#include <stdio.h>

#define USE_ESP_NEW_JPEG 1

litter_robot_detect::CatDetect::CatDetect() {}

litter_robot_detect::CatDetect::~CatDetect()
{
    if (interpreter)
    {
        delete interpreter;
        interpreter = nullptr;
    }
    if (resolver_)
    {
        delete resolver_;
        resolver_ = nullptr;
    }
    if (tensor_arena_)
    {
        heap_caps_free(tensor_arena_);
        tensor_arena_ = nullptr;
    }
}

esp_err_t litter_robot_detect::CatDetect::setup(size_t tensor_arena_size)
{
    // Load model
    model = tflite::GetModel(g_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        ESP_LOGE("CatDetect",
                 "Model provided is schema version %d not equal "
                 "to supported version %d.",
                 model->version(), TFLITE_SCHEMA_VERSION);
        return ESP_FAIL;
    }

    // Create interpreter
    resolver_ = new tflite::MicroMutableOpResolver<7>();
    resolver_->AddConv2D();
    resolver_->AddDepthwiseConv2D();
    resolver_->AddQuantize();
    resolver_->AddReshape();
    resolver_->AddSoftmax();
    resolver_->AddMean();
    resolver_->AddFullyConnected();

    tensor_arena_ = (uint8_t *)heap_caps_malloc(
        tensor_arena_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tensor_arena_)
    {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return ESP_ERR_NO_MEM;
    }

    interpreter = new tflite::MicroInterpreter(model, *resolver_, tensor_arena_,
                                               tensor_arena_size);
    if (!interpreter)
    {
        ESP_LOGE(TAG, "Failed to create interpreter");
        return ESP_ERR_NO_MEM;
    }

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "setup model successfully");
    return ESP_OK;
}

jpeg_error_t decode_jpeg(uint8_t *input_buf, int len, uint8_t height,
                         uint8_t width, uint8_t *output_buf, int *out_len)
{
    jpeg_error_t ret = JPEG_ERR_OK;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t *out_info = NULL;

    // Generate default configuration
    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = JPEG_PIXEL_FORMAT_RGB888;
    config.scale.width = width;
    config.scale.height = height;

    // Create jpeg_dec handle
    jpeg_dec_handle_t jpeg_dec = NULL;
    ret = jpeg_dec_open(&config, &jpeg_dec);
    if (ret != JPEG_ERR_OK)
    {
        return ret;
    }

    // Create io_callback handle
    jpeg_io = (jpeg_dec_io_t *)calloc(1, sizeof(jpeg_dec_io_t));
    if (jpeg_io == NULL)
    {
        ret = JPEG_ERR_NO_MEM;
        goto jpeg_dec_clean;
    }

    // Create out_info handle
    out_info =
        (jpeg_dec_header_info_t *)calloc(1, sizeof(jpeg_dec_header_info_t));
    if (out_info == NULL)
    {
        ret = JPEG_ERR_NO_MEM;
        goto jpeg_dec_clean;
    }

    // Set input buffer and buffer len to io_callback
    jpeg_io->inbuf = input_buf;
    jpeg_io->inbuf_len = len;

    // Parse jpeg picture header and get picture for user and decoder
    ret = jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info);
    if (ret != JPEG_ERR_OK)
    {
        goto jpeg_dec_clean;
    }

    *out_len = out_info->width * out_info->height * 3;
    ESP_LOGD("jpeg_decode", "Decoded JPEG image size: %dx%d, output len: %d",
             out_info->width, out_info->height, *out_len);
    // Calloc out_put data buffer and update inbuf ptr and inbuf_len
    if (config.output_type == JPEG_PIXEL_FORMAT_RGB565_LE ||
        config.output_type == JPEG_PIXEL_FORMAT_RGB565_BE ||
        config.output_type == JPEG_PIXEL_FORMAT_CbYCrY)
    {
        *out_len = out_info->width * out_info->height * 2;
    }
    else if (config.output_type == JPEG_PIXEL_FORMAT_RGB888)
    {
        *out_len = out_info->width * out_info->height * 3;
    }
    else
    {
        ret = JPEG_ERR_INVALID_PARAM;
        goto jpeg_dec_clean;
    }
    jpeg_io->outbuf = output_buf;

    // Start decode jpeg
    ret = jpeg_dec_process(jpeg_dec, jpeg_io);
    if (ret != JPEG_ERR_OK)
    {
        goto jpeg_dec_clean;
    }

    // Decoder deinitialize
jpeg_dec_clean:
    jpeg_dec_close(jpeg_dec);
    if (jpeg_io)
    {
        free(jpeg_io);
    }
    if (out_info)
    {
        free(out_info);
    }

    return ret;
}

litter_robot_detect::prediction_result_t
litter_robot_detect::CatDetect::run_inference(const camera_fb_t *fb)
{
    prediction_result_t result;
    result.err = ESP_OK;
    TfLiteTensor *input = interpreter->input(0);

    uint32_t start_decode = esp_log_timestamp();

#if USE_ESP_NEW_JPEG == 0
    ESP_LOGI(TAG, "Decoding JPEG using esp_jpeg");
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = fb->buf,
        .indata_size = fb->len,
        .outbuf = input->data.uint8, // Decode directly to tensor input
        .outbuf_size = input->bytes,
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 0,
        }};
    esp_jpeg_image_output_t outimg;

    esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "JPEG decode failed with error 0x%x", res);
        result.err = res;
        return result;
    }
#else
    uint8_t height = input->dims->data[1];
    uint8_t width = input->dims->data[2];
    ESP_LOGI(TAG, "Decoding JPEG using esp_new_jpeg to %dx%d", width, height);
    jpeg_error_t decode_err = decode_jpeg(
        fb->buf, fb->len, height, width, input->data.uint8, (int *)&input->bytes);
    if (decode_err != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "decode_jpeg failed with error %d", decode_err);
        result.err = ESP_FAIL;
        return result;
    }
#endif
    uint32_t end_decode = esp_log_timestamp();
    ESP_LOGI(TAG, "JPEG decode took %lu ms", end_decode - start_decode);

    // Fix for signed int8 models: convert [0,255] to [-128,127]
    // TFLite quantization often expects int8 inputs (-128 to 127) internally.
    // If inference_input_type was not strictly enforced or if the model
    // lacks a Cast/Quantize op at the entry, the input tensor might be int8.
    // Passing raw uint8 [0,255] results in wrapping (128 becomes -128), ruining
    // predictions.
    if (input->type == kTfLiteInt8)
    {
        ESP_LOGI(TAG, "Using int8 input tensor, adjusting [0,255] to [-128,127]");
        uint8_t *data = input->data.uint8;
        for (size_t i = 0; i < input->bytes; i++)
        {
            // xor 0x80 is equivalent to subtracting 128 for byte-sized values
            // 0 (0x00) ^ 0x80 = 128 (0x80) -> interpreted as -128
            // 255 (0xFF) ^ 0x80 = 127 (0x7F) -> interpreted as 127
            data[i] ^= 0x80;
        }
    }

#if ESP_LOG_LEVEL >= ESP_LOG_INFO
    for (int i = 0; i < 10; i++)
    {
        ESP_LOGD(TAG, "Input tensor data[%d]=%d", i, input->data.uint8[i]);
    }

    // Print the last 10 bytes of the input tensor for debugging
    for (int i = input->bytes - 10; i < input->bytes; i++)
    {
        ESP_LOGD(TAG, "Input tensor data[%d]=%d", i, input->data.uint8[i]);
    }
#endif

    TfLiteStatus invokeStatus = this->interpreter->Invoke();
    if (invokeStatus != kTfLiteOk)
    {
        ESP_LOGE(TAG, "Invoke failed");
        result.err = ESP_FAIL;
        return result;
    }

    decode_result(result);
    return result;
}

void litter_robot_detect::CatDetect::decode_result(
    prediction_result_t &result)
{
    // Handle different output types
    uint8_t empty_score, nachi_score, ngao_score;
    TfLiteTensor *output = interpreter->output(0);

    if (output->type == kTfLiteUInt8)
    {
        empty_score = output->data.uint8[0];
        nachi_score = output->data.uint8[1];
        ngao_score = output->data.uint8[2];
    }
    else if (output->type == kTfLiteInt8)
    {
        // Convert int8 [-128, 127] to uint8 [0, 255]
        empty_score = output->data.int8[0] + 128;
        nachi_score = output->data.int8[1] + 128;
        ngao_score = output->data.int8[2] + 128;
    }
    else
    {
        ESP_LOGE(TAG, "Unsupported output tensor type: %d", output->type);
        result.err = ESP_ERR_NOT_SUPPORTED;
        return;
    }

    ESP_LOGI(TAG, "empty_score=%d nachi_score=%d ngao_score=%d", empty_score,
             nachi_score, ngao_score);

    uint8_t scores[] = {empty_score, nachi_score, ngao_score};
    int max_index = 0;
    for (int i = 1; i < 3; ++i)
    {
        if (scores[i] > scores[max_index])
        {
            max_index = i;
        }
    }

    result.empty_score = empty_score;
    result.nachi_score = nachi_score;
    result.ngao_score = ngao_score;
    result.predicted_class = CLASS_NAMES[max_index];
}
