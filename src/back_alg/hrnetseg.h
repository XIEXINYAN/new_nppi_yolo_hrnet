#ifndef HRNETSEG_H
#define HRNETSEG_H
#include "NvInfer.h"
#include <cuda_runtime_api.h>
#include "NvInferRuntime.h"
#include <opencv2/opencv.hpp>
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include <fstream>
#include "cuda_utils.h"
#include <dirent.h>
#include <sys/timeb.h>
#include <chrono>
#define DEFAULT_DELTA_HEIGHT 100
#define DEFAULT_MEAN_STD 100
class HrNetSeg
{
public:
    HrNetSeg();
    ~HrNetSeg();
    std::vector<cv::Point> hrNetSegProcess(const float* data);
private:
    bool build();
    std::string onnx_file_name_="../config/ocr.onnx";
    std::string engine_file_name_=onnx_file_name_+"trt";
    int input_index_;
    int output1_index_;
    int output2_index_;
    void* buffer_[3];
    float *out_data_;
    const int num_classes=2;
    const int out_height_=135;
    const int out_width_=240;
    const int height_{540};
    const int width_{960};
    cudaStream_t stream_;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context;
    std::vector<std::string> findFileFloder(std::string path);
    void fillHole(const cv::Mat src, cv::Mat &dst);
    std::vector<cv::Point> findRoadCurb(cv::Mat in_image);
    void curbLineEvaluate(const std::vector<cv::Point> curb_line_points, float &delta_height, float& mean_std);
};

#endif // HRNETSEG_H
