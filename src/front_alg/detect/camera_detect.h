#ifndef CAMERA_DETECT_H
#define CAMERA_DETECT_H
#include "../common/cuda_utils.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <perception/common/object.h>
#include <fstream>
#include "NvInfer.h"
#include "NvInferRuntime.h"

#define USE_FP16
#define DEVICE 0
#define NMS_THRESH 0.5
#define CONF_THRESH 0.3

class CameraDetect
{
public:
  CameraDetect(int batch_size);
  ~CameraDetect();
  std::vector<std::vector<Object>> cameraDetectProcess(const float* data);
private:
  void process();
  std::vector<std::vector<Object>> m_detect_vec;
  const std::string wts_name_ = "../config/yolov5m.wts";
  const int width_= 640;
  const int height_=640;
  const int output_size_{6001};
  const int class_num_{24};
  static float* buffer[2];
  static float* prob;
  cudaStream_t stream_;
  int input_index_;
  int output_index_;
  bool engineParse();
  const float gd_=0.67f;
  const float gw_=0.75f;
  std::string net_ = "m";
  const std::string engine_name_ = "../config/yolov5m.engine";
  const char* input_name_ = "data";
  const char* output_name_ = "prob";
  int m_batchsize=0;

  nvinfer1::ICudaEngine* engine=nullptr;
  nvinfer1::IExecutionContext* context;
//  bool parse_args();
  nvinfer1::ICudaEngine* buildEngine(nvinfer1::IBuilder* builder,
                            nvinfer1::IBuilderConfig* config, nvinfer1::DataType dt);
  static int get_depth(int x, float gd);
  static int get_width(int x, float gw, int divisor=8);
  
};
inline int CameraDetect::get_depth(int x, float gd)
{
    if(x==1)
    {
        return 1;
    }
    else
    {
        return round(x*gd)>1?round(x*gd):1;
    }
}
inline int CameraDetect::get_width(int x, float gw, int divisor)
{
    if(int(x*gw)%divisor == 0)
    {
        return int(x*gw);
        return (int(x*gw/divisor)+1)*divisor;
    }
}
#endif // CAMERA_DETECT_H
