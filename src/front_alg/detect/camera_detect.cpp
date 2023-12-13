#include "camera_detect.h"
#include "../common/logger.h"
#include "yolo_common.hpp"
float* CameraDetect::buffer[];

float* CameraDetect::prob = nullptr;
CameraDetect::CameraDetect(int batch_size)
{
    m_batchsize = batch_size;
    m_detect_vec.resize(m_batchsize);
    // std::cout<<" ###############################"<<m_batchsize<<std::endl;
    if (true != engineParse())
    {
        return;
    }
    
    prob = new float[m_batchsize * output_size_];
    CUDA_CHECK(cudaMalloc((void**)&buffer[input_index_],  m_batchsize* 3 * height_ * width_ * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)&buffer[output_index_], m_batchsize*output_size_*sizeof(float)));
    CUDA_CHECK(cudaStreamCreate(&stream_));
}
CameraDetect::~CameraDetect()
{
    cudaStreamDestroy(stream_);
    CUDA_CHECK(cudaFree(buffer[input_index_]));
    CUDA_CHECK(cudaFree(buffer[output_index_]));
    engine->destroy();
    context->destroy();
    delete [] prob;
}

std::vector<std::vector<Object>> CameraDetect::cameraDetectProcess(const float *data)
{
    for (int i = 0; i < m_batchsize; i++)
    {
        m_detect_vec[i].clear();
    }
    auto start = std::chrono::system_clock::now();  
    CUDA_CHECK(cudaMemcpyAsync(buffer[input_index_], data, m_batchsize*3*height_*width_*sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    cudaStreamSynchronize(stream_);
    context->enqueue(m_batchsize, (void**)&buffer, stream_, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(prob,buffer[output_index_], m_batchsize*output_size_*sizeof(float), cudaMemcpyDeviceToHost, stream_));
    cudaStreamSynchronize(stream_);
    if(0)
    {
        cudaError_t err_cu_api;
        cv::Mat img_merge_float, channels[3];
        img_merge_float.create(height_, width_, CV_32FC3);
        for (int i = 0; i < 3; i++)
        {
            channels[i].create(height_, width_, CV_32FC1);
        }
        int size = width_ * height_;
        err_cu_api = cudaMemcpy(channels[0].data, buffer[input_index_]+2* size, size * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels[1].data, buffer[input_index_]+size, size * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels[2].data, buffer[input_index_], size*sizeof(float), cudaMemcpyDeviceToHost);

        cv::merge(channels, 3, img_merge_float);
        cv::Mat timg(height_, width_, CV_32FC3, cv::Scalar(255.0, 255.0, 255.0));
        img_merge_float = img_merge_float.mul(timg);
        cv::Mat img;
        img_merge_float.convertTo(img, CV_8UC3);
        cv::resize(img, img, cv::Size(width_, height_));

        cv::imshow("tmp1", img);
        
        err_cu_api = cudaMemcpy(channels[0].data, buffer[input_index_]+size*3*sizeof(float)+2* size, size * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels[1].data, buffer[input_index_]+size*3*sizeof(float)+size, size * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels[2].data, buffer[input_index_]+size*3*sizeof(float), size*sizeof(float), cudaMemcpyDeviceToHost);

        cv::merge(channels, 3, img_merge_float);
        img_merge_float = img_merge_float.mul(timg);
        img_merge_float.convertTo(img, CV_8UC3);
        cv::resize(img, img, cv::Size(width_, height_));

        cv::imshow("tmp2", img);
        cv::waitKey(1);
    }
    process();
    auto end = std::chrono::system_clock::now();
    std::cout << "detect time =" << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    return m_detect_vec;
}
void CameraDetect::process()
{
    std::vector<std::vector<Yolo::Detection>> batch_res(m_batchsize);
    for (int b = 0; b < m_batchsize; b++)
    {
        auto &res = batch_res[b];
        nms(res, &prob[b*output_size_], CONF_THRESH, NMS_THRESH);
        for (size_t j = 0; j < res.size(); j++)
        {
            cv::Rect r = get_rect(res[j].bbox);
            int max_x = r.x + r.width;
            int max_y = r.y + r.height;
            Object tmp_object;
            tmp_object.min_2d[0] = r.x < 0 ? 0 : r.x;
            tmp_object.min_2d[1] = r.y < 0 ? 0 : r.y;
            tmp_object.max_2d[0] = max_x > 1919 ? 1919 : max_x;
            tmp_object.max_2d[1] = max_y > 1079 ? 1079 : max_y;
            tmp_object.object_type = (int)res[j].class_id;
            tmp_object.confidence = (float)res[j].conf;
            tmp_object.center[0] = (r.width) / 2 + r.x;  // center_x
            tmp_object.center[1] = (r.height) / 2 + r.y; // center_y
            tmp_object.center[2] = 0;
            tmp_object.width = r.width;
            tmp_object.height = r.height;
            tmp_object.vrel_x = 0.0;
            tmp_object.vrel_y = 0.0;
            tmp_object.vrel_width = 0.0;
            tmp_object.vrel_height = 0.0;
            tmp_object.direction[0] = 0.0;
            tmp_object.direction[1] = 0.0;
            m_detect_vec[b].push_back(tmp_object);
        }
    }
    return;
}
bool CameraDetect::engineParse()
{
    cudaSetDevice(DEVICE);
    std::fstream file;
    file.open(engine_name_.c_str(), std::ofstream::in);
    if (!file.good())
    {
        std::cout << "build engien, wts_name is " << wts_name_ << std::endl;
        nvinfer1::IHostMemory *modelStream{nullptr};
        nvinfer1::IBuilder *builder = nvinfer1::createInferBuilder(sample::gLogger.getTRTLogger());
        nvinfer1::IBuilderConfig *config = builder->createBuilderConfig();
        engine = buildEngine(builder, config, DataType::kFLOAT);
        assert(engine != nullptr);
        modelStream = engine->serialize();
        builder->destroy();
        config->destroy();
        assert(modelStream != nullptr);
        std::ofstream p(engine_name_, std::ios::binary);
        if (!p)
        {
            std::cerr << "could not open plan output file" << std::endl;
            return false;
        }
        p.write(reinterpret_cast<const char *>(modelStream->data()),
                modelStream->size());
        modelStream->destroy();
        if (nullptr == engine)
        {
            std::cout << "nullptr ==engine" << std::endl;
            return false;
        }
        context = engine->createExecutionContext();
        if (nullptr == context)
        {
            std::cout << "nullptr ==context" << std::endl;
            return false;
        }
        assert(engine->getNbBindings() == 2);
        input_index_ = engine->getBindingIndex(input_name_);
        output_index_ = engine->getBindingIndex(output_name_);
        std::cout << input_index_ << "  " << output_index_ << std::endl;
        nvinfer1::Dims input_dim = context->getBindingDimensions(input_index_);
        nvinfer1::Dims output_dim = context->getBindingDimensions(output_index_);
        std::cout << input_dim.d[0] << "  " << input_dim.d[1] << "  " << input_dim.d[2] << "  " << input_dim.d[3]<<"  "
              << output_dim.d[0] << "  " << output_dim.d[1] << "  " << output_dim.d[2] << "  " << output_dim.d[3] << std::endl;

        return true;
    }
    else
    {
        std::cout << "read engine file" << engine_name_ << std::endl;
        char *trtModelStream = nullptr;
        size_t size = 0;
        file.seekg(0, file.end);
        size = file.tellg();
        std::cout << " file size is : " << size << std::endl;
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        file.read(trtModelStream, size);
        file.close();
        nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(sample::gLogger.getTRTLogger());
        assert(runtime != nullptr);
        engine = runtime->deserializeCudaEngine(trtModelStream, size);
        runtime->destroy();
        if (nullptr == engine)
        {
            std::cout << "nullptr ==engine" << std::endl;
            return false;
        }
        context = engine->createExecutionContext();
        if (nullptr == context)
        {
            std::cout << "nullptr ==context" << std::endl;
            return false;
        }

        delete[] trtModelStream;
        assert(engine->getNbBindings() == 2);
        input_index_ = engine->getBindingIndex(input_name_);
        output_index_ = engine->getBindingIndex(output_name_);
        std::cout << input_index_ << "  " << output_index_ << std::endl;
        assert(input_index_ == 0);
        assert(output_index_ == 1);
        nvinfer1::Dims input_dim = context->getBindingDimensions(input_index_);
        nvinfer1::Dims output_dim = context->getBindingDimensions(output_index_);
        std::cout << input_dim.d[0] << "  " << input_dim.d[1] << "  " << input_dim.d[2] << "  " << input_dim.d[3]<<"  "
              << output_dim.d[0] << "  " << output_dim.d[1] << "  " << output_dim.d[2] << "  " << output_dim.d[3] << std::endl;

        std::cout << "engineParse over" << std::endl;
        return true;
    }
}
// bool CameraDetect::parse_args()
//{
//   if(net_ == "s")
//   {
//     gd_ = 0.33;
//     gw_ = 0.50;
//   }
//   else if(net_ =="m")
//   {
//       gd_ =0.67;
//       gw_ =0.75;
//   }
//   else if(net_ =="l")
//   {
//       gd_ =1.0;
//       gw_ =1.0;
//   }
//   else if(net_ =="x")
//   {
//       gd_ =1.33;
//       gw_ =1.25;
//   }
//   else if(net_ == "c")
//   {
//       //gd_ =self_input;
//       //gw_ =atof(self_input);
//   }
//   else {
//       return false;
//   }
//   return true;
// }

nvinfer1::ICudaEngine *CameraDetect::buildEngine(nvinfer1::IBuilder *builder,
                                                 nvinfer1::IBuilderConfig *config, nvinfer1::DataType dt)
{
    nvinfer1::INetworkDefinition *network = builder->createNetworkV2(0U);
    nvinfer1::ITensor *data = network->addInput(input_name_, dt, nvinfer1::Dims3{3, height_, width_});
    assert(data);
    std::map<std::string, Weights> weightMap = loadWeights(wts_name_);
    /*----yolov5 backbone ---*/
    auto focus0 = focus(network, weightMap, *data, 3, get_width(64, gw_), 3, "model.0");
    auto conv1 = convBlock(network, weightMap, *focus0->getOutput(0), get_width(128, gw_), 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw_), get_width(128, gw_), get_depth(3, gd_), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), get_width(256, gw_), 3, 2, 1, "model.3");
    auto bottleneck_CSP4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw_), get_width(256, gw_), get_depth(9, gd_), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_CSP4->getOutput(0), get_width(512, gw_), 3, 2, 1, "model.5");
    auto bottleneck_CSP6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw_), get_width(512, gw_), get_depth(9, gd_), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_CSP6->getOutput(0), get_width(1024, gw_), 3, 2, 1, "model.7");
    auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), get_width(1024, gw_), get_width(1024, gw_), 5, 9, 13, "model.8");
    /* ----- yolov5 head ---- */
    auto bottleneck_csp9 = C3(network, weightMap, *spp8->getOutput(0), get_width(1024, gw_), get_width(1024, gw_), get_depth(3, gd_), false, 1, 0.5, "model.9");
    auto conv10 = convBlock(network, weightMap, *bottleneck_csp9->getOutput(0), get_width(512, gw_), 1, 1, 1, "model.10");

    auto upsample11 = network->addResize(*conv10->getOutput(0));
    assert(upsample11);
    upsample11->setResizeMode(ResizeMode::kNEAREST);
    upsample11->setOutputDimensions(bottleneck_CSP6->getOutput(0)->getDimensions());

    ITensor *inputTensor12[] = {upsample11->getOutput(0), bottleneck_CSP6->getOutput(0)};
    auto cat12 = network->addConcatenation(inputTensor12, 2);
    auto bottleneck_CSP13 = C3(network, weightMap, *cat12->getOutput(0), get_width(1024, gw_), get_width(512, gw_), get_depth(3, gd_), false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_CSP13->getOutput(0), get_width(256, gw_), 1, 1, 1, "model.14");

    auto upsample15 = network->addResize(*conv14->getOutput(0));
    assert(upsample15);
    upsample15->setResizeMode(ResizeMode::kNEAREST);
    upsample15->setOutputDimensions(bottleneck_CSP4->getOutput(0)->getDimensions());

    ITensor *inputTensor16[] = {upsample15->getOutput(0), bottleneck_CSP4->getOutput(0)};
    auto cat16 = network->addConcatenation(inputTensor16, 2);
    auto bottleneck_CSP17 = C3(network, weightMap, *cat16->getOutput(0), get_width(512, gw_), get_width(256, gw_), get_depth(3, gd_), false, 1, 0.5, "model.17");

    // yolo layer 0
    IConvolutionLayer *det0 = network->addConvolutionNd(*bottleneck_CSP17->getOutput(0), 3 * (class_num_ + 5), DimsHW{1, 1}, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_CSP17->getOutput(0), get_width(256, gw_), 3, 2, 1, "model.18");
    ITensor *inputTensor19[] = {conv18->getOutput(0), conv14->getOutput(0)};
    auto cat19 = network->addConcatenation(inputTensor19, 2);
    auto bottleneck_CSP20 = C3(network, weightMap, *cat19->getOutput(0), get_width(512, gw_), get_width(512, gw_), get_depth(3, gd_), false, 1, 0.5, "model.20");
    // yolo layer 1
    IConvolutionLayer *det1 = network->addConvolutionNd(*bottleneck_CSP20->getOutput(0), 3 * (class_num_ + 5), DimsHW{1, 1}, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_CSP20->getOutput(0), get_width(512, gw_), 3, 2, 1, "model.21");
    ITensor *inputTensor22[] = {conv21->getOutput(0), conv10->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensor22, 2);
    auto bottleneck_CSP23 = C3(network, weightMap, *cat22->getOutput(0), get_width(1024, gw_), get_width(1024, gw_), get_depth(3, gd_), false, 1, 0.5, "model.23");

    IConvolutionLayer *det2 = network->addConvolutionNd(*bottleneck_CSP23->getOutput(0), 3 * (class_num_ + 5), DimsHW{1, 1}, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);
    auto yolo = addYoLoLayer(network, weightMap, det0, det1, det2);
    yolo->getOutput(0)->setName(output_name_);
    network->markOutput(*yolo->getOutput(0));

    builder->setMaxBatchSize(2);
    config->setMaxWorkspaceSize(16 * (1 << 20)); // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFatInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntorpyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif
    std::cout << "Building engine,please wait for a while...." << std::endl;
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;
    network->destroy();
    for (auto &mem : weightMap)
    {
        free((void *)(mem.second.values));
    }
    return engine;
}
