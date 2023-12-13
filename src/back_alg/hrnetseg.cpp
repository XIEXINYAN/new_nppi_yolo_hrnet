#include "hrnetseg.h"
#include "logger.h"

HrNetSeg::HrNetSeg()
{
    if(!build())
    {
        std::cout<<"HrNetSeg cannot build"<<std::endl;
        return;
    }
    out_data_ = new float[num_classes*out_height_*out_width_];
    std::cout<<"hrnet inited"<<std::endl;
}
HrNetSeg::~HrNetSeg()
{
    cudaFree(buffer_[input_index_]);
    cudaFree(buffer_[output1_index_]);
    cudaFree(buffer_[output2_index_]);
    cudaStreamDestroy(stream_);
    context->destroy();
    engine->destroy();
    delete [] out_data_;
}
std::vector<cv::Point> HrNetSeg::hrNetSegProcess(const float* data)
{

    cv::Mat res;
    cv::Mat res_mat1=cv::Mat(out_height_, out_width_, CV_32FC1);
    cv::Mat res_mat2=cv::Mat(out_height_, out_width_, CV_32FC1);

    auto start = std::chrono::high_resolution_clock::now();

    //cv::cvtColor(img,img,cv::COLOR_BGR2RGB);
    
    
    // cudaMemcpy(in_data,data,3*height_*width_*sizeof(float),cudaMemcpyDeviceToDevice);
    
    // cudaMemcpy(in_data_,data,height_*width_*sizeof(float),cudaMemcpyDeviceToDevice);
    // cudaMemcpy(in_data_+height_*width_,data+height_*width_,height_*width_*sizeof(float),cudaMemcpyDeviceToDevice);
    // cudaMemcpy(in_data_+2*height_*width_,data+2*height_*width_,height_*width_*sizeof(float),cudaMemcpyDeviceToDevice);
    
    cudaMemset(buffer_[input_index_],0.0, 3*height_*width_*sizeof(float));
    CUDA_CHECK(cudaMemcpyAsync(buffer_[input_index_], data, 3*height_*width_*sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    context->enqueueV2(buffer_, stream_, nullptr);
    if(0)
    {
        cv::Mat img_merge2;
        img_merge2.create(height_, width_, CV_32FC3);
        cv::Mat channels2[3];
        for (int i = 0; i < 3; i++)
        {
            channels2[i].create(height_, width_, CV_32FC1);
        }

        float d_hrnet_mean[3] = {-0.515523, -0.526210, -0.545487}; // 1/255.0
        float d_hrnet_std[3] = {18.95, 18.44, 19.15};              // 1/255.0
        cudaError_t err_cu_api;
        err_cu_api = cudaMemcpy(channels2[0].data, buffer_[input_index_] + 2 * height_ * width_ * sizeof(float), height_ * width_ * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels2[1].data, buffer_[input_index_] + height_ * width_ * sizeof(float), height_ * width_ * sizeof(float), cudaMemcpyDeviceToHost);
        err_cu_api = cudaMemcpy(channels2[2].data, buffer_[input_index_], height_ * width_ * sizeof(float), cudaMemcpyDeviceToHost);

        cv::Mat timg(height_, width_, CV_32FC1, cv::Scalar(1 / d_hrnet_std[0]));
        channels2[2] = channels2[2].mul(timg);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(-d_hrnet_mean[0]));
        cv::add(channels2[2], timg, channels2[2]);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(255.0));
        channels2[2] = channels2[2].mul(timg);

        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(1 / d_hrnet_std[1]));
        channels2[1] = channels2[1].mul(timg);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(-d_hrnet_mean[1]));
        cv::add(channels2[1], timg, channels2[1]);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(255.0));
        channels2[1] = channels2[1].mul(timg);

        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(1 / d_hrnet_std[2]));
        channels2[0] = channels2[0].mul(timg);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(-d_hrnet_mean[2]));
        cv::add(channels2[0], timg, channels2[0]);
        timg = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(255.0));
        channels2[0] = channels2[0].mul(timg);

        cv::merge(channels2, 3, img_merge2);
        cv::Mat float_img;
        float_img.create(height_, width_, CV_8UC3);
        img_merge2.convertTo(float_img, CV_8UC3);
        cv::imshow("row", float_img);
        cv::waitKey(1);
    }
    CUDA_CHECK(cudaMemcpyAsync(out_data_, buffer_[output2_index_], num_classes*out_height_*out_width_*sizeof(float), cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    memcpy(res_mat1.data, out_data_+out_width_*out_height_, out_width_*out_height_*sizeof(float));
    
    // cv::imshow("seg1",res_mat1);
    // memcpy(res_mat2.data, out_data_, out_width_*out_height_*sizeof(float));
    // cv::resize(res_mat2, res_mat2, cv::Size(480, 270));
    // cv::imshow("seg2", res_mat2); // bk
    // cv::add(res_mat1, res_mat2, res_mat1);
    
    cv::Mat res_bi=res_mat1>0.05;
    //1.delete minues holl
    fillHole(res_bi,res_bi);
    //2.findCurbline
    std::vector<cv::Point> curb_line;
    curb_line.clear();
    cv::resize(res_bi,res_bi,cv::Size(width_,height_));
    curb_line=findRoadCurb(res_bi);
//    cv::imshow("res_bi",res_bi);
    float mean_height=0.0;
    float mean_std=0.0;
   curbLineEvaluate(curb_line,mean_height,mean_std);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
//    mean_std>10   nopainting
//    memset(out_data_,0,num_classes*out_width_*out_height_);
    return curb_line;
}

bool HrNetSeg::build()
{
    cudaSetDevice(0);
    std::fstream file;
    file.open(engine_file_name_.c_str(), std::ifstream::in);
    if(!file.good())
    {
        nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(sample::gLogger.getTRTLogger());
        assert(builder!=nullptr);
        const auto explicitBatch = 1U<<static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = builder->createNetworkV2(explicitBatch);
        auto config = builder->createBuilderConfig();
        auto parser = nvonnxparser::createParser(*network, sample::gLogger.getTRTLogger());
        auto parsed = parser->parseFromFile(onnx_file_name_.c_str(),
                                            static_cast<int>(sample::gLogger.getReportableSeverity()));
        if(!parsed)
        {
            return false;
        }
        config->setMaxWorkspaceSize((1<<30));
        config->setFlag(nvinfer1::BuilderFlag::kFP16);

        auto engine=builder->buildEngineWithConfig(*network, *config);
        assert(engine!=nullptr);
        std::cout<<"Build engine successfully!"<<std::endl;
        parser->destroy();
        config->destroy();
        builder->destroy();
        network->destroy();

        nvinfer1::IHostMemory* trtModelStream = engine->serialize();
        std::ofstream p(engine_file_name_.c_str(), std::ios::binary);
        p.write(reinterpret_cast<const char*>(trtModelStream->data()),
                trtModelStream->size());
        trtModelStream->destroy();
        p.close();
        std::cout<<"TRT Engine file saved to : "<<engine_file_name_<<std::endl;

        context = engine->createExecutionContext();
        assert(context!=nullptr);

        input_index_ = engine->getBindingIndex("input");
        output1_index_ = engine->getBindingIndex("output1");
        output2_index_ = engine->getBindingIndex("output2");

        std::cout<<input_index_<<"  "<<output1_index_<<"  "<<output2_index_<<std::endl;
        nvinfer1::Dims input_dim = context->getBindingDimensions(input_index_);
        nvinfer1::Dims output_dim = context->getBindingDimensions(output1_index_);

        std::cout<<input_dim.d[0]<<"  "<<input_dim.d[1]<<"  "<<input_dim.d[2]<<"  "<<input_dim.d[3]
                                <<output_dim.d[0]<<"  "<<output_dim.d[1]<<"  "<<output_dim.d[2]<<"  "<<output_dim.d[3]<<std::endl;
        CUDA_CHECK(cudaMalloc(&buffer_[input_index_],3*height_*width_*sizeof (float)));
        CUDA_CHECK(cudaMalloc(&buffer_[output1_index_],num_classes*out_width_*out_height_*sizeof (float)));
        CUDA_CHECK(cudaMalloc(&buffer_[output2_index_],num_classes*out_width_*out_height_*sizeof (float)));
        CUDA_CHECK( cudaStreamCreate(&stream_));
        std::cout<<"engineParse over"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"read engine file"<<engine_file_name_<<std::endl;
        char *trtModelStream=nullptr;
        size_t size =0;
        file.seekg(0, file.end);
        size = file.tellg();
        std::cout<<"file size is : "<<size<<std::endl;
        file.seekg(0, file.beg);
        trtModelStream = new char [size];
        file.read(trtModelStream, size);
        file.close();
        nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(sample::gLogger.getTRTLogger());
        assert(runtime != nullptr);
        auto engine=runtime->deserializeCudaEngine(trtModelStream, size);
        if(nullptr ==engine)
        {
            std::cout<<"nullptr ==engine"<<std::endl;
            return -1;
        }
        context = engine->createExecutionContext();
        if(nullptr ==context)
        {
            std::cout<<"nullptr ==context"<<std::endl;
            return -1;
        }

        delete [] trtModelStream;
        input_index_ = engine->getBindingIndex("input");
        output1_index_ = engine->getBindingIndex("output1");
        output2_index_ = engine->getBindingIndex("output2");
        runtime->destroy();
        std::cout<<input_index_<<" $$$ "<<output1_index_<<" $$$ "<<output2_index_<<std::endl;
        nvinfer1::Dims input_dim = context->getBindingDimensions(input_index_);
        nvinfer1::Dims output_dim = context->getBindingDimensions(output1_index_);

        std::cout<<input_dim.d[0]<<"  "<<input_dim.d[1]<<"  "<<input_dim.d[2]<<"  "<<input_dim.d[3]
                                <<output_dim.d[0]<<"  "<<output_dim.d[1]<<"  "<<output_dim.d[2]<<"  "<<output_dim.d[3]<<std::endl;
        CUDA_CHECK(cudaMalloc(&buffer_[input_index_],3*height_*width_*sizeof (float)));
        CUDA_CHECK(cudaMalloc(&buffer_[output1_index_],num_classes*out_height_*out_width_*sizeof (float)));
        CUDA_CHECK(cudaMalloc(&buffer_[output2_index_],num_classes*out_height_*out_width_*sizeof (float)));
//        auto res = cuMemGetAddressRange()
        CUDA_CHECK(cudaStreamCreate(&stream_));
        return true;

    }
}
void HrNetSeg::fillHole(const cv::Mat src, cv::Mat &dst)
{
    dst = src.clone();
    cv::Size size = src.size();
    cv::Mat tmp=cv::Mat::zeros(size.height+2, size.width+2, src.type());
    cv::Point seed_point;
    bool is_break = false;
    for(int i=0;i<size.height;i++)
    {
        for(int j=0;j<size.width;j++)
        {
            if(src.at<int>(j,i) == 0)
            {
                seed_point = cv::Point(j,i);
                is_break = true;
                break;
            }
        }
        if(is_break)
            break;
    }
    cv::floodFill(dst, tmp, seed_point, cv::Scalar(255));
    dst = src | (~dst);
    return;
}
std::vector<std::string> HrNetSeg::findFileFloder(std::string path)
{
    std::vector<std::string> filenames;
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return filenames;
    while((ptr = readdir(pDir))!=0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            std::string str=ptr->d_name;
            char * strc = new char[strlen(str.c_str())+1];
            strcpy(strc, str.c_str());
            std::string pattern="\n";
            char* tmpStr = strtok(strc, pattern.c_str());
            while (tmpStr != NULL)
            {
                filenames.push_back(std::string(tmpStr));
                tmpStr = strtok(NULL, pattern.c_str());
            }
            delete [] strc;
        }
    }
    closedir(pDir);
    return filenames;
}

std::vector<cv::Point> HrNetSeg::findRoadCurb(cv::Mat in_image)
{
    cv::Canny(in_image, in_image, 40, 40*2, 3);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(in_image, contours, hierarchy,
                     cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                     cv::Point(0,0));

    //analysis all pixel hist map
    int width=in_image.cols;
    int height=in_image.rows;
//    std::cout<<width<<"  "<<height<<std::endl;
    int interval=5;
    int hist_width[width/interval];
    int hist_height[width/interval];
    memset(hist_width, 0, width/interval*sizeof(int));
    memset(hist_height, 0, width/interval*sizeof (int));

    std::vector<cv::Point> point_vec;
    for(int i=0;i<contours.size();i++)
    {
        if(contours[i].size()<20)
            continue;
        for(int j=0;j<contours[i].size();j++)
        {
            int height = contours[i][j].y;
            int width = contours[i][j].x;
            hist_width[width/interval]++;
            hist_height[width/interval]+=height;
        }
    }

    int cnt_begin=0;
    int cnt_end=0;
    for(int i=0;i<width/interval;i++)
    {
        if(hist_width[i]!=0)
        {
            cnt_begin=i;
            break;
        }
    }
    for(int i=width/interval-1;i>=0;i--)
    {
        if(hist_width[i]!=0)
        {
            cnt_end=i;
            break;
        }
    }
    for(int i=cnt_begin;i<cnt_end;)
    {
        if(hist_width[i] !=0)
        {
            hist_height[i] /= hist_width[i];
            point_vec.push_back(cv::Point(i*interval, hist_height[i]));
            i++;
        }
        else
        {
            int j=i;
            int j_end=i+1;
            for(;j<cnt_end;j++)
            {
                if(hist_height[j]!=0)
                {
                    j_end=j;
                    break;
                }
            }
            for(int j=i;j<j_end;j++)
            {
//                int tmp=hist_height[i-1]+(hist_height[j_end]-hist_height[i-1])/(j_end-i+1)*(j-i+1);
                point_vec.push_back(cv::Point(j*interval, hist_height[i-1]));
            }
            i=j_end;
        }
    }
    return point_vec;
}

void HrNetSeg::curbLineEvaluate(const std::vector<cv::Point> curb_line_points, float &delta_height, float& mean_std)
{
    delta_height=0.0;
    mean_std=0.0;
    if(curb_line_points.size()<5)
    {
        delta_height=DEFAULT_DELTA_HEIGHT;
        mean_std=DEFAULT_MEAN_STD;
        return;
    }
    for(int i=3;i<curb_line_points.size();i+=3)
    {
        delta_height+=fabs(curb_line_points[i].y-curb_line_points[i-3].y);
    }
    delta_height /= (curb_line_points.size()/3);
    for(int i=1;i<curb_line_points.size();i++)
    {
        mean_std+=fabs(curb_line_points[i].y-curb_line_points[i-1].y);
    }
    mean_std/=curb_line_points.size();
    return;
}
