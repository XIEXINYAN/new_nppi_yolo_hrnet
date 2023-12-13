#include "camera_fault_detection.h"
#include <bitset>
std::deque<bool> CameraFaultDetection::is_image_partial_color_cnt_;
std::deque<bool> CameraFaultDetection::is_image_contours_cnt_;
std::deque<bool> CameraFaultDetection::is_image_blur_cnt_;
std::deque<bool> CameraFaultDetection::is_camera_near_occlusion_cnt_;
unsigned char CameraFaultDetection::image_error_=0x00;
bool CameraFaultDetection::is_time_cirle_start_=false;
std::deque<MatStats> CameraFaultDetection::camera_occlusion_stat_pool_;
pthread_mutex_t CameraFaultDetection::image_reeoe_code_mutex_;
bool CameraFaultDetection::is_one_time_finised_=false;
bool CameraFaultDetection::is_camera_rev_data_flag_=false;
bool CameraFaultDetection::is_camera_connect_flag_=false;
CameraFaultDetection::CameraFaultDetection(int image_width,int image_height)
    :image_height_(image_height/10)
    ,image_width_(image_width/10)
    ,pixel_num_of_part_image_abnormal_(image_width_*image_height_*3/4)
    ,image_contours_num_threshold_(image_width_*image_height_*3/4) //zoom_in_ratio*contour_ratio
    ,camera_occlusion_area_threshold_(image_width_*image_height_*3/4)
{
    int ret;
    ret=pthread_mutex_init(&image_reeoe_code_mutex_,nullptr);
    if(0!=ret)
{
    std::cout<<"camera error mutex not init"<<std::endl;
}
    is_image_blur_=false;
    is_image_black_=false;
    is_image_white_=false;

    is_image_part_gray_=false;
    is_image_part_black_=false;
    is_image_part_white_=false;

    is_image_partial_color_=false;
    is_image_partial_color_cnt_.clear();

    is_image_contours_=false;
    is_image_contours_cnt_.clear();

    is_image_blur_cnt_.clear();

    is_camera_near_occlusion_cnt_.clear();
    camera_occlusion_stat_pool_.clear();
    is_camera_near_occlusion_=false;
    is_camera_far_occlusion_=false;
    is_time_cirle_start_=false;

    is_image_partial_color_cnt_.assign(image_cnt_max_,false);
    is_image_contours_cnt_.assign(image_cnt_max_,false);
    is_image_blur_cnt_.assign(image_cnt_max_,false);
    is_camera_near_occlusion_cnt_.assign(image_cnt_max_,false);
    MatStats tmp_stats;
    for(int i=0;i<5;i++)
        tmp_stats.point[i]=0;
    camera_occlusion_stat_pool_.assign(image_cnt_max_,tmp_stats);
    boost::thread thrd(checkState);
//    logInit(0,1);
}
uint32_t CameraFaultDetection::getImageFault1(cv::Mat &in_image,long long time_stamp)
{
    long long time1=getTimeStamp();
    in_image_.release();
    in_image_gray_.release();
    in_image_=in_image.clone();
    if(isImgAbnormal())
    {
        time_stamp_=time_stamp;
        if(!is_time_cirle_start_)
        {
            last_time_stamp_=time_stamp_;
            last_image_=in_image_gray_.clone();
            is_time_cirle_start_=true;
        }
        isImgBlur();
        isCameraOcclusion();
    }
    uint32_t tmp_image_error1;
    if(0==is_camera_connect_flag_)
		//断线检测，100ms检测一次是否有图，五次无图为断线
        tmp_image_error1 = 469762369;
    if(1==is_image_black_)		
        tmp_image_error1 = 469762371;
    if(1==is_image_white_)
		//白图
        tmp_image_error1 = 469762370;
    if(1==is_image_part_black_)
		//部分黑图
        tmp_image_error1 = 469762372;
    if(1==is_image_part_gray_)
		//部分灰图
        tmp_image_error1 = 469762373;
    if(1==is_image_part_white_)
		//部分白图
        tmp_image_error1 = 469762374;
    if(1==is_image_partial_color_)
		//部分偏色
        tmp_image_error1 = 469762375;
    if(1==is_image_contours_)
		//坏点检测
        tmp_image_error1 = 469762376;
    if(1==is_image_blur_)
		//模糊
        tmp_image_error1 = 469762377;
    if(1==is_camera_near_occlusion_)
		//近处遮挡
        tmp_image_error1 = 469762378;
    if(1==is_camera_far_occlusion_)
		//远处遮挡
        tmp_image_error1 = 469762379;
    stateRegression();
    long long time2=getTimeStamp();
    is_one_time_finised_=true;
   std::cout<<"camera fault detection1 time is "<<time2-time1<<std::endl;
    return tmp_image_error1;
}

uint32_t CameraFaultDetection::getImageFault2(cv::Mat &in_image,long long time_stamp)
{
    long long time1=getTimeStamp();
    in_image_.release();
    in_image_gray_.release();
    in_image_=in_image.clone();
    if(isImgAbnormal())
    {
        time_stamp_=time_stamp;
        if(!is_time_cirle_start_)
        {
            last_time_stamp_=time_stamp_;
            last_image_=in_image_gray_.clone();
            is_time_cirle_start_=true;
        }
        isImgBlur();
        isCameraOcclusion();
    }
    uint32_t tmp_image_error2;
    if(0==is_camera_connect_flag_)
		//断线检测，100ms检测一次是否有图，五次无图为断线
        tmp_image_error2 = 469762380;
    if(1==is_image_black_)		
        tmp_image_error2 = 469762382;
    if(1==is_image_white_)
		//白图
        tmp_image_error2 = 469762381;
    if(1==is_image_part_black_)
		//部分黑图
        tmp_image_error2 = 469762383;
    if(1==is_image_part_gray_)
		//部分灰图
        tmp_image_error2 =469762384;
    if(1==is_image_part_white_)
		//部分白图
        tmp_image_error2 = 469762385;
    if(1==is_image_partial_color_)
		//部分偏色
        tmp_image_error2 = 469762386;
    if(1==is_image_contours_)
		//坏点检测
        tmp_image_error2 = 469762387;
    if(1==is_image_blur_)
		//模糊
        tmp_image_error2 = 469762388;
    if(1==is_camera_near_occlusion_)
		//近处遮挡
        tmp_image_error2 = 469762389;
    if(1==is_camera_far_occlusion_)
		//远处遮挡
        tmp_image_error2 = 469762390;
    stateRegression();
    long long time2=getTimeStamp();
    is_one_time_finised_=true;
   std::cout<<"camera fault detection1 time is "<<time2-time1<<std::endl;
    return tmp_image_error2;
}
bool CameraFaultDetection::isImgAbnormal()
{
    if(!in_image_.empty())
    {
        cv::cvtColor(in_image_,in_image_gray_,CV_BGR2GRAY);
        //all_black || all_white
        cv::Mat mat_mean,mat_stddev;
        cv::meanStdDev(in_image_gray_,mat_mean,mat_stddev);
        double m,s;
        m=mat_mean.at<double>(0,0);
        s=mat_stddev.at<double>(0,0);
        if(s==0 && m==255)
        {
            is_image_white_=true;
            is_camera_rev_data_flag_=false;
            char c[50];
            sprintf(c,"./%0d_white.jpg",white_image_cnt_);
            std::string str=c;
            cv::imwrite(str,in_image_);
            if(white_image_cnt_<10)
                white_image_cnt_++;
            else {
                white_image_cnt_=0;
            }
            return false;
        }
        else if(s==0 && m==0)
        {
            is_image_black_=true;
            char c[50];
            sprintf(c,"./%0d_black.jpg",black_image_cnt);
            std::string str=c;
            cv::imwrite(str,in_image_);
            if(black_image_cnt<10)
                black_image_cnt++;
            else {
                black_image_cnt=0;
            }
            return false;
        }
        else
        {
            is_camera_rev_data_flag_=true;
        }
        cv::resize(in_image_,in_image_,cv::Size(image_width_,image_height_));
        cv::resize(in_image_gray_,in_image_gray_,cv::Size(image_width_,image_height_));
//        cv::imshow("in_image",in_image_);
//        cv::waitKey(1);
        //part_black || part_white || part_gray
        calCameraHist();
        int pix_gray_sum_=0;
        for(int i=0;i<20;i++)
        {
            int image_pixel_num=int(image_gray_hist[i]);
            pix_gray_sum_+=image_pixel_num;
            //std::cout<<"part black is : "<<pix_gray_sum_<<"  "<<std::endl;
            if(pix_gray_sum_>pixel_num_of_part_image_abnormal_)
            {
                is_image_part_black_=true;
                cv::Mat tmp_image;
                cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
                char c[50];
                sprintf(c,"./%0d_part_black_image.jpg",part_black_image_cnt_);
                std::string str=c;
                cv::imwrite(str,tmp_image);
                if(part_black_image_cnt_<10)
                    part_black_image_cnt_++;
                else {
                    part_black_image_cnt_=0;
                }
                break;
            }
        }
        pix_gray_sum_=0;
        for(int i=20;i<235;i++)
        {
            int image_pixel_num=int(image_gray_hist[i]);
            if(image_pixel_num>pixel_num_of_part_image_abnormal_)
            {
                is_image_part_gray_=true;
                cv::Mat tmp_image;
                cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
                char c[50];
                sprintf(c,"./%d_part_gray_image.jpg",part_gray_image_cnt_);
                std::string str=c;
                cv::imwrite(c,tmp_image);
                if(part_gray_image_cnt_<10)
                    part_gray_image_cnt_++;
                else {
                    part_gray_image_cnt_=0;
                }
                break;
            }
        }
        pix_gray_sum_=0;
        for(int i=235;i<255;i++)
        {
            int image_pixel_num=int(image_gray_hist[i]);
            if(image_pixel_num>pixel_num_of_part_image_abnormal_)
            {
                is_image_part_white_=true;
                cv::Mat tmp_image;
                cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
                char c[50];
                sprintf(c,"./%d_part_white_image.jpg",part_white_image_cnt_);
                std::string str=c;
                cv::imwrite(str,tmp_image);
                if(part_white_image_cnt_<10)
                    part_white_image_cnt_++;
                else {
                    part_white_image_cnt_=0;
                }
                break;
            }
        }
        //std::cout<<endl<<endl<<" is image part gray  "<<is_image_part_gray_<<"  "<<is_image_part_black_<<"  "<<is_image_part_white_<<std::endl;
          //is_image_patical_color_check
        partialColorJudge();
//        for(int i=0;i<is_image_partial_color_cnt_.size();i++)
//            std::cout<<is_image_partial_color_cnt_[i]<<"  ";
//        std::cout<<std::endl<<std::endl;
        is_image_partial_color_cnt_.pop_front();
        int is_image_paritial_color=0;
        for(size_t i=0;i<is_image_partial_color_cnt_.size();i++)
            if(is_image_partial_color_cnt_[i]==1)
                is_image_paritial_color++;
        if(is_image_paritial_color>image_cnt_thread_)
            is_image_partial_color_=true;
        //findContours
        findImageConturs();
        is_image_contours_cnt_.pop_front();
        int is_image_contours_cnt=0;
        for(auto ic:is_image_contours_cnt_)
            if(1==ic)
                is_image_contours_cnt++;
        if(is_image_contours_cnt>image_cnt_thread_)
            is_image_contours_=true;
        //std::cout<<"***************"<<is_image_contours_<<std::endl;
        return true;
    }
}
void CameraFaultDetection::isImgBlur()
{
    if(!in_image_.empty())
    {
        //Tenengrad梯度方法
        cv::Mat image_laplace;
        cv::Laplacian(in_image_gray_, image_laplace, CV_16U);
        //soble
        //cv::Sobel(image_gray, image_laplace, CV_16U, 1, 1);
        //std--dev
        //cv::Mat meanValueImage;
        //cv::Mat meanStdValueImage;
        //求灰度图像的标准差
        //cv::meanStdDev(imageGrey, meanValueImage, meanStdValueImage);
        //double meanValue = 0.0;
        //meanValue = cv::meanStdValueImage.at&lt;double&gt;(0, 0);
        //图像的平均灰度
        double meanValue = 0.0;
        meanValue = cv::mean(image_laplace)[0];
        is_image_blur_cnt_.pop_front();
        if (meanValue<image_blur_threshold_)
        {

            cv::Mat tmp_image;
            cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
            char c[50];
            sprintf(c,"./%0d_blur_image.jpg",blur_image_cnt_);
            std::string str=c;
            cv::imwrite(str,tmp_image);
            if(blur_image_cnt_<10)
                blur_image_cnt_++;
            else {
                blur_image_cnt_=0;
            }
            is_image_blur_cnt_.push_back(true);
        }
        else {
            is_image_blur_cnt_.push_back(false);
        }
        int is_image_blur_cnt=0;
        for(auto ib:is_image_blur_cnt_)
            if(1==ib)
                is_image_blur_cnt++;
        if(is_image_blur_cnt>image_cnt_thread_)
            is_image_blur_=true;
        return ;
    }
}
void CameraFaultDetection::isCameraOcclusion()
{
    if(!in_image_.empty())
    {
        //step1:near occlusion calculate:maybe error when night
        cv::Mat threashold_image=cv::Mat::zeros(in_image_gray_.size(),in_image_gray_.type());
        threashold_image=(in_image_gray_<=uchar(camera_occlusion_gray_threshold_));
//        cv::imshow("threshold image",threashold_image);
//        cv::waitKey(1);
        cv::Mat labels, stats, centroids, img_color, img_gray;
        int nccomps=cv::connectedComponentsWithStats(
                    threashold_image,//二值图像
                    labels,//和原图一样大的标记图
                    stats,//nccomps×5的矩阵 表示每个连通区域的外接矩形和面积（pixel）
                    centroids //nccomps×2的矩阵 表示每个连通区域的质心
                    );
        is_camera_near_occlusion_cnt_.pop_front();
        int i=1;
        //std::cout<<nccomps<<std::endl;
        while(i<nccomps)
        {
            if(stats.at<int>(i,cv::CC_STAT_AREA)>camera_occlusion_area_threshold_)
            {
                is_camera_near_occlusion_cnt_.push_back(true);
                cv::Mat tmp_image;
                cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
                char c[50];
                sprintf(c,"./%0d_near_occlusion.jpg",near_occlusion_image_cnt_);
                std::string str=c;
                cv::imwrite(str,tmp_image);
                if(near_occlusion_image_cnt_<10)
                    near_occlusion_image_cnt_++;
                else {
                    near_occlusion_image_cnt_=0;
                }
                break;
            }
            else
            {
                i++;
            }
        }
        if(i==nccomps)
        {
            is_camera_near_occlusion_cnt_.push_back(false);
        }
        i=0;
        int is_camera_near_occlusion=0;
int size=is_camera_near_occlusion_cnt_.size();
        for(;i<size;i++)
        {
            bool co=is_camera_near_occlusion_cnt_[i];
            if(true==co)
                is_camera_near_occlusion++;
        }
        if(is_camera_near_occlusion>image_cnt_thread_)
            is_camera_near_occlusion_=true;
        //step2:far occulation calculate
        float car_speed=0.1;
        float car_yaw_rate=0.0;
        int car_direction=0;
        //DataSendThread::getCarMotionInfo(&car_yaw_rate,&car_speed,&car_direction);
        //car_speed m/s
        if(is_time_cirle_start_)
        {
//            std::cout<<car_speed<<std::endl;
            int time_thread=car_speed>0.5?camera_occlusion_distance_/car_speed:1;
            int delta_time = (time_stamp_-last_time_stamp_)/1000; //s
//            std::cout<<time_stamp_<<"  "<<last_time_stamp_<<"  "<<delta_time<<std::endl;
            if(delta_time>time_thread)
            {
                cv::Mat diff_image,binary_image;
//                std::cout<<in_image_gray_.size()<<"  "<<last_image_.size()<<std::endl;

                cv::absdiff(in_image_gray_,last_image_,diff_image);
                cv::threshold(diff_image,binary_image,0,255,cv::THRESH_OTSU);
                cv::bitwise_not(binary_image,binary_image);
//                cv::imshow("diff_image",binary_image);
//                cv::waitKey(1);
                cv::Mat labels, stats, centroids, img_color, img_gray;
                int nccomps=cv::connectedComponentsWithStats(
                            threashold_image,//二值图像
                            labels,//和原图一样大的标记图
                            stats,//nccomps×5的矩阵 表示每个连通区域的外接矩形和面积（pixel）[x0, y0, width0, height0, area0;
                            centroids //nccomps×2的矩阵 表示每个连通区域的质心
                            );
                MatStats tmp_stats;
                for(int i=0;i<5;i++)
                    tmp_stats.point[i]=0;
                int tmp_area=0;
                int cnt=0;
                //std::cout<<"nccomps:  "<<nccomps<<std::endl;
                for(int i=1;i<nccomps;i++)
                {
                    //std::cout<<stats.at<int>(i,cv::CC_STAT_AREA)<<"  "<<stats.at<int>(cnt,2) <<"  "<<image_width_<<"  "<<stats.at<int>(cnt,3)<<std::endl;
                    if(stats.at<int>(i,cv::CC_STAT_AREA)>camera_occlusion_area_threshold_)
                    {
                        if(tmp_area<=stats.at<int>(i,cv::CC_STAT_AREA))
                            cnt=i;
                    }
                }
//                //std::cout<<std::endl<<std::endl;
                if(cnt>0)
                {
                    tmp_stats.point[0]=stats.at<int>(cnt,0);
                    tmp_stats.point[1]=stats.at<int>(cnt,1);
                    tmp_stats.point[2]=stats.at<int>(cnt,2);
                    tmp_stats.point[3]=stats.at<int>(cnt,3);
                    tmp_stats.point[4]=0;
                }
//                std::cout<<tmp_stats.point[0]<<" "<<tmp_stats.point[1]<<"  "<<tmp_stats.point[2]<<"  "<<tmp_stats.point[3]<<"  "<<tmp_stats.point[4]<<std::endl;
                camera_occlusion_stat_pool_.pop_front();
                camera_occlusion_stat_pool_.push_back(tmp_stats);
                int tmp_cnt=0;

                for(size_t i=0;i<camera_occlusion_stat_pool_.size()-1;i++)
                {
                    MatStats stats_1;
                    stats_1=camera_occlusion_stat_pool_[i];
                    if(stats_1.point[4]==0)
                    {
                        for(size_t j=i+1;j<camera_occlusion_stat_pool_.size();j++)
                        {
                            MatStats stats_2;
                            stats_2=camera_occlusion_stat_pool_[j];
                            if(stats_2.point[4]==0)
                            {
                                if(isImageOverlapped(stats_1.point,stats_2.point))
                                {
                                    camera_occlusion_stat_pool_[i].point[4]=1;
                                    camera_occlusion_stat_pool_[j].point[4]=1;
                                    tmp_cnt++;
                                    continue;
                                }
                            }
                        }
                    }
                }
                for(size_t i=0;i<camera_occlusion_stat_pool_.size();i++)
                {
                    camera_occlusion_stat_pool_[i].point[4]=0;
                }
                if(tmp_cnt>image_cnt_thread_/2)
                    is_camera_far_occlusion_=true;
                last_image_=in_image_gray_.clone();
                last_time_stamp_=time_stamp_;
            }
        }
        return;
    }
}
void CameraFaultDetection::calCameraHist()
{
    for(int i=0;i<256;i++)
        image_gray_hist[i]=0;
    cv::Mat_<uchar>::iterator it = in_image_gray_.begin<uchar>();
    //std::cout<<in_image_gray_.rows<<"  "<<in_image_gray_.cols<<std::endl;
    for(int i=0;i<in_image_gray_.rows;i++)
        for(int j=0;j<in_image_gray_.cols;j++)
        {
            int pixel=int(*(it + i * in_image_gray_.cols + j));
            image_gray_hist[pixel]++;
        }
    return ;
//    for(int i=0;i<256;i++)
//        std::cout<<image_gray_hist[i]<<"  ";
//    std::cout<<std::endl<<std::endl<<std::endl<<std::endl;
}
//true代表存在偏色，false代表不存在偏色
void CameraFaultDetection::partialColorJudge()
{
    cv::Mat lab_image;
    cv::cvtColor(in_image_,lab_image,CV_BGR2Lab);
//    cv::imshow("lab_image",lab_image);
//    cv::waitKey(1);
    cv::Mat_<cv::Vec3b>::iterator begin = lab_image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator end = lab_image.end<cv::Vec3b>();
    float suma = 0, sumb = 0;
    for (; begin != end; begin++)
    {
        suma += (*begin)[1];//a
        sumb += (*begin)[2];//b
    }
    int MN = lab_image.rows * lab_image.cols;
    double Da = suma / MN - 128; //归一化到[-128,127]
    double Db = sumb / MN - 128; //同上
    //求平均色度
    double D = sqrt(Da * Da + Db * Db);
    begin = lab_image.begin<cv::Vec3b>();
    double Ma = 0, Mb = 0;
    //求色度中心距
    for (; begin != end; begin++) {
        Ma += abs((*begin)[1] - 128 - Da);
        Mb += abs((*begin)[2] - 128 - Db);
    }
    Ma = Ma / MN;
    Mb = Mb / MN;
    double M = sqrt(Ma * Ma + Mb * Mb);
    float K = float(D / M);
    if (K >= 1.5)
    {
        is_image_partial_color_cnt_.push_back(true);
        cv::Mat tmp_image;
        cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
        char c[50];
        sprintf(c,"./%0d_part_color_image.jpg",part_color_image_cnt_);
        std::string str=c;
        cv::imwrite(str,tmp_image);
        if(part_color_image_cnt_<10)
            part_color_image_cnt_++;
        else {
            part_color_image_cnt_=0;
        }
    }
    else
    {
        is_image_partial_color_cnt_.push_back(false);
    }
    return;
}
void CameraFaultDetection::findImageConturs()
{
    cv::Mat new_image;
    cv::Size zoom_in_size=cv::Size(in_image_gray_.cols/image_width_zoom_in_rate_,in_image_gray_.rows/image_height_zoom_in_rate_);
    new_image=cv::Mat(zoom_in_size,in_image_gray_.type());
    cv::resize(in_image_gray_,new_image,zoom_in_size);
    //std::cout<<"new_image type is "<<new_image.channels()<<std::endl;
    for(int i=0;i<256;i++)
    {
        cv::Mat threashold_image=cv::Mat::zeros(new_image.size(),new_image.type());
//        threashold_image=(new_image>=uchar(i));
//        threashold_image=(threashold_image<=uchar(i+25));
//        cv::bitwise_not(threashold_image,threashold_image);
//                cv::imshow("threshold image",threashold_image);
//                cv::waitKey(1);
        threashold_image=(new_image==uchar(i));
        cv::Mat labels, stats, centroids, img_color, img_gray;
        int nccomps=cv::connectedComponentsWithStats(
                    threashold_image,//二值图像
                    labels,//和原图一样大的标记图
                    stats,//nccomps×5的矩阵 表示每个连通区域的外接矩形和面积（pixel）
                    centroids //nccomps×2的矩阵 表示每个连通区域的质心
                    );
        for(int n=1;n<nccomps;n++)
        {
            if(stats.at<int>(n,cv::CC_STAT_AREA)>image_contours_num_threshold_)
            {
//                std::cout<<"i &&&&&&&&&&&&"<< i <<std::endl;
               // std::cout<<"stats.at<int>(n,cv::CC_STAT_AREA)"<<stats.at<int>(n,cv::CC_STAT_AREA)<<std::endl;
                is_image_contours_cnt_.push_back(true);
                cv::Mat tmp_image;
                cv::resize(in_image_,tmp_image,cv::Size(image_width_*4,image_height_*4));
                char c[50];
                sprintf(c,"./%0d_contours_image.jpg",contours_image_cnt_);
                std::string str=c;
                cv::imwrite(str,tmp_image);
                if(contours_image_cnt_<10)
                    contours_image_cnt_++;
                else {
                    contours_image_cnt_=0;
                }
                return ;
            }
        }
    }
    is_image_contours_cnt_.push_back(false);
    return ;
}
void CameraFaultDetection::stateRegression()
{
    in_image_.release();
    in_image_gray_.release();
    is_image_blur_=false;
    is_image_black_=false;
    is_image_white_=false;
    is_image_part_gray_=false;
    is_image_part_black_=false;
    is_image_part_white_=false;
    is_image_partial_color_=false;
    is_image_contours_=false;
    is_camera_near_occlusion_=false;
    is_camera_far_occlusion_=false;
}
bool CameraFaultDetection::isImageOverlapped(int* in_1,int* in_2)
{
    if((!in_1[3] && !in_1[2])|| (!in_2[3]&&!in_2[2]))
    {
        return false;
    }
    //std::cout<<in_1[0]<<"  "<<in_1[1]<<"  "<<in_1[2]<<"  "<<in_1[3]<<"  "<<in_1[4]<<std::endl;
    //std::cout<<in_2[0]<<"  "<<in_2[1]<<"  "<<in_2[2]<<"  "<<in_2[3]<<"  "<<in_2[4]<<std::endl;
    int p1_x=in_1[0],p1_y = in_1[1];//第1个矩阵左下角
    int p2_x = p1_x + in_1[2], p2_y = p1_y + in_1[3];//第1个矩阵右上角
    int p3_x = in_2[0], p3_y = in_2[1];//第2个矩阵左下角
    int p4_x = p3_x + in_2[2], p4_y = p3_y + in_2[3];//第2个矩阵右上角
    if (p1_x > p4_x || p2_x < p3_x || p1_y > p4_y || p2_y < p3_y)
    {
        return false;
    }
    //长 = 右上角最小x - 左下角最大x
    //宽 = 右上角最小y - 左下角最大y
    int Len = std::min(p2_x, p4_x) - std::max(p1_x, p3_x);
    int Wid = std::min(p2_y, p4_y) - std::max(p1_y, p3_y);
    //std::cout<<Len<<"  "<<Wid<<std::endl;
    int overlapped_area=Len * Wid;
    int min_area=std::min(in_1[2]*in_1[3],in_2[2]*in_2[3]);
    int ratio=min_area/overlapped_area;
    //std::cout<<in_1[4]<<"   "<<in_2[4]<<"  min_area"<<min_area<<"  "<<overlapped_area<<"  "<<ratio<<std::endl;
    if(ratio>image_overlapped_ratio_)
        return false;
    else
        return true;
}
long long CameraFaultDetection::getTimeStamp()
{
    struct timeb tb;
    ftime(&tb);
    return tb.time*1000+tb.millitm;
}
void CameraFaultDetection::checkState()
{
    int cnt=0;
    while(1)
    {
        if(true == is_camera_rev_data_flag_)
        {
            cnt=0;
            is_camera_connect_flag_=true;
        }
        else
        {
            cnt++;
            if(cnt>5)
            {
                cnt=0;
                is_camera_connect_flag_=false;
            }
        }
        is_camera_rev_data_flag_=false;
        usleep(200000);//100ms
    }
}
