#ifndef CAMERA_FAULT_DETECTION_H
#define CAMERA_FAULT_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include "math.h"
#include <deque>
#include <sys/timeb.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <assert.h>
#include <boost/thread/thread.hpp>
typedef struct
{
    int point[5];
}MatStats;
class CameraFaultDetection
{
public:
    CameraFaultDetection(int image_width,int image_height);
    ~CameraFaultDetection()=default;
    uint32_t getImageFault1(cv::Mat &in_image ,long long time_stamp);
    uint32_t getImageFault2(cv::Mat &in_image ,long long time_stamp);
private:
    const int save_image_size=10;
    int white_image_cnt_=0;
    int black_image_cnt=0;
    int part_black_image_cnt_=0;
    int part_gray_image_cnt_=0;
    int part_white_image_cnt_=0;
    int blur_image_cnt_=0;
    int near_occlusion_image_cnt_=0;
    int part_color_image_cnt_=0;
    int contours_image_cnt_=0;

    static pthread_mutex_t image_reeoe_code_mutex_;
    static bool is_one_time_finised_;
    cv::Mat in_image_;
    cv::Mat in_image_gray_;
    const int image_height_;
    const int image_width_;
    const int image_cnt_thread_=5;
    const int image_cnt_max_=10;

    void stateRegression();
    static unsigned char image_error_;
    /*
     * isImgAbnormal();
     */
    bool isImgAbnormal();//图像是否异常
    //image all black or white
    bool is_image_black_;
    bool is_image_white_;
    //image part black,white,gray
    void calCameraHist();
    int image_gray_hist[256];
    bool is_image_part_black_;
    bool is_image_part_white_;
    bool is_image_part_gray_;
    const int pixel_num_of_part_image_abnormal_;
    const float hist_ratio=0.1;

    //image part red,yellow,blue,green
    void partialColorJudge();
    bool is_image_partial_color_;
    static std::deque<bool> is_image_partial_color_cnt_;

    //image contours
    bool is_image_contours_;
    static std::deque<bool> is_image_contours_cnt_;
    void findImageConturs();
    const int image_contours_num_threshold_;
    const int image_width_zoom_in_rate_=2;
    const int image_height_zoom_in_rate_=2;

    //image blur
    void isImgBlur();//图像是否模糊
    bool is_image_blur_;
    static std::deque<bool> is_image_blur_cnt_;
    const float image_blur_threshold_=4.0;

    //CameraOcclusion
    void isCameraOcclusion();//摄像头是否遮挡
    bool is_camera_near_occlusion_;
    bool is_camera_far_occlusion_;
    const int camera_occlusion_gray_threshold_=30;
    const int camera_occlusion_area_threshold_;
    const int camera_occlusion_distance_=15;
    static std::deque<bool> is_camera_near_occlusion_cnt_;
    static std::deque<MatStats> camera_occlusion_stat_pool_;
    long long last_time_stamp_;
    long long time_stamp_;
    static bool is_time_cirle_start_;
    cv::Mat last_image_;
    const int image_overlapped_ratio_=3;
    bool isImageOverlapped(int* in_1,int* in_2);
    long long getTimeStamp();
    static void checkState();
    static bool is_camera_rev_data_flag_;
    static bool is_camera_connect_flag_;
};
#endif // CAMERA_FAULT_DETECTION_H
