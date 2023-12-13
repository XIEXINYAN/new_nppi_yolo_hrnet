/* author : xxy
 * func : read the camera param file
 * out : configparserptr , fileexistflag
*/
#ifndef CCAMERAPARAM_H
#define CCAMERAPARAM_H
#include <perception/common/config_parser.hpp>
#include <stdio.h>
#include <iostream>
using namespace std;

class CCameraParam
{
public:
    CCameraParam();
    ~CCameraParam(){}
    static bool m_config_file_exist;
    static bool m_short_flag;
    static bool m_long_flag;
    static bool m_back_flag;
    static int m_short_dev_num;
    static int m_long_dev_num;
    static int m_back_dev_num;
    static string m_short_config_file;
    static string m_long_config_file;
    static int m_width;
    static int m_height;
    static int m_camera;
    static float m_short_install_angle;
    static float m_long_install_angle;
    static bool m_lidar_flag;
    static string m_avi_path;
    static string m_images_path;
    static bool m_result_show;
private:
    ConfigParserPtr m_image_config_ptr;
    bool isFileExist(const std::string name);   
};

#endif // CCAMERAPARAM_H
