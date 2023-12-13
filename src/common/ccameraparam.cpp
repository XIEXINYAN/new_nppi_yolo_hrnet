#include "ccameraparam.h"

bool CCameraParam::m_config_file_exist=false;
bool CCameraParam::m_short_flag=false;
bool CCameraParam::m_long_flag=false;
bool CCameraParam::m_back_flag = false;
int CCameraParam::m_short_dev_num = -1;
int CCameraParam::m_long_dev_num = -1;
int CCameraParam::m_back_dev_num = -1;
string CCameraParam::m_short_config_file = "";
string CCameraParam::m_long_config_file = "";
int CCameraParam::m_width=1920;
int CCameraParam::m_height=1080;
int CCameraParam::m_camera=3;
float CCameraParam::m_short_install_angle=0.0;
float CCameraParam::m_long_install_angle=0.0;
bool CCameraParam::m_lidar_flag = false;
string CCameraParam::m_avi_path="";
string CCameraParam::m_images_path="";
bool CCameraParam::m_result_show = false;

CCameraParam::CCameraParam()
{
    string config_file_path;
    config_file_path = "../config/camera_params.txt";
    bool config_file_flag = isFileExist(config_file_path);
    if(!config_file_flag)
    {
        m_config_file_exist=false;
        return;
    }
    m_image_config_ptr.reset(new ConfigParser(config_file_path));
    m_short_dev_num =m_image_config_ptr->getInt("dev_num_sf");
    if(m_short_dev_num>=0)
    {
        m_short_config_file = m_image_config_ptr->getString("camera_dis_config_sf");
        if(!isFileExist(m_short_config_file))
        {
            m_config_file_exist=false;
            return;
        }
        int install_angle=m_image_config_ptr->getInt("camera_install_angle_sf");
        m_short_install_angle=float(install_angle)/180.0*3.1415;
        m_short_flag=true;
    }
    m_long_dev_num = m_image_config_ptr->getInt("dev_num_lf");
    if(m_long_dev_num>=0)
    {
        m_long_config_file = m_image_config_ptr->getString("camera_dis_config_lf");
        if(!isFileExist(m_long_config_file))
        {
            m_config_file_exist=false;
            return;
        }

        int install_angle=m_image_config_ptr->getInt("camera_install_angle_lf");
        m_short_install_angle=float(install_angle)/180.0*3.1415;
        m_long_flag=true;
    }
    m_config_file_exist=true;
    m_back_dev_num = m_image_config_ptr->getInt("dev_num_back");
    if(m_back_dev_num>=0)
    {
        m_back_flag=true;
    }
    m_camera = m_image_config_ptr->getInt("iscamera");
    m_width=m_image_config_ptr->getInt("image_width");
    m_height=m_image_config_ptr->getInt("image_height");
    std::cout<<"camera param set width: "<<m_width<<" height: "<<m_height<<std::endl;
    int lidar_flag=m_image_config_ptr->getInt("is_lidar_open");
    if(1==lidar_flag)
    {
        m_lidar_flag=true;
    }
    int result_show=m_image_config_ptr->getInt("is_result_show");
    if(1==result_show)
    {
        m_result_show=true;
    }
    if(m_camera==0)
    {
        m_avi_path=m_image_config_ptr->getString("videopath");
    }
    else if(m_camera ==2)
    {
        m_images_path=m_image_config_ptr->getString("imagepath");
    }
    m_config_file_exist=true;
    return;
}
bool CCameraParam::isFileExist(const string name)
{
    if(FILE* file=fopen(name.c_str(),"r"))
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}
