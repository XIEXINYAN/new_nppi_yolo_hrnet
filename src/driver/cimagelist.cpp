#include "cimagelist.h"
#include <dirent.h>
#include <sys/stat.h>

void CImageList::writeImage()
{
//    showImageList();
//    std::cout<<m_current_cnt<<"  "<<m_total_cnt<<std::endl;
    if(m_current_cnt == m_total_cnt)
    {
        std::cout<<"file list end"<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
//        m_current_cnt=0;
        return;
    }
    m_current_cnt++;
    m_time_stamp = getTimeStamp();
    std::string img_file = m_file_list[m_current_cnt];
    std::cout<<"current image is "<<img_file<<std::endl;
    cv::Mat bgr_img =cv::imread(img_file);
    m_err_api = cudaMemcpy(d_img_data, bgr_img.data, m_width*m_height*3*sizeof(unsigned char), cudaMemcpyHostToDevice);
    return;
}
void CImageList::getAllFileList(std::string dirname)
{
    if(nullptr == dirname.c_str())
    {
        std::cout << " dir_name is null ! " << std::endl;
        return ;
    }
    struct stat s;
    lstat(dirname.c_str(), &s);
    if(!S_ISDIR(s.st_mode))
    {
        std::cout << "dir_name is not a valid directory !" << std::endl;
        return;
    }
    struct dirent* filename;
    DIR *dir;
    dir = opendir(dirname.c_str());
    if (nullptr == dir)
    {
        std::cout << "Can not open dir " << dirname << std::endl;
        return;
    }
    while((filename = readdir(dir))!=nullptr)
    {
        if (strcmp(filename->d_name, ".") == 0 ||
            strcmp(filename->d_name, "..") == 0)
            continue;
        struct stat s;
        std::string sub_file = dirname+"/"+filename->d_name;
//        std::cout<<sub_file<<std::endl;
        lstat(sub_file.c_str(), &s);
        if(S_ISDIR(s.st_mode))
        {
            std::cout<<sub_file<<std::endl;
            getAllFileList(sub_file);
        }
        else {
//            std::cout<<"#####################"<<sub_file<<std::endl;
            m_file_list.push_back(sub_file);
        }
    }
    m_total_cnt = m_file_list.size();
//    showImageList();
    return;
}
