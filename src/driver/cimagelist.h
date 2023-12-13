#ifndef CIMAGELIST_H
#define CIMAGELIST_H

#include <iostream>
#include <vector>
#include <string.h>
#include <vector>
#include "cframe.h"

class CImageList : public CFrame
{
public:
    CImageList(string images_path) : CFrame()
    {
        m_image_path=images_path;
        m_file_list.clear();
        getAllFileList(m_image_path);
        return;
    }
    ~CImageList()=default;
    void showImageList()
    {
        if(m_total_cnt == 0)
        {
            std::cout<<"no file in "<<m_image_path<<std::endl;
            return;
        }
        for(int i=0;i<m_total_cnt;i++)
        {
            std::cout<<m_file_list[i]<<std::endl;
        }
        return;
    }
    std::string getCurImageName()
    {
        int cnt = m_current_cnt==0?m_total_cnt-1:m_current_cnt-1;
        return m_file_list[cnt];
    }
    void writeImage() override;
private:
    int m_total_cnt=0;
    int m_current_cnt=-1;
    std::vector<std::string> m_file_list;
    void getAllFileList(std::string dirname);
    string m_image_path;
};

#endif // CIMAGELIST_H
