#include "cfaultalg.h"

using namespace std;

CFaultAlg::CFaultAlg()
{
    // 创建sdk实例
    m_pFailureSdk = new FailureReporter();
    // 设置故障诊断模块id
    m_pFailureSdk->setModule(FailureModule_Perception_Camera);
    // 调用init函数
    m_pFailureSdk->init();
    // 调用start函数
    m_pFailureSdk->start();

    cout<<"fault alg inited"<<endl;
    return;
}
CFaultAlg::~CFaultAlg()
{
    if(m_pFailureSdk)
    {
        delete m_pFailureSdk;
        m_pFailureSdk=nullptr;
    }
}
