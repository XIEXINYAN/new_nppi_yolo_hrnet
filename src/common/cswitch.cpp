#include "cswitch.h"

int CSwitch::m_cnt=0;

CSwitch::CSwitch()
{
    //decision flag
    m_decide_state_msg = new TaskBehaviorStatePubSubType();
    m_decide_state_sub = new ddsSubscriber<TaskBehaviorState>( m_decide_state_msg,
                                                               std::bind(&CSwitch::decideStateCallback,this, std::placeholders::_1));
}
CSwitch::~CSwitch()
{
    if(m_decide_state_msg)
    {
        delete m_decide_state_msg;
        m_decide_state_msg = nullptr;
    }
    if(m_decide_state_sub)
    {
        delete m_decide_state_sub;
        m_decide_state_sub = nullptr;
    }
}
void CSwitch::decideStateCallback(const TaskBehaviorState &dec_msg)
{
    m_decision_number = dec_msg.number();
    auto decide_state = dec_msg.vehicleDrive();
    switch (decide_state)
    {
    case Task_Drive_State::TASK_DUMPING_PREVIEW_BACK:
        m_back_flag=true;
        break;
    case Task_Drive_State::TASK_LOADING_PREVIEW_BACK:
        m_back_flag=true;
        break;
    default:
        m_back_flag=false;
        break;
    }
    return;
}
