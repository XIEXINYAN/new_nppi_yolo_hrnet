#ifndef CSWITCH_H
#define CSWITCH_H
#include <perception/dds_msgs/decide_state_machinePubSubTypes.h>

#include <perception/dds_codes/dds_common.hpp>
#include <perception/dds_codes/dds_topic.hpp>
#include <boost/shared_ptr.hpp>

class CSwitch
{
public:
    CSwitch();
    ~CSwitch();
    bool getFlag(int& decision_num)
    {
        // return 1;
        if(decision_num !=m_decision_number)
        {
            decision_num = m_decision_number;
            return m_back_flag;
        }
        else
        {
            m_cnt++;
            if(m_cnt>6)
            {
                m_back_flag=false;
                m_cnt=0;
            }
            return m_back_flag;
        }
    }
private:
    bool m_back_flag=false;
    static int m_cnt;
    int32_t m_decision_number=-1;
    TopicDataType *m_decide_state_msg = nullptr;
    ddsSubscriber<TaskBehaviorState> *m_decide_state_sub=nullptr;
    void decideStateCallback(const TaskBehaviorState &dec_msg);
};

#endif // CSWITCH_H
