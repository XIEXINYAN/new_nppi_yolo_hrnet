#ifndef CCAMERAPROCESS_H
#define CCAMERAPROCESS_H

#include <semaphore.h>
#include <signal.h>
#include <sys/timeb.h>
#include <sys/stat.h>
#include "common/cswitch.h"
#include "front_alg/cfrontcameraalg.h"
#include "fault_alg/cfaultalg.h"
#include "back_alg/cbackcameraalg.h"
#include "common/ccameraparam.h"

class CCameraProcess
{
public:
    CCameraProcess();
    ~CCameraProcess();
private:
    static sem_t m_timer_sem;
    void initTimer(int sec, int usec);
    static void timerThread(union sigval v);
    static void *pollThread(void* arg);
    void poll();

    int32_t m_decision_number=-1;
    CSwitch m_switch;
    CFrontCameraAlgPtr m_front_alg_ptr;
    CFaultAlgPtr m_fault_alg_ptr;
    CBackCameraAlgPtr m_back_alg_ptr;
    CCameraParam m_camera_param;
};
#endif // CCAMERAPROCESS_H
