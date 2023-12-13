#include "ccameraprocess.h"
#include "common/ccameraparam.h"

sem_t CCameraProcess::m_timer_sem;

CCameraProcess::CCameraProcess()
{
    m_fault_alg_ptr.reset(new CFaultAlg());
    m_back_alg_ptr.reset(new CBackCameraAlg());
    m_front_alg_ptr.reset(new CFrontCameraAlg());

    //dingsshixinhaoliang
    int ret;
    ret = sem_init(&m_timer_sem, 0, 0);
    if(0!=ret)
    {
        cout<<"Camera timer sem init failure!"<<endl;
        return;
    }
    int delta_time = 100000;
    initTimer(0, delta_time);
    //chuli xiancheng
    pthread_t poll_id;
    ret = pthread_create(&poll_id, nullptr, pollThread, (void*)this);
    if(0!=ret)
    {
        cout<<"camera poll thread init faliture!"<<endl;
        return;
    }
    cout<<"camera process thread init"<<endl;
    return;
}

CCameraProcess::~CCameraProcess()
{
}
void *CCameraProcess::pollThread(void *arg)
{
   CCameraProcess *camera_thread = (CCameraProcess *)arg;
   while(1)
   {
       sem_wait(&camera_thread->m_timer_sem);
       while(sem_trywait(&camera_thread->m_timer_sem)>=0)
           ;
       {
           camera_thread->poll();
       }
   }
}
void CCameraProcess::poll()
{
    if(!CCameraParam::m_config_file_exist)
    {
        cout<<"cam config file is not found "<<endl;
        return;
    }
    bool is_back_flag=m_switch.getFlag(m_decision_number);
    // std::cout<<is_back_flag<<std::endl;
    if(is_back_flag)
    {
        m_back_alg_ptr->process();
    }
    else
    {
        m_front_alg_ptr->process();
    }
    return;
}
void CCameraProcess::initTimer(int sec, int usec)
{
    timer_t timerid;
    struct sigevent evp;
    memset(&evp, 0, sizeof (struct sigevent));
    evp.sigev_value.sival_int = 111;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = timerThread;
    if (timer_create(CLOCK_REALTIME, &evp, &timerid) == -1)
    {
        perror("fail to timer_create");
        exit(-1);
    }
    struct itimerspec it;
    it.it_interval.tv_sec = sec;
    it.it_interval.tv_nsec = usec * 1000;
    it.it_value.tv_sec = sec;
    it.it_value.tv_nsec = usec * 1000;
    if (timer_settime(timerid, 0, &it, NULL) == -1)
    {
        perror("fail to timer_settime");
        exit(-1);
    }
    return;
}

void CCameraProcess::timerThread(union sigval v)
{
    sem_post(&CCameraProcess::m_timer_sem);
}



