#include "src/ccameraprocess.h"
#include <daemon_sdk.h>

int main(void)
{   
    //zkbs::init("syzk-camera");
    //zkbs::setVersion(VER("1.0.0"));//OTA部署修改
    //DaemonHandl Hdl = daemon_create("syzk-camera", 500, DEV_TYPE_A04);
    //mine_log::set_log_level(SANY_LOG_DEBUG);
    CCameraProcess a;

    while(1)
    {
        sleep(1);
    }
    //daemon_destroy(Hdl);
    return 0;
}

