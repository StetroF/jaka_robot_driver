#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <pthread.h>
// #include "timespec.h"
#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)
// int64_t edg_timespec2ns(timespec t)
// {
//     return t.tv_sec * 1000000000 + t.tv_nsec;
// }
// void edg_sync(timespec reftime_, int64_t *sys_time_offset)
// {
//     auto reftime = edg_timespec2ns(reftime_);
//     static int64_t integral = 0;
//     int64_t cycletime = 1000000;
//     int64_t delta = (reftime - 0) % cycletime;
//     if (delta > (cycletime / 2))
//     {
//         delta = delta - cycletime;
//     }
//     if (delta > 0)
//     {
//         integral++;
//     }
//     if (delta < 0)
//     {
//         integral--;
//     }
//     if (sys_time_offset)
//     {
//         *sys_time_offset = -(delta / 100) - (integral / 20);  //类似PI调节
//     }
// }
timespec timespec_add(timespec t1, timespec t2)
    {
        timespec result;
        result.tv_sec = t1.tv_sec + t2.tv_sec;
        result.tv_nsec = t1.tv_nsec + t2.tv_nsec;
        if (result.tv_nsec >= 1000000000)
        {
            result.tv_sec += 1;
            result.tv_nsec -= 1000000000;
        }
        return result;
    }
int servoj_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    
    double v[] = {deg2rad(30),deg2rad(30)};
    double a[] = {deg2rad(150),deg2rad(150)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,v,a);
    robot.servo_move_use_none_filter();
    // robot.servo_move_use_joint_LPF(125.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    
    JointValue jpos_cmd;
    memset(&jpos_cmd,0,sizeof(jpos_cmd));
    JointValue jpos_cmd2;
    memset(&jpos_cmd2,0,sizeof(jpos_cmd2));

    bool rob1_change_to_servo = true;

    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, -1);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    int rob2_start_k = 0;
    for(int i = 0;;i++)
    {
        timespec max_wkup_time;
        robot.edg_recv(&next);
        timespec cur_time;
        timespec cc;
        clock_gettime(CLOCK_REALTIME, &cc);

        JointValue jpos[2];
        CartesianPose cpos[2];
        int64_t recv_time = 0;
        robot.edg_get_stat(0, &jpos[0], &cpos[0]);
        robot.edg_get_stat(1, &jpos[1], &cpos[1]);
        unsigned long int details[3];
        robot.edg_stat_details(details);
    
        double t = (i - 0)/10000.0;
        double kk = 1;
        jpos_cmd.jVal[0] = -(sin(kk*t)*30.0/180.0*3.14);
        jpos_cmd.jVal[1] = -(-cos(kk*t)*20.0/180.0*3.14 + 20.0/180.0*3.14);
        jpos_cmd.jVal[3] = -(-cos(kk*t)*10.0/180.0*3.14 + 10.0/180.0*3.14);
        // robot.edg_servo_j(0,&jpos_cmd,MoveMode::ABS);
        if(rob1_change_to_servo == true && rob2_start_k == 0)
        {
            rob2_start_k = i;
        }

        if(rob2_start_k && rob1_change_to_servo == true)
        {
            double tt = (i - rob2_start_k)/10000.0;
            double kkk = 15;
            printf("set robot2\n");
            jpos_cmd2.jVal[0] = -(sin(kkk*tt)*30.0/180.0*3.14);
            jpos_cmd2.jVal[1] = -(-cos(kkk*tt)*20.0/180.0*3.14 + 20.0/180.0*3.14);
            jpos_cmd2.jVal[3] = -(-cos(kkk*tt)*10.0/180.0*3.14 + 10.0/180.0*3.14);
        }
        robot.edg_servo_j(1, &jpos_cmd2, MoveMode::ABS);
        robot.edg_send();
        
#if 1
        printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %ld %ld %ld\n", 
        //std::chrono::duration_cast<std::chrono::nanoseconds>(cur.time_since_epoch()).count(),
        // edg_timespec2ns(cc),
        // edg_timespec2ns(cur_time),
        // edg_timespec2ns(max_wkup_time),
        jpos[0].jVal[0], jpos[0].jVal[1], jpos[0].jVal[3],
        jpos[1].jVal[0], jpos[1].jVal[1], jpos[1].jVal[3],
        jpos_cmd.jVal[0], jpos_cmd.jVal[1], jpos_cmd.jVal[3],
        details[0], details[1], details[2]
        );


        
#endif
        //等待下一个周期
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
}


int main()
{
    //
    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");

    robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    robot.servo_move_use_none_filter();
    robot.motion_abort();
    robot.power_on();
    robot.enable_robot();
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    
    servoj_test(robot);
    // servop_test(robot);
    
    robot.login_out();
    return 0;
}