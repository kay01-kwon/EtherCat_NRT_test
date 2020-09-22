#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include <ethercat_test/vel.h>
#include "soem/ethercat.h"
#include "ethercat_test/pdo_def.h"
#include "ethercat_test/servo_def.h"
#include "ethercat_test/ecat_dc.h"

#define EC_TIMEOUTMON 500
#define NUMOFWHEEL_DRIVE    4
#define NSEC_PER_SEC 1000000000

using ethercat_test::vel;
using std::cout;
using std::endl;

unsigned int cycle_ns = 5e6; // nanosecond

EPOS4_Drive_pt	epos4_drive_pt[NUMOFWHEEL_DRIVE];
int started[NUMOFWHEEL_DRIVE]={0}, ServoState=0, TargetState=0;
uint8 servo_ready = 0, servo_prestate = 0;

struct sched_param shed_p;
char IOmap[4096];
pthread_t thread1, thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "eth0";
int run = 1;
int sys_ready = 0;
boolean limit_flag = FALSE;

int wait = 0;
int recv_fail_cnt = 0;

int32_t wheeldes[NUMOFWHEEL_DRIVE] = {0};

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;

boolean ecat_init(void)
{
    int i, oloop, iloop, chk, wkc_count;
    needlf = FALSE;
    inOP = FALSE;

    cout<<"Starting DC test"<<endl;
    if (ec_init(ecat_ifname))
    {
        cout<<"ec_init on" <<ecat_ifname<<" succeeded."<<endl;

        /* find and auto-config slaves in network */
        if (ec_config_init(FALSE) > 0)
        {
            cout<<ec_slavecount<<" slaves found and configured."<<endl;

                for (int k=1; k<(NUMOFWHEEL_DRIVE+1); ++k)
                {
                    if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k].name,"EPOS4") == 0)) //change name for other drives
                    {
                        cout<<"Re mapping for EPOS4 "<<k<<"..."<<endl;
                        os=sizeof(ob); ob = 0x00;	//RxPDO, check MAXPOS ESI
                        //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
                        wkc_count=ec_SDOwrite(k, 0x1c12, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                        wkc_count=ec_SDOwrite(k, 0x1c13, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                        /* TxPDO 1 maping: 0x1A00 */
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        // 1. StatusWord UINT16
                        os=sizeof(ob); ob = 0x60410010;
                        wkc_count=ec_SDOwrite(k, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);

                        // 2. VelocityActualValue INT32
                        os=sizeof(ob); ob = 0x606C0020;
                        wkc_count=ec_SDOwrite(k, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);

                        // 3. ErrorCode UINT16
                        os=sizeof(ob); ob = 0x603F0010;
                        wkc_count=ec_SDOwrite(k, 0x1A00, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);

                        os=sizeof(ob3); ob3 = 0x03;
                        wkc_count=ec_SDOwrite(k, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        if (wkc_count==0)
                        {
                            cout<<"TxPDO assignment error"<<endl;
                            //return FALSE;
                        }
                        
                        // Make other TxPDO mapping disabled
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1A01, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1A02, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1A03, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        // End of TxPDO mapping

                        /* RxPDO 1 mapping : 0x1600 */
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        // 1. ControlWord UINT16
                        os=sizeof(ob); ob = 0x60400010;
                        wkc_count=ec_SDOwrite(k, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                        
                        // 2. TargetVelocity INT32
                        os=sizeof(ob); ob = 0x60FF0020;
                        wkc_count=ec_SDOwrite(k, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x02;
                        wkc_count=ec_SDOwrite(k, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        if (wkc_count==0)
                        {
                            cout<<"RxPDO assignment error"<<endl;
                            //return FALSE;
                        }

                        // Make other RxPDO mapping disabled
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1601, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1602, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x00;
                        wkc_count=ec_SDOwrite(k, 0x1603, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                        // End of RxPDO mapping

                        os=sizeof(ob2); ob2 = 0x1600;
                        wkc_count=ec_SDOwrite(k, 0x1C12, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);

                        // Assign Sync Manger (2,3)                        
                        os=sizeof(ob3); ob3 = 0x01;
                        wkc_count=ec_SDOwrite(k, 0x1C12, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        os=sizeof(ob2); ob2 = 0x1A00;
                        wkc_count=ec_SDOwrite(k, 0x1C13, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                        
                        os=sizeof(ob3); ob3 = 0x01;
                        wkc_count=ec_SDOwrite(k, 0x1C13, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        os=sizeof(ob3); ob3 = 0x01;
                        wkc_count=ec_SDOwrite(k, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                        // CSV mode
                        os=sizeof(ob3); ob3 = 0X09;
                        wkc_count=ec_SDOwrite(k, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    }
                }


            ec_config_map(&IOmap);
            /* Configurate distributed clock */
            ec_configdc();
            cout<<"Slaves mapped, state to SAFE_OP."<<endl;
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /* configure DC options for every DC capable slave found in the list */
            cout<<"DC capable : "<<ec_configdc()<<endl;

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;

            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

            cout<<"segments : "<<ec_group[0].nsegments<<" : "<<ec_group[0].IOsegment[0]<<" "
            <<ec_group[0].IOsegment[1]<<" "<< ec_group[0].IOsegment[2]<<" "<<ec_group[0].IOsegment[3]<<endl;

            cout<<"Request operational state for all slaves"<<endl;

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            cout<<"Caculated workcounter "<<expectedWKC<<endl;
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* To enter state OP we need to send valid date to outpus */
            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            chk = 200;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                cout<<"Operational state reached for all slaves."<<endl;
                for (int k=0; k<NUMOFWHEEL_DRIVE; ++k)
                {
                    epos4_drive_pt[k].ptOutParam=(EPOS4_DRIVE_RxPDO_t*)  ec_slave[k+1].outputs;
                    epos4_drive_pt[k].ptInParam= (EPOS4_DRIVE_TxPDO_t*)  ec_slave[k+1].inputs;
                }
                inOP = TRUE;
            }
            else
            {
                cout<<"Not all slaves reached operational state."<<endl;
                ec_readstate();
                for (i=0; i<ec_slavecount; i++)
                {
                    if (ec_slave[i+1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State 0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i+1].state, ec_slave[i+1].ALstatuscode, ec_ALstatuscode2string(ec_slave[i+1].ALstatuscode));
                    }
                }
                for (i=0; i<NUMOFWHEEL_DRIVE; i++)
                    ec_dcsync0(i+1, FALSE, 0, 0);
            }
        }
        else
        {
            cout<<"No slaves found!"<<endl;
            inOP = FALSE;
        }
    }
    else
    {
        cout<<"No socket connection on "<<ecat_ifname<<endl<<"Execute as root"<<endl;
        return FALSE;
    }
    return inOP;
}

void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts -> tv_sec += sec;
    ts -> tv_nsec += nsec;
    if(ts -> tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts -> tv_nsec % NSEC_PER_SEC;
        ts -> tv_sec += (ts -> tv_nsec - nsec) / NSEC_PER_SEC;
        ts -> tv_nsec = nsec;
    }
}


void wheel_callback(const vel& msg)
{
    for(int i = 0 ; i < NUMOFWHEEL_DRIVE; i++)
        wheeldes[i] = msg.velocity[i];
}

void EPOS_OP()
{
    unsigned long ready_cnt = 0;
    uint16_t controlword=0;

    int argc;
    char** argv;

    vel msg2;
    ros::init(argc, argv, "dualarm_pub");

    ros::NodeHandle n;
    ros::Publisher pub2 = n.advertise<vel>("measure",1);
    
    int i;

    if (ecat_init()==FALSE)
    {
        run = 0;
        printf("EPOS INIT FAIL");
        return;
    }
    osal_usleep(1e3);

    struct timespec ts, tleft;
    struct timespec rt_ts;

    /* Distributed clock set up */
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000;
    long long diff_dc32;

    for (i=0; i<NUMOFWHEEL_DRIVE; ++i)
        ec_dcsync0(i+1, TRUE, cycle_ns, 0);


    long long cycletime = cycle_ns;
    long long cur_time = 0; // current master time
    long long cur_cycle_cnt = 0; // number of cycles has been passed
    long long cycle_time;  // cur_cycle_cnt*cycle_ns
    long long remain_time; // cur_time%cycle_ns
    long long dc_remain_time; // remain time to next cycle of REF clock, cur_dc32 % cycletime

    ec_send_processdata();
    clock_gettime(CLOCK_MONOTONIC,&ts);
    ts.tv_sec = 0;
    cur_time = (ts.tv_nsec & 0xFFFFFFFF);
    cur_cycle_cnt = cur_time/cycle_ns;
    cycle_time = cur_cycle_cnt*cycle_ns;
    remain_time = cur_time%cycle_ns;

    cout<<"cycles have been passed : "<<cur_cycle_cnt<<endl;
    cout<<"remain time to next cycle : "<<remain_time<<endl;

    wkc = ec_receive_processdata(EC_TIMEOUTRET); // get reference DC time
    cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF); // consider only lower 32-bit, because epos has 32-bit processor
    dc_remain_time = cur_dc32%cycletime;
    rt_ts.tv_nsec = cycle_time + dc_remain_time; // update master time to REF clock
    rt_ts.tv_sec = 0;

    cout<<"DC remain time to next cycle : "<<dc_remain_time<<endl;
    cout<<run<<endl;

    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&rt_ts,&tleft);

    while (run)
    {
        // wait for next cycle
        add_timespec(&ts,cycle_ns + toff);
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&ts,&tleft);

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        //cout<<ts.tv_nsec<<endl;
        if (wkc < 3*NUMOFWHEEL_DRIVE)
            recv_fail_cnt++;

        cur_dc32 = (uint32_t) (ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32>pre_dc32)
            diff_dc32 = cur_dc32-pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        //servo-on
        for (i=0; i<NUMOFWHEEL_DRIVE; ++i)
        {
            controlword=0;
            started[i]=ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[i].ptOutParam->ControlWord=controlword;
            if (started[i]) ServoState |= (1<<i); //started[i] is same as enable
        }

        if (ServoState == (1<<NUMOFWHEEL_DRIVE)-1) //all servos are in ON state
        {
            if (servo_ready==0) {
                servo_ready = 1;
            }
        }
        if (servo_ready) ready_cnt++;
        if (ready_cnt>=1000) //wait for 1s after servo-on
        {
            ready_cnt=10000;
            sys_ready=1;
        }

        if (sys_ready)
        {
            for (i=0; i<NUMOFWHEEL_DRIVE; ++i)
            {
                epos4_drive_pt[i].ptOutParam->TargetVelocity=wheeldes[i];
                msg2.velocity[i] = epos4_drive_pt[0].ptInParam->VelocityActualValue;
                //cout<<wheeldes[i]<<"\t";
                pub2.publish(msg2);
            }
            //cout<<endl;
            ros::spinOnce();

        } // sysready
        else
        {
            for (i=0; i<NUMOFWHEEL_DRIVE; ++i)

            {
                epos4_drive_pt[i].ptOutParam->TargetVelocity=0;
            }
        }

            //input_diagnosis.omega1 = epos4_drive_pt[0].ptOutParam->TargetVelocity;
            //publisher_input1.publish(input_diagnosis);
        if (sys_ready)
            if (worst_time<ethercat_time) worst_time=ethercat_time;

        if (limit_flag)
        {
            wait += 1;
            if (wait == 2000)
                run = 0;
        }
    } //while run

    osal_usleep(1e3);

    for (i=0; i<NUMOFWHEEL_DRIVE; ++i)
        ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFWHEEL_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=2; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    osal_usleep(1e3);

    printf("End EPOS CSV control, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

    ec_close();

}


void pub_run(void *arg)
{
    int i=0;
    unsigned long itime = 0;
    long stick = 0;
    int argc;
    char** argv;
    
    vel msg2;

    ros::init(argc, argv, "dualarm_pub");

    ros::NodeHandle n;

    ros::Publisher pub2 = n.advertise<vel>("measure",1);

    while (run)
    {
        if (inOP==TRUE)
        {
            if (!sys_ready)
            {
                if(stick==0)
                    printf("waiting for system ready...\n");
                if(stick%10==0)

                stick++;
            }
            else
            {
                itime++;

                for(int i = 0; i < NUMOFWHEEL_DRIVE; i++)
                    msg2.velocity[i] = epos4_drive_pt[0].ptInParam->VelocityActualValue;
                
                pub2.publish(msg2);

            }
        }
        osal_usleep(5e3);
    }
}


int main(int argc, char** argv)
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    printf("use default adapter %s\n", ecat_ifname);

    ros::init(argc, argv, "dualarm_sub");
    ros::NodeHandle n;
    ros::Subscriber sub3 = n.subscribe("input_msg",1, wheel_callback);
    
    ecat_init();
    EPOS_OP();

    printf("End program\n");
    return (0);

}