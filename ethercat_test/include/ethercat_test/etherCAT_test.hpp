#include <ros/ros.h>
#include <ethercat_test/vel.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <soem/ethercat.h>

#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#define EC_TIMEOUTMON 500
#define NUMOFWHEEL_DRIVE 4
#define NSEC_PER_SEC 1000000000

using ethercat_test::vel;
using std::cout;

// Cycle nano second : 10 ms
unsigned int cycle_ns = 10e6;

EPOS4_Drive_pt epos4_drive_pt[NUMOFWHEEL_DRIVE];
int started[NUMOFWHEEL_DRIVE] = {0};
int ServoState = 0;
int TargetState = 0;

uint8 servo_ready = 0;
uint8 servo_prestate = 0;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

char ecat_ifname[32] = "eth0";
int run = 1;
int sys_ready = 0;

int32_t wheel_des[NUMOFWHEEL_DRIVE] = {0};

int os;
uint32_t ob_32;
uint16_t ob_16;
uint8_t ob_8;

unsigned long ready_cnt = 0;

struct timespec ts, tleft;
int64 t_ts = 0;

class etherCAT_test{

    public:
        boolean ecat_init();
        void encoder_publisher_setting();
        void callback_control_input(const vel &control_input);
        void control_input_subscriber_setting();
        void EPOS_OP();
        int64 addtimespec(struct timespec *ts, int64 addtime);
        void EC_configuration();
        void PDO_mapping(int k);
        
    private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_control_input;
    ros::Publisher pub_encoder_value;

};

boolean etherCAT_test::ecat_init()
{
    needlf = FALSE;
    inOP = FALSE;
    cout<<"Starting DC test\n";
    if(ec_init(ecat_ifname))
    {
        cout<<"ec_init on "<<ecat_ifname<<"succeeded. \n";
        if(ec_config_init(FALSE) > 0)
        {
            cout<<ec_slavecount<<" slaves found and configured. \n";

            for(int k = 1; k < NUMOFWHEEL_DRIVE + 1; ++k)
            {
                if((ec_slavecount >= 1) && (strcmp(ec_slave[k].name,"EPOS4") == 0))
                    etherCAT_test::PDO_mapping(k);
            }

            etherCAT_test::EC_configuration();
        }
        else
        {
            cout<<"No slaves is found. \n";
            inOP = FALSE;
        }
    }
    else
    {
        cout<<"No socket connection on "<<ecat_ifname<<"\n";
        inOP = FALSE;
    }

    if(inOP == TRUE)
    {
        for(int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
            ec_dcsync0(i+1, TRUE,cycle_ns,0);
    }

    return inOP;
}

void etherCAT_test::encoder_publisher_setting()
{
    pub_encoder_value = nh_.advertise<vel>("/measure",1);
}

void etherCAT_test::callback_control_input(const vel &control_input)
{
    for(int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
        wheel_des[i] = control_input.velocity[i];
}

void etherCAT_test::control_input_subscriber_setting()
{
    sub_control_input = nh_.subscribe("/input_msg",1,&etherCAT_test::callback_control_input,this);
}


void etherCAT_test::EPOS_OP()
{
    uint16_t controlword = 0;
    vel measure_msg;


    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Servo On
    for(int i = 0; i< NUMOFWHEEL_DRIVE; ++i)
    {
        controlword = 0;
        started[i] = ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord,&controlword);
        epos4_drive_pt[i].ptOutParam->ControlWord = controlword;
        if(started[i]) ServoState |= (1<<i); 
    }

    if(ServoState == (1<<NUMOFWHEEL_DRIVE)-1)
    {
        if(servo_ready == 0)
            servo_ready = 1;
    }
    if(servo_ready) ready_cnt++;
    if(ready_cnt>=1000)
    {
        ready_cnt = 10000;
        sys_ready = 1;
    }
    if(sys_ready)
    {
        for(int i = 0; i <NUMOFWHEEL_DRIVE; ++i)
            epos4_drive_pt[i].ptOutParam->TargetVelocity = wheel_des[i];
    }
    else
    {
        for(int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
            epos4_drive_pt[i].ptOutParam->TargetVelocity = 0;
    }

    for(int i = 0; i < NUMOFWHEEL_DRIVE; ++i)
        measure_msg.velocity[i] = epos4_drive_pt[i].ptInParam->VelocityActualValue;

    pub_encoder_value.publish(measure_msg);
}

int64 etherCAT_test::addtimespec(struct timespec *ts, int64 addtime)
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

    return ts->tv_nsec + ts->tv_sec * NSEC_PER_SEC;
}

void etherCAT_test::EC_configuration()
{
    int i, oloop, iloop,chk;
    ec_config_map(&IOmap);
            
    /* Configurate distributed clock */
    ec_configdc();
    cout<<"Slaves mapped, state to SAFE_OP.\n";
            
    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    /* configure DC options for every DC capable slave found in the list */
    cout<<"DC capable : "<<ec_configdc()<<"\n";

    oloop = ec_slave[0].Obytes;
    if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            
    iloop = ec_slave[0].Ibytes;
    if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

    cout<<"Request operational state for all slaves\n";
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    
    cout<<"Caculated workcounter"<<expectedWKC<<"\n";
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
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
        cout<<"Operational state reached for all slaves.\n";
        
        for (int k=0; k<NUMOFWHEEL_DRIVE; ++k)
        {
            epos4_drive_pt[k].ptOutParam=(EPOS4_DRIVE_RxPDO_t*)  ec_slave[k+1].outputs;
            epos4_drive_pt[k].ptInParam= (EPOS4_DRIVE_TxPDO_t*)  ec_slave[k+1].inputs;
        }
        
        inOP = TRUE;
            
    }
    else
    {
        cout<<"Not all slaves reached operational state.\n";
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


void etherCAT_test::PDO_mapping(int k)
{
    int wkc_count;
    cout<<"Re-mapping for EPOS4\t"<<k<<"\n";
    os = sizeof(ob_32); ob_32 = 0x00;
    // Sync manager 2 PDO Assignment (RxPDO)
    wkc_count = ec_SDOwrite(k, 0x1C12,0x00, 
    FALSE, os, &ob_32, EC_TIMEOUTRXM);
    
    // Sync manager 3 PDO Assignment (TxPDO)
    wkc_count = ec_SDOwrite(k, 0x1C13, 0x00,
    FALSE, os, &ob_32, EC_TIMEOUTRXM);

    /* TxPDO mapping : 0x1A00 */
    os = sizeof(ob_8); ob_8 = 0x00;
    wkc_count = ec_SDOwrite(k, 0x1A00, 0x00,
    FALSE, os, &ob_8, EC_TIMEOUTRXM);

    // 1. StatusWord UINT16
    os = sizeof(ob_32); ob_32 = 0x60410010;
    wkc_count = ec_SDOwrite(k, 0x1A00, 0x01,
    FALSE, os, &ob_32, EC_TIMEOUTRXM);

    // 2. VelocityActualValue INT32
    os = sizeof(ob_32); ob_32 = 0x606C0020;
    wkc_count = ec_SDOwrite(k, 0x1A00, 0x02,
    FALSE, os, &ob_32, EC_TIMEOUTRXM);

    // 3. ErrorCode UINT16
    os = sizeof(ob_32); ob_32 = 0x603F0010;
    wkc_count = ec_SDOwrite(k, 0x1A00, 0x03, 
    FALSE, os, &ob_32, EC_TIMEOUTRXM);

    os = sizeof(ob_8); ob_8 = 0x03;
    wkc_count = ec_SDOwrite(k, 0x1A00, 0x00,
    FALSE, os, &ob_8, EC_TIMEOUTRXM);

    // End Of TxPDO mapping

    if(wkc_count == 0)
        cout<<"TxPDO Assignment Error\n";
    
    /* Make Other TxPDO mapping disabled */
    os = sizeof(ob_8); ob_8 = 0x00;
    wkc_count = ec_SDOwrite(k, 0x1A01, 0x00,
    FALSE, os, &ob_8, EC_TIMEOUTRXM);

    wkc_count = ec_SDOwrite(k, 0x1A02, 0x00,
    FALSE, os, &ob_8, EC_TIMEOUTRXM);

    wkc_count = ec_SDOwrite(k, 0x1A03, 0x00,
    FALSE, os, &ob_8, EC_TIMEOUTRXM);

    /* RxPDO 1 mapping : 0x1600 */
    os=sizeof(ob_8); ob_8 = 0x00;
    wkc_count=ec_SDOwrite(k, 0x1600, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);
                        
    // 1. ControlWord UINT16
    os=sizeof(ob_32); ob_32 = 0x60400010;
    wkc_count=ec_SDOwrite(k, 0x1600, 0x01, 
	FALSE, os, &ob_32, EC_TIMEOUTRXM);
          
    // 2. TargetVelocity INT32
    os=sizeof(ob_32); ob_32 = 0x60FF0020;
    wkc_count=ec_SDOwrite(k, 0x1600, 0x02, 
	FALSE, os, &ob_32, EC_TIMEOUTRXM);
                    
    os=sizeof(ob_8); ob_8 = 0x02;
    wkc_count=ec_SDOwrite(k, 0x1600, 0x00, 
    FALSE, os, &ob_8, EC_TIMEOUTRXM);
    // End of RxPDO mapping

    if (wkc_count==0)
        cout<<"RxPDO Assignment Error\n";

    // Make other RxPDO mapping disabled
    os=sizeof(ob_8); ob_8 = 0x00;
    wkc_count=ec_SDOwrite(k, 0x1601, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);

    os=sizeof(ob_8); ob_8 = 0x00;
    wkc_count=ec_SDOwrite(k, 0x1602, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);
                        
	os=sizeof(ob_8); ob_8 = 0x00;
    wkc_count=ec_SDOwrite(k, 0x1603, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);

    // Assign Sync Manger (2, 3)
    os=sizeof(ob_16); ob_16 = 0x1600;
    wkc_count=ec_SDOwrite(k, 0x1C12, 0x01, 
	FALSE, os, &ob_16, EC_TIMEOUTRXM);

    os=sizeof(ob_8); ob_8 = 0x01;
    wkc_count=ec_SDOwrite(k, 0x1C12, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);

    os=sizeof(ob_16); ob_16 = 0x1A00;
    wkc_count=ec_SDOwrite(k, 0x1C13, 0x01, 
	FALSE, os, &ob_16, EC_TIMEOUTRXM);

    os=sizeof(ob_8); ob_8 = 0x01;
    wkc_count=ec_SDOwrite(k, 0x1C13, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);

    // Interpolation Time Period UINT8
    os=sizeof(ob_8); ob_8 = 0x01;
    wkc_count=ec_SDOwrite(k, 0x60C2, 0x01, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);

    // Mode of Operation : 0x08(CSP), 0x09(CSV) UINT8
    os=sizeof(ob_8); ob_8 = 0X09;
    wkc_count=ec_SDOwrite(k, 0x6060, 0x00, 
	FALSE, os, &ob_8, EC_TIMEOUTRXM);
    
}