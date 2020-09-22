#include <ethercat_test/etherCAT_test.hpp>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"etherCAT");

    etherCAT_test ethercat_mobile_test;
    ethercat_mobile_test.control_input_subscriber_setting();
    ethercat_mobile_test.encoder_publisher_setting();

    ethercat_mobile_test.ecat_init();

    while (ros::ok())
    {
        ethercat_mobile_test.EPOS_OP();
        ros::spinOnce();

    }
    
}