#include <ros/ros.h>
#include "dumping_line_detection/convolution_core.h"

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL," ");
    ros::init(argc,argv,"dumping_line_detection");
    convolution_ns::convolution convolutioner;
    convolutioner.main_loop();

    return 0;
}