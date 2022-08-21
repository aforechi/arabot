#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "armbot");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Int8MultiArray>("arabot_arm", 100);
    ros::Rate loop_rate(1);

    std_msgs::Int8MultiArray array;
    array.data.resize(4);
    while (ros::ok()){
        array.data[0]=10;
        array.data[1]=10;
        array.data[2]=10;
        array.data[3]=10;
        pub.publish(array);
        ROS_INFO("I published joint angles!");
        ros::spinOnce();
        loop_rate.sleep();
    }
}
