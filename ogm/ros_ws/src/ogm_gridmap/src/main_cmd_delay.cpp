#include "delay_node.h"

#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
    return init_delay_node<geometry_msgs::msg::Twist>(argc, argv);
}
