
#include "pointcloud_node.h"

void Gridmap::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<LidarData> scans_arr;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
        iter_y(*msg, "y"), iter_z(*msg, "z"), iter_intensity(*msg, "intensity");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
            RCLCPP_DEBUG(
            this->get_logger(),
            "rejected for nan in point(%f, %f, %f)\n",
            *iter_x, *iter_y, *iter_z);
            continue;
        } // if
        float dist = hypot(*iter_x, *iter_y);

        if (3.0 < dist && dist < 70 && *iter_z > -0.4) {
            //std::cout << dist << std::endl;
            LidarData point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.intensity = *iter_intensity;
            scans_arr.push_back(point);
        } // if
    } // for
    //RCLCPP_INFO(
    //        this->get_logger(),
    //        "received cloud");

    if (scanning) {
        clear_map(&map);
        input(scans_arr);
        generate_scan();
        std::cout << "map done" << std::endl;
        publish_map();
    }

    else {
        
        float stamp = rosTimeToSeconds(msg->header.stamp);
        inputStamped(stamp, scans_arr); // store the scan
    }
} // function

void Gridmap::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //RCLCPP_INFO(
    //        this->get_logger(),
    //        "received state");
    if (!scanning) {
        Quaternion quat;
        quat.x = msg->pose.pose.orientation.x;
        quat.y = msg->pose.pose.orientation.y;
        quat.z = msg->pose.pose.orientation.z;
        quat.w = msg->pose.pose.orientation.w;
        Euler eul = quaternionToEuler(quat);
        if (!initialized_state()) {
            init_state(msg->pose.pose.position.x, msg->pose.pose.position.y, eul.yaw);
        }

        else {
            float stamp = rosTimeToSeconds(msg->header.stamp);
            save_state(stamp, msg->pose.pose.position.x, msg->pose.pose.position.y, eul.yaw);
            if (numberOfSubmaps() > 0) {
                publish_map();
                if (get_window_size()) {
                    generate_map();
                   
                }
            }
        }
    }
} // function

void Gridmap::map_build_callback() {
    if (initialized_state() && numberOfSubmaps() > 0) {
        if (!get_window_size()) { // window size is 0
            generate_map(); 
        }
    }
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //rclcpp::executors::MultiThreadedExecutor executor;
    //auto node = std::make_shared<Gridmap>();
    rclcpp::spin(std::make_shared<Gridmap>());
    //executor.add_node(node);
    //executor.spin();
    rclcpp::shutdown();
    return 0;
}