#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "semantic_kitti_loader.hpp"
#include "utils.hpp"

using PointType = PointXYZILID;
using namespace std;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

int main(int argc, char**argv) {

    std::string seq;
    std::string data_path;
    
    int         init_idx;
    bool        stop_per_each_frame;

    std::map<int, string> label_map;
    std::map<int, vector<int>> color_map;

    std::vector<int> labels = { 0, 1, 10, 11, 13, 15, 16, 18, 20, 30, 31, 32, 40, 44,
                                48, 49, 50, 51, 52, 60, 70, 71, 72, 80, 81, 99, 252,
                                256, 253, 254, 255, 257, 258, 259 };

    ros::init(argc, argv, "Offline_KITTI");

    ros::NodeHandle nh;
    
    signal(SIGINT, signal_callback_handler);

    nh.param<string>("/sequence", seq, "00");
    nh.param<string>("/data_path", data_path, "/");
    nh.param<int>("/init_idx", init_idx, 0);
    nh.param<bool>("/stop_per_each_frame", stop_per_each_frame, false);
    
    data_path = data_path + "/" + seq;
    
    for (int label: labels) {
        string color_param = "/color_map/color" + std::to_string(label);
        string label_param = "/label_map/label" + std::to_string(label);

        nh.getParam(color_param, color_map[label]);
        nh.getParam(label_param, label_map[label]);
    }

    ros::Publisher pub_cloud   = nh.advertise<sensor_msgs::PointCloud2>("/colored_cloud", 1, true);
    ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/markers", 1, true);
    
    visualization_msgs::MarkerArray marker_arr;

    int cnt = 0;
    for (int label: labels) {

        string label_name = label_map[label];
        vector<int> color = color_map[label];

        cout << label << " " << label_name << " " << color[0] << " " << color[1] << " " << color[2] << endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "label";
        marker.id = cnt;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = label_name;

        marker.pose.position.x = -50 + 3 * cnt;
        marker.pose.position.y = 50;
        marker.pose.position.z = 1;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 3.0;

        marker.color.a = 1.0;
        marker.color.b = (float) (color[0]/255.0);
        marker.color.g = (float) (color[1]/255.0);
        marker.color.r = (float) (color[2]/255.0);

        marker_arr.markers.push_back(marker);
        cnt++;
    }

    SemanticKittiLoader<PointType> loader(data_path);

    int      N = loader.size();
    for (int n = init_idx; n < N; ++n) {     

        cout << n << "th node come" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);

        pcl::PointCloud<pcl::PointXYZRGB> pc_curr_color;

        for (auto pt: pc_curr) {
            pcl::PointXYZRGB pt_color;
            pt_color.x = pt.x;
            pt_color.y = pt.y;
            pt_color.z = pt.z;

            vector<int> color = color_map[pt.label];
            pt_color.r = color[0];
            pt_color.g = color[1];
            pt_color.b = color[2];

            pc_curr_color.points.push_back(pt_color);
        }

        pub_markers.publish(marker_arr);
        pub_cloud.publish(cloud2msg(pc_curr_color));
        ros::spinOnce();

        if (stop_per_each_frame) cin.ignore();
    }

    return 0;
}
