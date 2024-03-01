#ifndef TRAJECTORY_DATA_H
#define TRAJECTORY_DATA_H

#include <iostream>
#include <vector>
#include <limits>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


class PixelPos{
    public:
    int x;
    int y;
    PixelPos(int x, int y): x(x), y(y){}

    bool operator==(const PixelPos& other) const {
        return (x == other.x) && (y == other.y);
    }
};


class WorldPos{
    public:
    float x;
    float y;
    WorldPos(float x, float y): x(x), y(y){}

    bool operator==(const WorldPos& other) const {
        return (x == other.x) && (y == other.y);
    }
};


class Waypoint {
public:
    PixelPos pixel_pos;
    WorldPos world_pos = WorldPos(0.0f, 0.0f);

    float occupied_from;
    float occupied_until = std::numeric_limits<float>::infinity();
    
    Waypoint* previous_waypoint;

    Waypoint(PixelPos pixel_pos, float occupied_from) 
        : pixel_pos(pixel_pos), occupied_from(occupied_from) {}

    Waypoint(PixelPos pixel_pos, float occupied_from, Waypoint* previous_waypoint) 
        : pixel_pos(pixel_pos), occupied_from(occupied_from), previous_waypoint(previous_waypoint) {}


     Waypoint(PixelPos pixel_pos, float occupied_from, float occupied_until, WorldPos world_pos, Waypoint* previous_waypoint) 
        : pixel_pos(pixel_pos), occupied_from(occupied_from), occupied_until(occupied_until), world_pos(world_pos), previous_waypoint(previous_waypoint) {}

    bool operator==(const Waypoint& other) const {
        return pixel_pos == other.pixel_pos;
    }

    bool operator<(const Waypoint& other) const {
        return occupied_from < other.occupied_from;
    }
};

class TrajectoryData {
public:
    int planner_id;
    std::vector<Waypoint> waypoints;
    Waypoint* start;
    Waypoint* goal;

    TrajectoryData(int planner_id, std::vector<Waypoint> waypoints) 
        : planner_id(planner_id), waypoints(waypoints) {
        start = waypoints.empty() ? nullptr : &waypoints[0];
        goal = waypoints.empty() ? nullptr : &waypoints.back();

        if (waypoints.empty()) {
            ROS_WARN("Planner %i: Trajectory contains 0 Waypoints!", planner_id);
        }
    }
};


class WavefrontExpansionNode {
    public:
    int id;
    
    WavefrontExpansionNode(int id) : id(id){}

    TrajectoryData findPath(){}

    private:
};


class Spawner {
public:
    int path_finder_count;
    std::vector<PixelPos> starting_positions;
    std::vector<PixelPos> goal_positions;
    cv_bridge::CvImagePtr cv_bridge;

    std::vector<WavefrontExpansionNode> path_finders;
    ros::NodeHandle node_handle;

    Spawner() : path_finder_count(4) {
        // for 1.0m
        std::vector<PixelPos> raw_starting_positions =  {PixelPos(28, 28), PixelPos(49, 67), PixelPos(32, 26), PixelPos(38, 68)};
        std::vector<PixelPos> raw_goal_positions =      {PixelPos(98, 53), PixelPos(74, 15), PixelPos(84, 55), PixelPos(78, 18)};

        double grid_size = 0.3; // m
        double robot_size = 1.0; // m
        double factor = robot_size / grid_size;

        for (const auto& pos : raw_starting_positions) {
            starting_positions.push_back(PixelPos(std::round(pos.x * factor), std::round(pos.y * factor)));
        }

        for (const auto& pos : raw_goal_positions) {
            goal_positions.push_back(PixelPos(std::round(pos.x * factor), std::round(pos.y * factor)));
        }

        for (int i = 0; i < path_finder_count; ++i) {
            ROS_INFO("Creating Node %i", i);
            path_finders.push_back(WavefrontExpansionNode(i));
        }

        ROS_INFO("waiting for transformation service...");
        ros::service::waitForService("pixel_to_world");
        ROS_INFO("transformation service is running!");

        ros::Subscriber sub = node_handle.subscribe("/formation_builder/map", 1, &Spawner::MapCallback, this);
    }

    void MapCallback(const sensor_msgs::Image::ConstPtr& msg);


    std::vector<std::vector<int>> ImageToMatrix(const sensor_msgs::Image::ConstPtr& ros_image) {
        std::vector<std::vector<int>> matrix;
        if (!ros_image) {
            ROS_ERROR("Image is invalid!");
            return matrix;
        }
        cv_bridge::CvImagePtr cv_image_ptr;

        cv_image_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::MONO8);

        cv::Mat image = cv_image_ptr->image;

        if (image.empty()) {
            ROS_ERROR("ImageToMatrix(): Image is empty!");
            return matrix;
        }

        matrix.resize(image.rows, std::vector<int>(image.cols));

        for (int i = 0; i < image.rows; ++i) {
            for (int j = 0; j < image.cols; ++j) {
                matrix[i][j] = static_cast<int>(image.at<uchar>(i, j));
            }
        }

        return matrix;
    }
};

#endif /* TRAJECTORY_DATA_H */