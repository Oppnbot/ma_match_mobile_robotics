#include "mapf.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_world");
  ros::NodeHandle n;

  ROS_INFO("Hello World!!!!");

  ros::spin();
  return 0;
}


void Spawner::MapCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::vector<std::vector<int>> grid = Spawner::ImageToMatrix(msg);

    std::vector<TrajectoryData> trajectories;

    for(int i = 0; i < this->path_finders.size(); i++){
        if(i >= this->goal_positions.size()){
            break;
        }

        std::vector<Waypoint> starting_waypoints;
        Waypoint starting_waypoint = Waypoint(this->starting_positions[i], 0.0f);
        starting_waypoints.push_back(starting_waypoint);

        int kernel_size = 3;
        int half_kernel_size = (kernel_size - 1) / 2;
        for(int x = -half_kernel_size; x < half_kernel_size; x++){
            for(int y = -half_kernel_size; y < half_kernel_size; y++){
                if(x==0 && y==0){
                    continue;
                }
                PixelPos position = PixelPos(this->starting_positions[i].x + x , this->starting_positions[i].y + y);
                starting_waypoints.push_back(Waypoint(position, 0.0f, &starting_waypoint));
            }
        }
        trajectories.push_back(TrajectoryData(i, starting_waypoints));

        int index = 0;
        for (const auto& pf : this->path_finders){
            if(index >= this->starting_positions.size() || index >= this->goal_positions.size()){
                ROS_WARN("Not enough starting/goal positions for the chosen ammount of planners");
                break;
            }
            trajectories.erase(trajectories.begin());
            TrajectoryData trajetory = pf-
            index++;
        }



    }


}