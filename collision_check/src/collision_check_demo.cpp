#include <ros/ros.h>
#include <collision_check_srv.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check_demo");
    ros::NodeHandle nh;
    
    // Create collision check client
    ros::ServiceClient collision_client = 
        nh.serviceClient<collision_check::collision_check_srv>("collision_checker/check");
        
    // Create publisher for visualization
    ros::Publisher display_pub = 
        nh.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1);

    // Set up collision check request
    collision_check::collision_check_srv srv;
    srv.request.env_stl_path = "path/to/environment.stl";
    srv.request.ob_names = {"object1"};
    srv.request.ob_stl_paths = {"path/to/object1.stl"};
    srv.request.min_limits = {-1.0, -1.0, 0.0};
    srv.request.max_limits = {1.0, 1.0, 1.0};
    srv.request.limit_collision = 1;

    // Call collision checker
    if (collision_client.call(srv)) {
        ROS_INFO("Collision check successful");
        // Process results
        for (size_t i = 0; i < srv.response.x.size(); i++) {
            ROS_INFO("Object %zu position: x=%f, y=%f, z=%f", 
                    i, srv.response.x[i], srv.response.y[i], srv.response.z[i]);
        }
    }

    ros::spin();
    return 0;
}