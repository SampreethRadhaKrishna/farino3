#include <rclcpp/rclcpp.hpp>
#include "fairino_msgs/srv/cartesian_move.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


using namespace std::placeholders;

class MoveFr3Server : public rclcpp::Node
{
public:
    MoveFr3Server() : Node("move_fr3_server")
    {
        service_ = create_service<fairino_msgs::srv::CartesianMove>("move_fr3_server", std::bind(&MoveFr3Server::serviceCallback, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to send goal");
    }


private:
    rclcpp::Service<fairino_msgs::srv::CartesianMove>::SharedPtr service_;

    void serviceCallback(const std::shared_ptr<fairino_msgs::srv::CartesianMove::Request> req,
                         const std::shared_ptr<fairino_msgs::srv::CartesianMove::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Request Received x: " << req->x << " y: " << req->y << " z: " << req->z);
        float xx = req->x;
        float yy = req->y;
        float zz = req->z;
        bool success = move_robot(xx, yy, zz, this->shared_from_this());
        res->success = success;
        
    }

    bool move_robot(float &dx, float &dy, float &dz,  const std::shared_ptr<rclcpp::Node> node)
    {
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");

        std::string end_effector_link = "wrist3_link";

        double x = dx;
        double y = dy;
        double z = dz;
        
        bool arm_within_bounds = arm_move_group.setPositionTarget(x,y,z);
        // bool arm_within_bounds = arm_move_group.setPoseTarget(target_pose1);
        if (!arm_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
            // return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(arm_plan_success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Planner SUCCEED, moving the arm");
            arm_move_group.move();
            return true;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Planners failed!");
            return false;
        }
    }

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveFr3Server>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}