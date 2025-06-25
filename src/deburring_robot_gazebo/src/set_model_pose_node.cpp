#include <rclcpp/rclcpp.hpp>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>


class SetModelPoseNode : public rclcpp::Node
{
public:
    SetModelPoseNode()
    : Node("set_model_pose_node")
    {
        gz::msgs::Pose pose_msg = CreatePoseMessage();

        gz::msgs::Boolean reply;
        bool result;

        if (!gz_node.Request("/world/empty/set_pose", pose_msg, 1000, reply, result))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send request to Gazebo.");
        }
        else if (!result || !reply.data())
        {
            RCLCPP_ERROR(this->get_logger(), "Request sent, but Gazebo responded with failure.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Model pose successfully updated.");
        }
    }

private:
    gz::transport::Node gz_node;

    gz::msgs::Pose CreatePoseMessage()
    {
        gz::msgs::Pose pose_msg;
        pose_msg.set_name("teapot_1");

        gz::msgs::Vector3d *position = pose_msg.mutable_position();
        position->set_x(0.5);
        position->set_y(0.0);
        position->set_z(0.0);

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = -1.5708; 

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        gz::msgs::Quaternion *orientation = pose_msg.mutable_orientation();
        orientation->set_x(quat.x());
        orientation->set_y(quat.y());
        orientation->set_z(quat.z());
        orientation->set_w(quat.w());

        return pose_msg;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<SetModelPoseNode>());
    rclcpp::shutdown();
    return 0;
}