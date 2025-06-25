#include "deburring_robot_msgs/action/auto_scanning.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace std::placeholders;
using AutoScanning = deburring_robot_msgs::action::AutoScanning;
namespace fs = std::filesystem;

namespace auto_scanning
{
    class AutoScanningClient : public rclcpp::Node
    {
    public:
        explicit AutoScanningClient(const rclcpp::NodeOptions & options)
        : Node("handeye_estimation_client", options)
        {
            this->client_ptr_ = rclcpp_action::create_client<AutoScanning>(
                this,
                "handeye_estimation");
            
            // load the config file
            config_ = YAML::LoadFile("src/auto_scanning/config/config.yaml");
            point_cloud_save_path_ = config_["point_cloud_save_path"].as<std::string>();
            cleanOldData(point_cloud_save_path_);

            initializeTargetPoses();

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&AutoScanningClient::send_goal, this));
        }

    private:
        rclcpp_action::Client<AutoScanning>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::Pose> target_poses_;
        size_t current_index_ = 0;

        // config node
        YAML::Node config_;
        std::string point_cloud_save_path_;

        double obj_pos_x_ = 0.5;
        double obj_pos_y_ = 0.0;
        double obj_pos_z_ = 0.0;

        double deg2rad(double degrees)
        {
            return degrees * M_PI / 180.0;
        }

        void cleanOldData(const std::string &pcd_dir_str)
        {
            try 
            {
                fs::path pcd_dir(pcd_dir_str);

                // clear all img file
                if (fs::exists(pcd_dir) && fs::is_directory(pcd_dir)) 
                {
                    for (const auto& entry : fs::directory_iterator(pcd_dir)) 
                    {
                        if (fs::is_regular_file(entry)) 
                        {
                            fs::remove(entry);
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Cleared all previous point cloud files under: %s", pcd_dir.c_str());
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Point cloud directory not found: %s", pcd_dir.c_str());
                }
            } 
            catch (const std::exception &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error during file cleanup: %s", e.what());
            }
        }

        void generatePoseOnCircle(double radius, double phi, double theta)
        {
            double x = obj_pos_x_ + radius * std::sin(phi) * std::cos(theta);
            double y = obj_pos_y_ + radius * std::sin(phi) * std::sin(theta);
            double z = obj_pos_z_ + radius * std::cos(phi);

            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;

            tf2::Vector3 z_axis(obj_pos_x_ - x,
                                obj_pos_y_ - y,
                                obj_pos_z_ - z);
            z_axis.normalize();

            tf2::Vector3 ref_y = (theta > 0.0) ?
                tf2::Vector3(1, 0, 0) :
                tf2::Vector3(-1, 0, 0);

            tf2::Vector3 y_axis = ref_y - z_axis * (ref_y.dot(z_axis));

            if (y_axis.length() < 1e-6)
            {
                ref_y = tf2::Vector3(0, 1, 0);
                y_axis = ref_y - z_axis * (ref_y.dot(z_axis));
            }

            y_axis.normalize();

            tf2::Vector3 x_axis = y_axis.cross(z_axis);
            x_axis.normalize();

            tf2::Matrix3x3 rot_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z());

            tf2::Quaternion q;
            rot_matrix.getRotation(q);
            
            // if(theta!=0)
            // {
            //     tf2::Quaternion q_correction;
            //     q_correction.setRPY(0, 0, -M_PI/2);
            //     q = q * q_correction;
            // }

            // if(theta > 0)
            // {
            //     tf2::Quaternion q_correction;
            //     q_correction.setRPY(0, 0, -M_PI/2);
            //     q = q * q_correction;
            // }
            // else if(theta < 0)
            // {
            //     tf2::Quaternion q_correction;
            //     q_correction.setRPY(0, 0, M_PI/2);
            //     q = q * q_correction;
            // }
            
            if(theta > 0)
            {
                tf2::Quaternion q_correction;
                q_correction.setRPY(0, 0, -M_PI);
                q = q * q_correction;
            }

            q.normalize();

            pose.orientation = tf2::toMsg(q);

            target_poses_.push_back(pose);
        }

        void initializeTargetPoses()
        {
            generatePoseOnCircle(0.5, deg2rad(60), deg2rad(90));
            generatePoseOnCircle(0.5, deg2rad(0), deg2rad(0));
            generatePoseOnCircle(0.5, deg2rad(60), deg2rad(-90));
        }

        void send_goal()
        {
            timer_->cancel();

            if (current_index_ >= target_poses_.size()) {
                RCLCPP_INFO(this->get_logger(), "All goals have been processed. Shutting down.");
                rclcpp::shutdown();
                return;
            }

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available.");
                rclcpp::shutdown();
                return;
            }

            geometry_msgs::msg::Pose current_pose = target_poses_[current_index_];
            RCLCPP_INFO(this->get_logger(), 
                "Current Target Position: [x=%.3f, y=%.3f, z=%.3f], Orientation (quaternion): [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            );

            auto goal_msg = AutoScanning::Goal();
            goal_msg.target_pose = target_poses_[current_index_];

            RCLCPP_INFO(this->get_logger(), "Sending goal %zu", current_index_);

            auto send_goal_options = rclcpp_action::Client<AutoScanning>::SendGoalOptions();

            send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<AutoScanning>::SharedPtr & goal_handle)
            {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };

            send_goal_options.feedback_callback = [this](
                rclcpp_action::ClientGoalHandle<AutoScanning>::SharedPtr,
                const std::shared_ptr<const AutoScanning::Feedback> feedback)
            {
                switch (feedback->current_phase)
                {
                    case 0:
                        RCLCPP_INFO(this->get_logger(), "Initialize client.");
                        break;
                    case 1:
                        RCLCPP_INFO(this->get_logger(), "Successfully moved the arm to the scan position.");
                        break;
                    case 2:
                        RCLCPP_INFO(this->get_logger(), "Successfully saved the pointcloud.");
                        break;
                    case 3:
                        RCLCPP_INFO(this->get_logger(), "Successfully moved the arm to the ready position.");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown phase");
                        break;
                }
            };

            send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<AutoScanning>::WrappedResult & result)
            {
                switch (result.code) 
                {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Position %zu succeeded.", current_index_);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Position %zu was aborted", current_index_);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Position %zu was canceled", current_index_);
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }

                current_index_++;
                // Re-enable timer to trigger next goal
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1000),
                    std::bind(&AutoScanningClient::send_goal, this));
            };
            
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
    };

}  // namespace auto_scanning

RCLCPP_COMPONENTS_REGISTER_NODE(auto_scanning::AutoScanningClient)
