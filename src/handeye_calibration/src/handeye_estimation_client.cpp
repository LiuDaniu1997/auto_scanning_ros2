#include "deburring_robot_msgs/action/handeye_calibration.hpp"

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
using HandeyeCalibration = deburring_robot_msgs::action::HandeyeCalibration;
namespace fs = std::filesystem;

namespace handeye_calibration
{
    class HandeyeEstimationClient : public rclcpp::Node
    {
    public:
        explicit HandeyeEstimationClient(const rclcpp::NodeOptions & options)
        : Node("handeye_estimation_client", options)
        {
            this->client_ptr_ = rclcpp_action::create_client<HandeyeCalibration>(
                this,
                "handeye_estimation");
            
            // load the config file
            config_ = YAML::LoadFile("src/handeye_calibration/config/config.yaml");
            marker_pos_x_ = config_["marker_pos_x"].as<double>();
            marker_pos_y_ = config_["marker_pos_y"].as<double>();
            marker_pos_z_ = config_["marker_pos_z"].as<double>();
            
            // clear the previous data
            std::string image_data_folder = config_["image_data_folder"].as<std::string>();
            std::string handeye_samples_file_path = config_["handeye_samples_file_path"].as<std::string>();
            cleanOldData(image_data_folder, handeye_samples_file_path);

            initializeTargetPoses();

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&HandeyeEstimationClient::send_goal, this));
        }

    private:
        rclcpp_action::Client<HandeyeCalibration>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::Pose> target_poses_;
        size_t current_index_ = 0;

        // config node
        YAML::Node config_;

        double marker_pos_x_;
        double marker_pos_y_;
        double marker_pos_z_;

        void cleanOldData(const std::string &img_dir_str, const std::string &yaml_file_str)
        {
            try 
            {
                fs::path img_dir(img_dir_str);
                fs::path yaml_file(yaml_file_str);

                // clear all img file
                if (fs::exists(img_dir) && fs::is_directory(img_dir)) 
                {
                    for (const auto& entry : fs::directory_iterator(img_dir)) 
                    {
                        if (fs::is_regular_file(entry)) 
                        {
                            fs::remove(entry);
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Cleared all previous img files under: %s", img_dir.c_str());
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Image directory not found: %s", img_dir.c_str());
                }
                
                // clear yaml file
                std::ofstream yaml_clear(yaml_file, std::ios::trunc);
                if (yaml_clear.is_open()) 
                {
                    yaml_clear.close();
                    RCLCPP_INFO(this->get_logger(), "Cleared YAML file: %s", yaml_file.c_str());
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Could not open YAML file to clear: %s", yaml_file.c_str());
                }

            } 
            catch (const std::exception &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error during file cleanup: %s", e.what());
            }
        }

        double deg2rad(double degrees)
        {
            return degrees * M_PI / 180.0;
        }

        void generatePoseOnCircle(double radius, double phi, double theta)
        {
            double x = marker_pos_x_ + radius * std::sin(phi) * std::cos(theta);
            double y = marker_pos_y_ + radius * std::sin(phi) * std::sin(theta);
            double z = marker_pos_z_ + radius * std::cos(phi);

            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;

            tf2::Vector3 z_axis(marker_pos_x_ - x,
                                marker_pos_y_ - y,
                                marker_pos_z_ - z);
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

            tf2::Quaternion q_correction;
            q_correction.setRPY(0, 0, -M_PI/2);
            q = q * q_correction;
            q.normalize();

            pose.orientation = tf2::toMsg(q);

            target_poses_.push_back(pose);
        }

        void initializeTargetPoses()
        {
            generatePoseOnCircle(0.5, deg2rad(10), deg2rad(60));
            generatePoseOnCircle(0.5, deg2rad(20), deg2rad(75));
            generatePoseOnCircle(0.5, deg2rad(30), deg2rad(90));
            
            generatePoseOnCircle(0.5, deg2rad(45), deg2rad(100));
            generatePoseOnCircle(0.5, deg2rad(30), deg2rad(120));
            generatePoseOnCircle(0.5, deg2rad(20), deg2rad(140));
            generatePoseOnCircle(0.5, deg2rad(10), deg2rad(160));
            
            generatePoseOnCircle(0.5, deg2rad(0), deg2rad(0));
            
            generatePoseOnCircle(0.5, deg2rad(10), deg2rad(-160));
            generatePoseOnCircle(0.5, deg2rad(20), deg2rad(-140));
            generatePoseOnCircle(0.5, deg2rad(30), deg2rad(-120));
            generatePoseOnCircle(0.5, deg2rad(45), deg2rad(-100));
            
            generatePoseOnCircle(0.5, deg2rad(30), deg2rad(-90));
            generatePoseOnCircle(0.5, deg2rad(20), deg2rad(-75));
            generatePoseOnCircle(0.5, deg2rad(10), deg2rad(-60));
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

            auto goal_msg = HandeyeCalibration::Goal();
            goal_msg.target_pose = target_poses_[current_index_];

            RCLCPP_INFO(this->get_logger(), "Sending goal %zu", current_index_);

            auto send_goal_options = rclcpp_action::Client<HandeyeCalibration>::SendGoalOptions();

            send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<HandeyeCalibration>::SharedPtr & goal_handle)
            {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };

            send_goal_options.feedback_callback = [this](
                rclcpp_action::ClientGoalHandle<HandeyeCalibration>::SharedPtr,
                const std::shared_ptr<const HandeyeCalibration::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Current task phase: %d", feedback->current_phase);
            };

            send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<HandeyeCalibration>::WrappedResult & result)
            {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal %zu succeeded.", current_index_);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }

                current_index_++;
                // Re-enable timer to trigger next goal
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1000),
                    std::bind(&HandeyeEstimationClient::send_goal, this));
            };

            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
    };

}  // namespace handeye_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(handeye_calibration::HandeyeEstimationClient)
