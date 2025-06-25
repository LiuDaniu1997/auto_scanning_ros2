#include "deburring_robot_msgs/action/handeye_calibration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include <memory>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std::placeholders;
using HandeyeCalibration = deburring_robot_msgs::action::HandeyeCalibration;

namespace handeye_calibration
{
    class HandeyeEstimationServer : public rclcpp::Node
    {
    public:
        explicit HandeyeEstimationServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("handeye_estimation_server", options)
        {
            RCLCPP_INFO(get_logger(), "Starting the Server");
            action_server_ = rclcpp_action::create_server<HandeyeCalibration>(
            this, "handeye_estimation", std::bind(&HandeyeEstimationServer::goalCallback, this, _1, _2),
            std::bind(&HandeyeEstimationServer::cancelCallback, this, _1),
            std::bind(&HandeyeEstimationServer::acceptedCallback, this, _1));
            
            // load the config file
            config_ = YAML::LoadFile("src/handeye_calibration/config/config.yaml");
            camera_info_ = YAML::LoadFile(config_["camera_info_path"].as<std::string>());

            // create image subscription
            image_topic_ = config_["image_topic"].as<std::string>();
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic_, 10,
                std::bind(&HandeyeEstimationServer::imageCallback, this, _1));

            // load the camera parameters
            auto camera_matrix_data = camera_info_["camera_matrix"]["data"];
            std::vector<double> camera_matrix_vector = camera_matrix_data.as<std::vector<double>>();
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_vector.data()).clone();

            auto dist_coeffs_data = camera_info_["distortion_coefficients"]["data"];
            std::vector<double> dist_coeffs_vector = dist_coeffs_data.as<std::vector<double>>();
            dist_coeffs_ = cv::Mat(1, 5, CV_64F, dist_coeffs_vector.data()).clone();

            aruco_marker_side_length_ = config_["aruco_marker_side_length"].as<float>();

            // initialize transform listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            handeye_base_link_ = config_["handeye_base_link"].as<std::string>();
            handeye_ee_ = config_["handeye_ee"].as<std::string>();
        }
    private:
        rclcpp_action::Server<HandeyeCalibration>::SharedPtr action_server_;

        // initialize moveit group
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;

        // subscribe the image
        std::string image_topic_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        cv::Mat latest_image_;
        std::mutex image_mutex_;
        
        // config node
        YAML::Node config_;
        YAML::Node camera_info_;

        // camera info
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // Aruco marker info
        float aruco_marker_side_length_;

        // transform listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // links for handeye calibration
        std::string handeye_base_link_;
        std::string handeye_ee_;

        // yaml node that saves the handeye calibration result;
        YAML::Node sample_;

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const HandeyeCalibration::Goal> goal)
        {
            RCLCPP_INFO(
                get_logger(),
                "Received goal with target pose:\n"
                "Position: [x: %.3f, y: %.3f, z: %.3f]\n"
                "Orientation: [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
                goal->target_pose.position.x,
                goal->target_pose.position.y,
                goal->target_pose.position.z,
                goal->target_pose.orientation.x,
                goal->target_pose.orientation.y,
                goal->target_pose.orientation.z,
                goal->target_pose.orientation.w
            );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<HandeyeCalibration>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            if(arm_move_group_)
            {
                arm_move_group_->stop();
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<HandeyeCalibration>> goal_handle)
        {
          // this needs to return quickly to avoid blocking the executor, so spin up a new thread
          std::thread{ std::bind(&HandeyeEstimationServer::execute, this, _1), goal_handle }.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<HandeyeCalibration>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing Tasks...");
            auto feedback = std::make_shared<HandeyeCalibration::Feedback>();
            auto & current_phase = feedback->current_phase;
            current_phase = 0;
            auto result = std::make_shared<HandeyeCalibration::Result>();
            YAML::Node temp_sample;

            /** 
                1: Move the arm to the specified position
            **/ 
            
            // MoveIt 2 Interface
            if(!arm_move_group_)
            {
                arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            }
            
            // Set the target pose
            arm_move_group_->setPoseTarget(goal_handle->get_goal()->target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if(arm_plan_success)
            {
                RCLCPP_INFO(get_logger(), "Moving the arm...");
                arm_move_group_->move();
                current_phase++;
                goal_handle->publish_feedback(feedback);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Planner failed!");
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            
            /** 
                2: Locate the marker
            */ 
            // get current image
            rclcpp::Rate rate(10);
            cv::Mat current_image;
            for (int i = 0; i < 50; ++i) 
            {
                {
                    std::lock_guard<std::mutex> lock(image_mutex_);
                    if (!latest_image_.empty()) 
                    {
                        current_image = latest_image_.clone();
                        break;
                    }
                }
                rate.sleep();
            }

            // if don't receive any image
            if (current_image.empty()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive image.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            
            // save the image with current time
            std::string image_data_folder = config_["image_data_folder"].as<std::string>();
            auto now = std::chrono::system_clock::now();
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream timestamp_ss;
            timestamp_ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
            std::string img_save_path = image_data_folder + "image_" + timestamp_ss.str() + ".jpg";
            cv::imwrite(img_save_path, current_image);
            RCLCPP_INFO(this->get_logger(), "Received the image, image saved to: %s", img_save_path.c_str());

            // locate the marker
            RCLCPP_INFO(get_logger(), "Locating the marker...");
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(current_image, dictionary, corners, ids);
            if (!ids.empty() && ids.size()==1) 
            {
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, aruco_marker_side_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
                
                for (size_t i = 0; i < ids.size(); ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "Marker ID: %d", ids[i]);

                    RCLCPP_INFO(this->get_logger(), "Translation (m): [%.3f, %.3f, %.3f]",
                                tvecs[i][0], tvecs[i][1], tvecs[i][2]);

                    RCLCPP_INFO(this->get_logger(), "Rotation (rad): [%.3f, %.3f, %.3f]",
                                rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                    
                                                // calculate the pose of the marker
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);
                    
                    cv::Mat T_marker_to_camera = cv::Mat::eye(4, 4, CV_64F);
                    rotation_matrix.copyTo(T_marker_to_camera(cv::Rect(0, 0, 3, 3)));
                    T_marker_to_camera.at<double>(0, 3) = tvecs[0][0];
                    T_marker_to_camera.at<double>(1, 3) = tvecs[0][1];
                    T_marker_to_camera.at<double>(2, 3) = tvecs[0][2];

                    temp_sample["T_marker_to_camera"] =  matrixToYamlNode(T_marker_to_camera);
                }
            } 
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to locate the marker.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            current_phase++;
            goal_handle->publish_feedback(feedback);

            /**
                3: Recode the TF
            */
            RCLCPP_INFO(get_logger(), "Recording TF between base link and end effector");
            geometry_msgs::msg::TransformStamped tf_ee_to_base;
            try {
                tf_ee_to_base = tf_buffer_->lookupTransform(handeye_base_link_, handeye_ee_, tf2::TimePointZero);
                tf2::Transform tf2_transform;
                tf2::fromMsg(tf_ee_to_base.transform, tf2_transform);
                cv::Mat T_ee_to_base = cv::Mat::eye(4, 4, CV_64F);
                for (int i = 0; i < 3; ++i) 
                {
                    for (int j = 0; j < 3; ++j) 
                    {
                        T_ee_to_base.at<double>(i, j) = tf2_transform.getBasis()[i][j];
                    }
                    T_ee_to_base.at<double>(i, 3) = tf2_transform.getOrigin()[i];
                }
                temp_sample["T_ee_to_base"] =  matrixToYamlNode(T_ee_to_base);
                sample_["handeye_samples"].push_back(temp_sample);

                // write the result to yaml file
                std::string save_path = config_["handeye_samples_file_path"].as<std::string>();
                std::ofstream fout(save_path);
                if (fout.is_open())
                {
                    fout << sample_;
                    fout.close();
                    RCLCPP_INFO(this->get_logger(), "Saved handeye samples to: %s", save_path.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to write handeye samples to file.");
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }
            } 
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            current_phase++;
            goal_handle->publish_feedback(feedback);

            if(rclcpp::ok())
            {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
        }

        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
        {
            try 
            {
                auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                std::lock_guard<std::mutex> lock(image_mutex_);
                latest_image_ = cv_ptr->image.clone();
            } 
            catch (const cv_bridge::Exception &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        YAML::Node matrixToYamlNode(const cv::Mat &mat) 
        {
            YAML::Node node;
            std::ostringstream stream;

            stream << std::fixed << std::setprecision(4);
            stream << "[";
            int count = 0;

            for (int i = 0; i < mat.rows; ++i) 
            {
                for (int j = 0; j < mat.cols; ++j) 
                {
                    stream << mat.at<double>(i, j);
                    ++count;
                    if (!(i == mat.rows - 1 && j == mat.cols - 1)) 
                    {
                        stream << ", ";
                        if (count % 4 == 0)
                            stream << "\n";
                    }
                }
            }

            stream << "]";
            node["data"] = YAML::Load(stream.str());
            return node;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(handeye_calibration::HandeyeEstimationServer)