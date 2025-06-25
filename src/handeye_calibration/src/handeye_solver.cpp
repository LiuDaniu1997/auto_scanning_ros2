#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <fstream>
#include <iostream>
#include <vector>

class HandeyeSolverNode : public rclcpp::Node
{
public:
    HandeyeSolverNode() : Node("handeye_solver_node")
    {
        // load the config file
        config_ = YAML::LoadFile("src/handeye_calibration/config/config.yaml");
        
        std::string handeye_samples_file_path = config_["handeye_samples_file_path"].as<std::string>();
        YAML::Node data = YAML::LoadFile(handeye_samples_file_path);

        std::vector<cv::Mat> T_ee_to_base_mats, T_marker_to_camera_mats;

        for (const auto &sample : data["handeye_samples"]) 
        {
            std::vector<double> T_ee_to_base_data = sample["T_ee_to_base"]["data"].as<std::vector<double>>();
            std::vector<double> T_marker_to_camera_data = sample["T_marker_to_camera"]["data"].as<std::vector<double>>();

            cv::Mat T_ee_to_base_sample(4, 4, CV_64F, T_ee_to_base_data.data());
            cv::Mat T_marker_to_camera_sample(4, 4, CV_64F, T_marker_to_camera_data.data());

            T_ee_to_base_mats.push_back(T_ee_to_base_sample.clone());
            T_marker_to_camera_mats.push_back(T_marker_to_camera_sample.clone());
        }

        if (T_ee_to_base_mats.size() < 3) 
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough samples to perform calibration.");
            return;
        }

        std::vector<cv::Mat> R_ee_to_base, t_ee_to_base;
        std::vector<cv::Mat> R_marker_to_camera, t_marker_to_camera;

        for (size_t i = 0; i < T_ee_to_base_mats.size(); ++i) 
        {
            cv::Mat T_ee_to_base = T_ee_to_base_mats[i];
            cv::Mat T_marker_to_camera = T_marker_to_camera_mats[i];

            R_ee_to_base.push_back(T_ee_to_base(cv::Range(0, 3), cv::Range(0, 3)).clone());
            t_ee_to_base.push_back(T_ee_to_base(cv::Range(0, 3), cv::Range(3, 4)).clone());

            R_marker_to_camera.push_back(T_marker_to_camera(cv::Range(0, 3), cv::Range(0, 3)).clone());
            t_marker_to_camera.push_back(T_marker_to_camera(cv::Range(0, 3), cv::Range(3, 4)).clone());
        }

        cv::Mat R_cam_to_ee, t_cam_to_ee;
        cv::calibrateHandEye(
            R_ee_to_base, t_ee_to_base,
            R_marker_to_camera, t_marker_to_camera,
            R_cam_to_ee, t_cam_to_ee,
            cv::CALIB_HAND_EYE_TSAI
        );

        // Output result
        RCLCPP_INFO(this->get_logger(), "Hand-eye calibration result:");
        for (int i = 0; i < 3; ++i) 
        {
            std::stringstream row;
            for (int j = 0; j < 3; ++j) row << R_cam_to_ee.at<double>(i, j) << " ";
            row << t_cam_to_ee.at<double>(i, 0);
            RCLCPP_INFO(this->get_logger(), "%s", row.str().c_str());
        }

        std::string handeye_result_path = config_["handeye_result_path"].as<std::string>();

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "T_cam_to_ee";
        out << YAML::BeginMap;
        out << YAML::Key << "data";
        out << YAML::Flow << YAML::BeginSeq;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
                out << R_cam_to_ee.at<double>(i, j);
            out << t_cam_to_ee.at<double>(i, 0);
        }
        out << 0.0 << 0.0 << 0.0 << 1.0;

        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;

        std::ofstream fout(handeye_result_path);
        if (fout.is_open()) 
        {
            fout << out.c_str();
            fout.close();
            RCLCPP_INFO(this->get_logger(), "Hand-eye result saved to: %s", handeye_result_path.c_str());
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", handeye_result_path.c_str());
        }

        T_cam_to_ee_ = cv::Mat::eye(4, 4, CV_64F);
        R_cam_to_ee.copyTo(T_cam_to_ee_(cv::Rect(0, 0, 3, 3)));
        t_cam_to_ee.copyTo(T_cam_to_ee_(cv::Rect(3, 0, 1, 3)));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        handeye_base_link_ = config_["handeye_base_link"].as<std::string>();
        handeye_cam_link_ = config_["handeye_cam_link"].as<std::string>();
        handeye_ee_ = config_["handeye_ee"].as<std::string>();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&HandeyeSolverNode::publishTransform, this));
    }
private:
    // config node
    YAML::Node config_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Mat T_cam_to_ee_;

    // transform listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string handeye_base_link_;
    std::string handeye_cam_link_;
    std::string handeye_ee_;

    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        
        tf_msg.header.frame_id = handeye_base_link_;           
        tf_msg.child_frame_id = handeye_cam_link_;

        geometry_msgs::msg::TransformStamped tf_ee_to_base;
        cv::Mat T_ee_to_base = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat T_cam_to_base = cv::Mat::eye(4, 4, CV_64F);

        try {
            tf_ee_to_base = tf_buffer_->lookupTransform(handeye_base_link_, handeye_ee_, tf2::TimePointZero);
            tf2::Transform tf2_transform;
            tf2::fromMsg(tf_ee_to_base.transform, tf2_transform);
            for (int i = 0; i < 3; ++i) 
            {
                for (int j = 0; j < 3; ++j) 
                {
                    T_ee_to_base.at<double>(i, j) = tf2_transform.getBasis()[i][j];
                }
                T_ee_to_base.at<double>(i, 3) = tf2_transform.getOrigin()[i];
            }

            T_cam_to_base = T_ee_to_base * T_cam_to_ee_;
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
        }

        tf_msg.transform.translation.x = T_cam_to_base.at<double>(0, 3);
        tf_msg.transform.translation.y = T_cam_to_base.at<double>(1, 3);
        tf_msg.transform.translation.z = T_cam_to_base.at<double>(2, 3);

        tf2::Matrix3x3 rot(
            T_cam_to_base.at<double>(0, 0), T_cam_to_base.at<double>(0, 1), T_cam_to_base.at<double>(0, 2),
            T_cam_to_base.at<double>(1, 0), T_cam_to_base.at<double>(1, 1), T_cam_to_base.at<double>(1, 2),
            T_cam_to_base.at<double>(2, 0), T_cam_to_base.at<double>(2, 1), T_cam_to_base.at<double>(2, 2));

        tf2::Quaternion q;
        rot.getRotation(q);

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandeyeSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}