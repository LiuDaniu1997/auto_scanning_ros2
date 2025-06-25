#include "deburring_robot_msgs/action/auto_scanning.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <mutex>

#include <memory>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std::placeholders;
using AutoScanning = deburring_robot_msgs::action::AutoScanning;
typedef pcl::PointXYZRGB PointT;

namespace auto_scanning
{
    class AutoScanningServer : public rclcpp::Node
    {
    public:
        explicit AutoScanningServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("handeye_estimation_server", options)
        {
            RCLCPP_INFO(get_logger(), "Starting the Server");
            action_server_ = rclcpp_action::create_server<AutoScanning>(
            this, "handeye_estimation", std::bind(&AutoScanningServer::goalCallback, this, _1, _2),
            std::bind(&AutoScanningServer::cancelCallback, this, _1),
            std::bind(&AutoScanningServer::acceptedCallback, this, _1));

            // load the config file
            config_ = YAML::LoadFile("src/auto_scanning/config/config.yaml");
            point_cloud_topic_ = config_["point_cloud_topic"].as<std::string>();
            point_cloud_save_path_ = config_["point_cloud_save_path"].as<std::string>();

            // initialize transform listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            tf_base_link_ = config_["tf_base_link"].as<std::string>();
            tf_ee_ = config_["tf_ee"].as<std::string>();
            handeye_result_file_path_ = config_["handeye_result_file_path"].as<std::string>();

            handeye_result_ = YAML::LoadFile(handeye_result_file_path_);
            T_cam_to_ee_vec_ = handeye_result_["T_cam_to_ee"]["data"].as<std::vector<double>>();
            Eigen::Matrix4d T;
            for (int i = 0; i < 16; ++i) {
                T(i / 4, i % 4) = T_cam_to_ee_vec_[i];
            }
            T_cam_to_ee_ = Eigen::Affine3d(T);
        }
    private:
        rclcpp_action::Server<AutoScanning>::SharedPtr action_server_;

        // initialize moveit group
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
        const double eef_step_ = 0.01;
        moveit_msgs::msg::RobotTrajectory trajectory_;
        std::vector<geometry_msgs::msg::Pose> waypoints_;

        // config node
        YAML::Node config_;
        std::string point_cloud_save_path_;

        // point cloud
        std::string point_cloud_topic_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
        std::mutex pointcloud_mutex_;

        // transform listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // links for tf
        std::string tf_base_link_;
        std::string tf_ee_;

        // auto scanning result file path
        std::string handeye_result_file_path_;
        YAML::Node handeye_result_;
        std::vector<double> T_cam_to_ee_vec_;
        Eigen::Affine3d T_cam_to_ee_;

        bool planAndExecuteCartesianPath(const geometry_msgs::msg::Pose &target_pose,
                          moveit::planning_interface::MoveGroupInterface &move_group,
                          rclcpp::Logger logger,
                          double eef_step,
                          std::vector<geometry_msgs::msg::Pose> &waypoints,
                          moveit_msgs::msg::RobotTrajectory &trajectory)
        {
            waypoints.clear();
            waypoints.push_back(target_pose);

            double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
            RCLCPP_INFO(logger, "Cartesian path planned (%.2f%% achieved)", fraction * 100.0);

            if (fraction >= 0.95)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory = trajectory;
                move_group.execute(plan);
                return true;
            }
            else
            {
                RCLCPP_ERROR(logger, "Cartesian path planning failed!");
                return false;
            }
        }

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const AutoScanning::Goal> goal)
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
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<AutoScanning>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<AutoScanning>> goal_handle)
        {
          // this needs to return quickly to avoid blocking the executor, so spin up a new thread
          std::thread{ std::bind(&AutoScanningServer::execute, this, _1), goal_handle }.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<AutoScanning>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing Tasks...");
            auto feedback = std::make_shared<AutoScanning::Feedback>();
            auto & current_phase = feedback->current_phase;
            current_phase = 0;
            auto result = std::make_shared<AutoScanning::Result>();

            /** 
                1: Move the arm to the specified position
            **/ 
            
            // MoveIt 2 Interface
            if(!arm_move_group_)
            {
                arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
            }
            
            // Set the target pose   
            if (!planAndExecuteCartesianPath(goal_handle->get_goal()->target_pose, *arm_move_group_, this->get_logger(), eef_step_, waypoints_, trajectory_))
            {
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(get_logger(), "Successfully moved the arm to scan position.");
            current_phase++;
            goal_handle->publish_feedback(feedback);

            // record the current pointcloud
            RCLCPP_INFO(get_logger(), "Waiting for a point cloud...");
            
            latest_pointcloud_.reset();

            // temp subscription
            pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                point_cloud_topic_, 10,
                std::bind(&AutoScanningServer::pointcloudCallback, this, std::placeholders::_1));

            rclcpp::Time start_time = this->now();
            rclcpp::Rate rate(10);
            bool received = false;

            while (rclcpp::ok() && !received && (this->now() - start_time).seconds() < 2.0)
            {
                {
                    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
                    received = (latest_pointcloud_ != nullptr);
                }
                rate.sleep();
            }

            if (!received)
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout: Did not receive point cloud.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            // save the pointcloud to pcd file
            std::lock_guard<std::mutex> lock(pointcloud_mutex_);
            pcl::PointCloud<PointT> pcl_cloud_camera;
            pcl::PointCloud<PointT> pcl_cloud_world;
            pcl_cloud_world.points.reserve(pcl_cloud_camera.points.size());
            
            pcl::fromROSMsg(*latest_pointcloud_, pcl_cloud_camera);

            // record curretn tf between base_link and end effector
            RCLCPP_INFO(get_logger(), "Recording TF between base link and end effector");
            geometry_msgs::msg::TransformStamped tf_ee_to_base;

            try {
                tf_ee_to_base = tf_buffer_->lookupTransform(tf_base_link_, tf_ee_, tf2::TimePointZero);
                Eigen::Affine3d T_ee_to_base = tf2::transformToEigen(tf_ee_to_base);
                Eigen::Affine3d T_camera_to_base = T_ee_to_base * T_cam_to_ee_; // base_T_ee * ee_T_cam

                Eigen::Affine3d T_cam_base_to_cam = Eigen::Affine3d::Identity();
                T_cam_base_to_cam.linear() =
                    (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX())).toRotationMatrix();
                T_cam_base_to_cam = T_cam_base_to_cam.inverse();

                for (const auto& pt : pcl_cloud_camera.points)
                {
                    Eigen::Vector3d pt_cam_base(pt.x, pt.y, pt.z); 
                    Eigen::Vector3d pt_world = T_camera_to_base * T_cam_base_to_cam * pt_cam_base; // base_T_camera * camera_T_camera_base * camera_base_p

                    PointT pt_out;
                    pt_out.x = pt_world.x();
                    pt_out.y = pt_world.y();
                    pt_out.z = pt_world.z();
                    pcl_cloud_world.points.push_back(pt_out);
                }

                pcl_cloud_world.width = pcl_cloud_world.points.size();
                pcl_cloud_world.height = 1;
                pcl_cloud_world.is_dense = true;
            } 
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            std::stringstream filename;
            filename << point_cloud_save_path_ << "cloud_world_" << std::fixed << std::setprecision(3)
                    << this->now().seconds() << ".pcd";
            RCLCPP_INFO(this->get_logger(), "Save point cloud to %s", filename.str().c_str());

            if (pcl::io::savePCDFileBinary(filename.str(), pcl_cloud_world) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Saved point cloud successfully");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            current_phase++;
            goal_handle->publish_feedback(feedback);

            geometry_msgs::msg::Pose ready_pose;
            ready_pose.position.x = 0.5;
            ready_pose.position.y = 0.0;
            ready_pose.position.z = 0.5;
            ready_pose.orientation.x = 0.707;
            ready_pose.orientation.y = -0.707;
            ready_pose.orientation.z = 0.0;
            ready_pose.orientation.w = 0.0;

            if (!planAndExecuteCartesianPath(ready_pose, *arm_move_group_, this->get_logger(), eef_step_, waypoints_, trajectory_))
            {
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(get_logger(), "Successfully moved the arm to ready position.");
            current_phase++;
            goal_handle->publish_feedback(feedback);

            if(rclcpp::ok())
            {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
        }

        void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(pointcloud_mutex_);
            latest_pointcloud_ = msg;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(auto_scanning::AutoScanningServer)