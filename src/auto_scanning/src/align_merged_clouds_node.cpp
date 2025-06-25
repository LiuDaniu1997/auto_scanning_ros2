#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class AlignMergedCloudsNode : public rclcpp::Node
{
public:
    AlignMergedCloudsNode() : Node("align_merged_clouds_node")
    {
        config_ = YAML::LoadFile("src/auto_scanning/config/config.yaml");
        merged_point_cloud_save_path_ = config_["merged_point_cloud_save_path"].as<std::string>();
        aligned_point_cloud_save_path_ = config_["aligned_point_cloud_save_path"].as<std::string>();

        fs::path dir(merged_point_cloud_save_path_);
        if (!fs::exists(dir) || !fs::is_directory(dir)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid merged point cloud path.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Start merging the point clouds...");
        std::vector<fs::path> pcd_files;
        for (const auto &entry : fs::directory_iterator(dir)) {
            if (entry.path().extension() == ".pcd")
                pcd_files.push_back(entry.path());
        }

        if (pcd_files.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Need at least two merged PCD files.");
            return;
        }

        std::sort(pcd_files.begin(), pcd_files.end());

        PointCloud::Ptr cloud1(new PointCloud), cloud2(new PointCloud);
        pcl::io::loadPCDFile(pcd_files[0], *cloud1);
        pcl::io::loadPCDFile(pcd_files[1], *cloud2);

        PointCloud::Ptr final = registerAndMergeCloudsWithGICP(cloud2, cloud1); // cloud 1: target，cloud2 will be aligned to cloud 1

        std::stringstream filename;
        filename << aligned_point_cloud_save_path_ << "final_aligned_cloud_" << std::fixed
                 << std::setprecision(3) << this->now().seconds() << ".pcd";

        if (pcl::io::savePCDFileBinary(filename.str(), *final) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved final aligned point cloud to %s", filename.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save final point cloud.");
        }
    }

private:
    YAML::Node config_;
    std::string merged_point_cloud_save_path_;
    std::string aligned_point_cloud_save_path_;

    PointCloud::Ptr registerAndMergeCloudsWithGICP(const PointCloud::Ptr &source, const PointCloud::Ptr &target)
    {
        float voxel_size = 0.001f;

        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        PointCloud::Ptr src_filtered(new PointCloud), tgt_filtered(new PointCloud);
        voxel.setInputCloud(source);
        voxel.filter(*src_filtered);
        voxel.setInputCloud(target);
        voxel.filter(*tgt_filtered);

        Eigen::Affine3f source_pose = Eigen::Affine3f::Identity();
        source_pose.translation() << 0.5f, 0.0f, 0.0f;
        source_pose.rotate(Eigen::AngleAxisf(-M_PI / 2.0, Eigen::Vector3f::UnitZ()));

        Eigen::Affine3f target_pose = Eigen::Affine3f::Identity();
        target_pose.translation() << 0.5f, 0.0f, 0.0f;

        Eigen::Affine3f initial_transform = target_pose * source_pose.inverse();

        PointCloud::Ptr src_filtered_with_init_guess(new PointCloud);
        pcl::transformPointCloud(*src_filtered, *src_filtered_with_init_guess, initial_transform);

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(src_filtered_with_init_guess);
        gicp.setInputTarget(tgt_filtered);
        gicp.setMaxCorrespondenceDistance(0.001);
        gicp.setTransformationEpsilon(1e-10);
        gicp.setEuclideanFitnessEpsilon(0.001);
        gicp.setMaximumIterations(50);

        PointCloud::Ptr aligned(new PointCloud);
        gicp.align(*aligned);

        PointCloud::Ptr merged(new PointCloud);
        *merged = *aligned + *tgt_filtered;

        PointCloud::Ptr merged_filtered(new PointCloud);
        voxel.setInputCloud(merged);
        voxel.filter(*merged_filtered);

        return merged_filtered;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<AlignMergedCloudsNode>());
    rclcpp::shutdown();
    return 0;
}