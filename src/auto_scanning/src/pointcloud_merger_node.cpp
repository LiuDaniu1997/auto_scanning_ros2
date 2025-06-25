#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>

using namespace std;
namespace fs = std::filesystem;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : Node("pointcloud_merger_node")
    {
        // load the config file
        config_ = YAML::LoadFile("src/auto_scanning/config/config.yaml");
        point_cloud_save_path_ = config_["point_cloud_save_path"].as<std::string>();
        merged_point_cloud_save_path_ = config_["merged_point_cloud_save_path"].as<std::string>();

        fs::path dir(point_cloud_save_path_);
        if (!fs::exists(dir) || !fs::is_directory(dir)) {
            RCLCPP_ERROR(this->get_logger(), "Provided path is not a directory.");
            return;
        }

        std::vector<fs::path> pcd_files;
        for (const auto &entry : fs::directory_iterator(dir)) {
            if (entry.path().extension() == ".pcd")
                pcd_files.push_back(entry.path());
        }

        if (pcd_files.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "At least 3 PCD files required.");
            return;
        }

        PointCloud::Ptr cloud1(new PointCloud), cloud2(new PointCloud), cloud3(new PointCloud);
        pcl::io::loadPCDFile(pcd_files[0], *cloud1);
        pcl::io::loadPCDFile(pcd_files[1], *cloud2);
        pcl::io::loadPCDFile(pcd_files[2], *cloud3);

        cloud1 = removePlane(cloud1);
        cloud2 = removePlane(cloud2);
        cloud3 = removePlane(cloud3);

        PointCloud::Ptr merged1 = registerAndMergeCloudsWithGICP(cloud1, cloud2);
        PointCloud::Ptr final = registerAndMergeCloudsWithGICP(cloud3, merged1);

        std::stringstream filename;
        filename << merged_point_cloud_save_path_ << "merged_point_cloud_" << std::fixed << std::setprecision(3)
                << this->now().seconds() << ".pcd";

        if (pcl::io::savePCDFileBinary(filename.str(), *final) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.str().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud.");
            return;
        }
    }

private:
    // config node
    YAML::Node config_;
    std::string point_cloud_save_path_;
    std::string merged_point_cloud_save_path_;

    PointCloud::Ptr removePlane(const PointCloud::Ptr &cloud)
    {
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);

        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coeff);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No plane found.");
            return cloud;
        }

        pcl::ExtractIndices<PointT> extract;
        PointCloud::Ptr cloud_out(new PointCloud);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_out);

        return cloud_out;
    }

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

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(src_filtered);
        gicp.setInputTarget(tgt_filtered);
        gicp.setMaxCorrespondenceDistance(0.01);
        gicp.setTransformationEpsilon(1e-10);
        gicp.setEuclideanFitnessEpsilon(0.01);
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
    rclcpp::spin_some(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
