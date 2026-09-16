//
// Created by ZhiangQi on 25-5-5.
//

#ifndef DEPTH_IMAGE_H
#define DEPTH_IMAGE_H

#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "FreeDOM-ROS2/common_types.h"

namespace freedom
{
    class DepthImage
    {
    public:
        struct DepthImageConfig
        {
            double lidar_horizon_fov;
            double lidar_vertical_fov_upper;
            double lidar_vertical_fov_lower;
            unsigned int depth_image_vertical_lines;

            double depth_image_min_range;
            double max_raycast_enhancement_range;
            double raycast_enhancement_depth_margin;

            unsigned int inpaint_size;
            unsigned int erosion_size;
            double min_raycast_enhancement_area;
            double top_margin;

            bool learn_fov;
            bool enable_fov_mask;
            std::string fov_mask_path;

            unsigned int num_threads;
        };

        DepthImage(){}

        void set_params(const DepthImageConfig& config);

        void raycast_enhancement(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
        inline const cv::Mat& get_depth_image() const { return *depth_image; }
        inline const cv::Mat& get_raycast_enhance_region() const { return *raycast_enhance_region; }
        inline const cv::Mat& get_inpainted_image() const { return *inpainted_image; }
        inline const cv::Mat& get_eroded_raycast_enhance_region() const { return *eroded_raycast_enhance_region; }
        inline const cv::Mat& get_fov_image() const { return *fov_image; }
        inline const Points& get_enhanced_pointcloud() const { return enhanced_pointcloud; }

        ~DepthImage();
    private:
        // LiDAR fov
        double lidar_horizon_fov;
        double lidar_vertical_fov;
        double lidar_vertical_fov_upper;
        double lidar_vertical_fov_lower;

        // depth image
        unsigned int rows;
        unsigned int cols;
        double depth_unit,depth_unit_inv;
        double horizon_res,vertical_res;
        double horizon_res_half,vertical_res_half;
        double horizon_res_inv,vertical_res_inv;

        double vertical_fov;
        double vertical_fov_upper;
        double vertical_fov_lower;

        double depth_image_min_range;
        double max_raycast_enhancement_range;
        unsigned int raycast_enhancement_depth_margin;

        unsigned int inpaint_size;
        unsigned int erosion_size;
        unsigned int min_raycast_enhancement_area_pixel_num;
        unsigned int top_margin;

        bool is_panorama;
        unsigned int col_margin;
        unsigned int cols_w_margin;
        unsigned int image_col_min;
        unsigned int image_col_max;
        cv::Range right_margin_src;
        cv::Range left_margin_src;
        cv::Range right_margin;
        cv::Range left_margin;
        cv::Range image_col_range;

        // learn fov mask
        bool learn_fov;
        unsigned int scan_count;
        std::unique_ptr<cv::Mat> average_image;

        // fov mask
        bool enable_fov_mask;
        std::string fov_mask_path;

        // images
        std::unique_ptr<cv::Mat> depth_image;
        std::unique_ptr<cv::Mat> raycast_enhance_region;
        std::unique_ptr<cv::Mat> inpainted_image;
        std::unique_ptr<cv::Mat> eroded_raycast_enhance_region;
        std::unique_ptr<cv::Mat> fov_image;

        // enhanced pointcloud
        Points enhanced_pointcloud;

        // erosion core
        cv::Mat erosion_element;

        unsigned int num_threads;

        void reset();

        bool point2idx(const pcl::PointXYZ& point, unsigned int& row, unsigned int& col,unsigned int& depth) const;

        void idx2point(unsigned int row, unsigned int col,unsigned int depth,Point& point) const;
    };


}

#endif //DEPTH_IMAGE_H
