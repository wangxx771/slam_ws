//
// Created by ZhiangQi on 25-5-5.
//

#ifndef SCANMAP_H
#define SCANMAP_H
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>

#include <FreeDOM-ROS2/common_types.h>
#include <FreeDOM-ROS2/map.h>
#include <FreeDOM-ROS2/utils.h>

namespace freedom{
class ScanMap : public Map{
public:
    struct ScanMapConfig : public Config
    {
        double sensor_min_range;
        double sensor_max_range;
        double sensor_min_z;
        double sensor_max_z;
    };

    struct ScanVoxel
    {
        Point center;
        Index voxel_idx;
        LinearIndex local_voxel_linear_idx;
        DynamicLevel dynamic_level = DynamicLevel::STATIC;
        std::vector<LinearIndex> points;

        ScanVoxel() = default;

        ScanVoxel(const Point& point_,const Index& voxel_idx_,const LinearIndex& local_voxel_linear_idx_) :
            center(point_),voxel_idx(voxel_idx_),local_voxel_linear_idx(local_voxel_linear_idx_){}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<ScanVoxel,Eigen::aligned_allocator<ScanVoxel>> ScanVoxelList;

    struct ScanBlock
    {
        Index block_idx;
        LinearIndex local_block_linear_idx;
        ScanVoxelList scan_voxels;
        LinearIndex2LinearIndexMap local_voxel_linear_idx2scan_voxel_linear_idx;

        ScanBlock() = default;

        ScanBlock(const Index& block_idx_,const LinearIndex& local_block_linear_idx_) :
            block_idx(block_idx_),local_block_linear_idx(local_block_linear_idx_){}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<ScanBlock,Eigen::aligned_allocator<ScanBlock>> ScanBlockList;

    struct LocalScanBlock
    {
        LinearIndex scan_block_linear_idx;
        Points points;

        LocalScanBlock() = default;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::vector<LocalScanBlock,Eigen::aligned_allocator<LocalScanBlock>> LocalScanBlockGrid;

    ScanMap() : Map() {}
    void set_params(const ScanMapConfig& config);
    void reset();

    void build_scan_map(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);

    // for visualization
    inline const Point& get_sensor_origin() const {return sensor_origin;}

    inline const ScanBlockList& get_scan_blocks() const {return scan_blocks;}
    inline ScanBlockList& get_scan_blocks() {return scan_blocks;}

    inline const LocalScanBlockGrid& get_local_blocks() const {return local_blocks;}

    inline ScanVoxel* have_scan_voxel(const Index& voxel_idx);

    inline const IndexBias& get_scan_map_min_idx() const {return scan_map.min_idx;}
    inline const IndexBias& get_scan_map_max_idx() const {return scan_map.max_idx;}
    inline const IndexBias& get_scan_map_idx_size() const {return scan_map.idx_size;}
    inline const PointBias& get_scan_map_min() const {return scan_map.min;}
    inline const PointBias& get_scan_map_max() const {return scan_map.max;}
    inline const PointBias& get_scan_map_size() const {return scan_map.size;}

private:
    void build_scan_blocks(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
    void build_scan_voxels();

    // params
    double sensor_min_range;
    double sensor_max_range;
    double sensor_min_range_squared;
    double sensor_max_range_squared;
    double sensor_min_z;
    double sensor_max_z;

    unsigned int scan_map_horizontal_block_num;
    unsigned int scan_map_vertical_block_num;
    unsigned int scan_map_horizontal_vertical_block_num_multiplied;
    unsigned int scan_map_total_block_num;

    IndexBias scan_map_center_min_bias;
    IndexBias scan_map_center_max_bias;
    IndexBias scan_map_idx_size;
    PointBias scan_map_size;

    // variables
    Point sensor_origin;

    LocalScanBlockGrid local_blocks;
    std::unique_ptr<std::vector<std::mutex>> block_mutexes;

    ScanBlockList scan_blocks;

    RoboCentricMap scan_map;
};

// inline const ScanMap::ScanVoxel* ScanMap::have_scan_voxel(const Index& voxel_idx) const

inline ScanMap::ScanVoxel* ScanMap::have_scan_voxel(const Index& voxel_idx)
{
    Index block_idx;
    LinearIndex local_voxel_linear_idx;
    LinearIndex local_block_linear_idx;
    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
    scan_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

    if(local_blocks[local_block_linear_idx].points.empty())
        return nullptr;

    ScanBlock& scan_block = scan_blocks[local_blocks[local_block_linear_idx].scan_block_linear_idx];
    auto it = scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.find(local_voxel_linear_idx);
    if(it != scan_block.local_voxel_linear_idx2scan_voxel_linear_idx.end())
        return &scan_block.scan_voxels[it->second];
    else
        return nullptr;
}
}
#endif //SCANMAP_H
