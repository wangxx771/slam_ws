//
// Created by ZhiangQi on 25-5-5.
//

#ifndef MRMAP_H
#define MRMAP_H
#include <Eigen/Eigen>
#include <cmath>
#include <limits>

#include "FreeDOM-ROS2/map.h"
#include "FreeDOM-ROS2/scanmap.h"
#include "FreeDOM-ROS2/utils.h"
#include "FreeDOM-ROS2/common_types.h"
#include "FreeDOM-ROS2/raycast.h"
#include "FreeDOM-ROS2/depth_image.h"

namespace freedom{
class FreeVoxel
{
public:
    bool is_free;
    unsigned int free_counter;
    unsigned int occ_counter;

    FreeVoxel():is_free(false),free_counter(0),occ_counter(0){}
};

class FreeBlock
{
public:
    unsigned int free_count;
    FreeVoxel* voxels;

    static unsigned int voxel2blockMultiples_cubed_;

    FreeBlock():free_count(0){allocate_all();}

    ~FreeBlock(){free_all();}

    inline static void set_block_param(unsigned int& voxel2blockMultiples_cubed){voxel2blockMultiples_cubed_ = voxel2blockMultiples_cubed;}

    inline bool is_free() const;

    inline FreeVoxel& getFreeVoxelFromLocalVoxelLinearIdx (const unsigned int local_voxel_linear_idx) const;

    inline FreeVoxel& getFreeVoxel(unsigned int local_voxel_linear_idx)
    {
        return voxels[local_voxel_linear_idx];
    }

    inline const FreeVoxel& getFreeVoxel (unsigned int local_voxel_linear_idx) const
    {
        return voxels[local_voxel_linear_idx];
    }

    inline void increaseFreeCount();
    inline void decreaseFreeCount();

private:
    inline void allocate_all();
    inline void free_all();
};

class StaticVoxel
{
public:
    unsigned int* scan_in_subvoxel;
    Pointf* points;
    DynamicLevel* dynamic_level;

    unsigned int occ_count;
    unsigned int static_occ_count;

    static unsigned int subvoxel2voxelMultiples_cubed_;
    static constexpr unsigned int NOT_A_SCAN = std::numeric_limits<unsigned int>::max();

    StaticVoxel():occ_count(0),static_occ_count(0)
    {
        scan_in_subvoxel = new unsigned int[subvoxel2voxelMultiples_cubed_];
        points = new Pointf[subvoxel2voxelMultiples_cubed_];
        dynamic_level = new DynamicLevel[subvoxel2voxelMultiples_cubed_];
        std::fill(scan_in_subvoxel,scan_in_subvoxel+subvoxel2voxelMultiples_cubed_,NOT_A_SCAN);
    }

    ~StaticVoxel()
    {
        delete[] scan_in_subvoxel;
        delete[] points;
        delete[] dynamic_level;
    }

    inline static void set_voxel_param(unsigned int& subvoxel2voxelMultiples_cubed){subvoxel2voxelMultiples_cubed_ = subvoxel2voxelMultiples_cubed;}

    inline bool is_free(){ return (occ_count == 0); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StaticBlock
{
public:
    StaticVoxel** voxels_ptr;
    unsigned int occ_count;

    static unsigned int voxel2blockMultiples_cubed_;

    StaticBlock():occ_count(0)
    {
        voxels_ptr = new StaticVoxel*[voxel2blockMultiples_cubed_];
        std::fill(voxels_ptr,voxels_ptr+voxel2blockMultiples_cubed_,nullptr);
    }

    ~StaticBlock()
    {
        for(unsigned int i=0; i<voxel2blockMultiples_cubed_; i++)
        {
            delete voxels_ptr[i];
        }
        delete[] voxels_ptr;
    }

    inline static void set_block_param(unsigned int& voxel2blockMultiples_cubed){voxel2blockMultiples_cubed_ = voxel2blockMultiples_cubed;}

    inline bool is_static_voxel_allocated(unsigned int local_voxel_linear_idx) const
    {
        return (voxels_ptr[local_voxel_linear_idx] != nullptr);
    }

    inline StaticVoxel& getStaticVoxel(unsigned int local_voxel_linear_idx)
    {
        return *voxels_ptr[local_voxel_linear_idx];
    }

    inline const StaticVoxel& getStaticVoxel(unsigned int local_voxel_linear_idx) const
    {
        return *voxels_ptr[local_voxel_linear_idx];
    }

    inline bool is_free() const
    {
        return (occ_count == 0);
    }

    inline void allocate_voxel(unsigned int local_voxel_linear_idx)
    {
        voxels_ptr[local_voxel_linear_idx] = new StaticVoxel();
        occ_count ++;
    }

    inline void free_voxel(unsigned int local_voxel_linear_idx)
    {
        delete voxels_ptr[local_voxel_linear_idx];
        voxels_ptr[local_voxel_linear_idx] = nullptr;
        occ_count --;
    }
};

class MRMap : public Map{
public:
    struct MRMapConfig : public Config
    {
        double sensor_max_range;
        double sensor_min_z;
        double sensor_max_z;

        double raycast_max_range;
        double raycast_min_z;
        double raycast_max_z;

        unsigned int counts_to_free;
        unsigned int counts_to_revert;

        unsigned int conservative_connectivity;
        unsigned int aggressive_connectivity;
    };

    struct LocalRaycastBlock
    {
        std::vector<std::atomic<uint64_t>> trversed_voxels;

        LocalRaycastBlock(const unsigned int num_uint64_voxels) :
            trversed_voxels(num_uint64_voxels){reset();}

        inline void reset();
        inline void free(const unsigned int local_voxel_linear_idx);
    };
    typedef std::vector<std::atomic<bool>,Eigen::aligned_allocator<std::atomic<bool>>> LocalRaycastedFlag;
    typedef std::vector<LocalRaycastBlock,Eigen::aligned_allocator<LocalRaycastBlock>> LocalRaycastBlockGrid;

    struct LocalOccupiedBlock
    {
        std::vector<std::atomic<uint64_t>> occupancy;

        LocalOccupiedBlock(const unsigned int num_uint64_voxels) :
            occupancy(num_uint64_voxels){reset();}

        inline void reset();
        inline void occ(const unsigned int local_voxel_linear_idx);
        inline bool is_occ(const unsigned int local_voxel_linear_idx) const;
    };
    typedef std::vector<LocalOccupiedBlock,Eigen::aligned_allocator<LocalOccupiedBlock>> LocalOccupiedBlockGrid;

    MRMap() : Map() {}

    void set_params(const MRMapConfig& config);
    void reset();

    void scan_removal(ScanMap& scan);
    void freespace_estimation(const ScanMap& scan, const DepthImage& depth_image, Indices& freespace_incremental);
    void map_removal(const Indices& freespace_incremental);
    void staticspace_integration(const ScanMap& scan, unsigned int scan_seq);
    void remove_map_out_of_bound();

    inline const LocalRaycastedFlag& get_raycasted_flags() const {return *local_raycast_flag;}
    inline const LocalRaycastBlockGrid& get_raycast_blocks() const {return *local_raycast_blocks;}
    inline const LocalOccupiedBlockGrid& get_occupied_blocks() const {return *local_occupied_blocks;}

    inline const IndexBias& get_raycast_map_min_idx() const {return raycast_map.min_idx;}
    inline const IndexBias& get_raycast_map_max_idx() const {return raycast_map.max_idx;}
    inline const IndexBias& get_raycast_map_idx_size() const {return raycast_map.idx_size;}
    inline const PointBias& get_raycast_map_min() const {return raycast_map.min;}
    inline const PointBias& get_raycast_map_max() const {return raycast_map.max;}
    inline const PointBias& get_raycast_map_size() const {return raycast_map.size;}

    inline const std::unordered_map<Index,FreeBlock,IndexHash>& get_free_blocks() const {return free_space;}
    inline const std::unordered_map<Index,StaticBlock,IndexHash>& get_static_blocks() const {return static_space;}

private:
    // params
    double sensor_max_range;
    double sensor_min_z;
    double sensor_max_z;

    double raycast_max_range;
    double raycast_max_range_squared;
    double raycast_min_z;
    double raycast_max_z;

    unsigned int counts_to_free;
    unsigned int counts_to_revert;

    Neighbours conservative_connectivity;
    bool cluster_aggressive_twice;
    Neighbours first_aggressive_connectivity;
    Neighbours second_aggressive_connectivity;

    // variables
    Point sensor_origin;
    Index sensor_origin_voxel_idx;
    Index sensor_origin_block_idx;

    std::unique_ptr<LocalRaycastedFlag> local_raycast_flag;
    std::unique_ptr<LocalRaycastBlockGrid> local_raycast_blocks;
    std::unique_ptr<LocalOccupiedBlockGrid> local_occupied_blocks;

    RoboCentricMap raycast_map;

    std::unordered_map<Index,FreeBlock,IndexHash> free_space;
    std::unordered_map<Index,StaticBlock,IndexHash> static_space;

    inline Point closestPointInRaycastRange(const Point& end);

    inline FreeBlock* haveFreeBlock(const Index& block_idx);
    inline FreeBlock& getFreeBlock(const Index& block_idx);

    inline StaticBlock* haveStaticBlock(const Index& block_idx);
    inline StaticBlock& getStaticBlock(const Index& block_idx);
};

inline bool FreeBlock::is_free() const
{
    return (free_count == voxel2blockMultiples_cubed_);
}

inline FreeVoxel& FreeBlock::getFreeVoxelFromLocalVoxelLinearIdx (const unsigned int local_voxel_linear_idx) const
{
    return voxels[local_voxel_linear_idx];
}

inline void FreeBlock::increaseFreeCount()
{
    free_count ++;
}

inline void FreeBlock::decreaseFreeCount()
{
    free_count --;
}

inline void FreeBlock::allocate_all()
{
    voxels = new FreeVoxel[voxel2blockMultiples_cubed_];
}

inline void FreeBlock::free_all()
{
    delete[] voxels;
    voxels = nullptr;
}

inline void MRMap::LocalOccupiedBlock::reset()
{
    for (auto& voxel : occupancy)
        voxel.store(0, std::memory_order_relaxed);
}

inline void MRMap::LocalOccupiedBlock::occ(const unsigned int local_voxel_linear_idx)
{
    occupancy[local_voxel_linear_idx/64].fetch_or(1ULL<<(local_voxel_linear_idx%64), std::memory_order_relaxed);
}

inline bool MRMap::LocalOccupiedBlock::is_occ(const unsigned int local_voxel_linear_idx) const
{
    return (occupancy[local_voxel_linear_idx/64].load(std::memory_order_relaxed) & (1ULL << local_voxel_linear_idx%64));
}

inline void MRMap::LocalRaycastBlock::reset()
{
    for (auto& voxel : trversed_voxels)
        voxel.store(0, std::memory_order_relaxed);
}

inline void MRMap::LocalRaycastBlock::free(const unsigned int local_voxel_linear_idx)
{
    trversed_voxels[local_voxel_linear_idx/64].fetch_or(1ULL<<(local_voxel_linear_idx%64), std::memory_order_relaxed);
}

inline Point MRMap::closestPointInRaycastRange(const Point& end)
{
    PointBias ray = end - sensor_origin;

    if(ray.squaredNorm() <= raycast_max_range_squared && ray.z() > raycast_min_z && ray.z() < raycast_max_z)
        return end;

    double t_r,t_z,min_t;

    t_r = raycast_max_range / ray.norm();

    if(ray.z()>0)
        t_z = raycast_max_z / ray.z();
    else if(ray.z()<0)
        t_z = raycast_min_z / ray.z();
    else
        t_z = 1.0;

    min_t = std::min(t_r,t_z);
    return sensor_origin + ray * min_t;
}

inline FreeBlock* MRMap::haveFreeBlock(const Index& block_idx)
{
    auto it = free_space.find(block_idx);
    if(it != free_space.end())
        return &(it->second);
    else
        return nullptr;
}

inline FreeBlock& MRMap::getFreeBlock(const Index& block_idx)
{
    return free_space[block_idx];
}

inline StaticBlock* MRMap::haveStaticBlock(const Index& block_idx)
{
    auto it = static_space.find(block_idx);
    if(it != static_space.end())
        return &(it->second);
    else
        return nullptr;
}

inline StaticBlock& MRMap::getStaticBlock(const Index& block_idx)
{
    return static_space[block_idx];
}
}
#endif //MRMAP_H
