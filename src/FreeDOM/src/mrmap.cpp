//
// Created by ZhiangQi on 25-5-5.
//

#include "FreeDOM-ROS2/mrmap.h"

namespace freedom{
unsigned int FreeBlock::voxel2blockMultiples_cubed_;
unsigned int StaticVoxel::subvoxel2voxelMultiples_cubed_;
unsigned int StaticBlock::voxel2blockMultiples_cubed_;

void MRMap::set_params(const MRMapConfig& config)
{
    Map::set_params(config);

    FreeBlock::set_block_param(voxel2blockMultiples_cubed_);
    StaticVoxel::set_voxel_param(subvoxel2voxelMultiples_cubed_);
    StaticBlock::set_block_param(voxel2blockMultiples_cubed_);

    sensor_max_range = config.sensor_max_range;
    sensor_min_z = config.sensor_min_z;
    sensor_max_z = config.sensor_max_z;

    raycast_max_range = std::min({enable_local_map ? config.local_map_range : 1e10,
                                 config.sensor_max_range,
                                 config.raycast_max_range});
    raycast_max_range_squared = raycast_max_range * raycast_max_range;
    raycast_min_z = std::max({enable_local_map ? config.local_map_min_z : -1e10,
                             config.sensor_min_z,
                             config.raycast_min_z});
    raycast_max_z = std::min({enable_local_map ? config.local_map_max_z : 1e10,
                             config.sensor_max_z,
                             config.raycast_max_z});
    raycast_map.set_params(raycast_max_range,raycast_min_z,raycast_max_z,block_size);

    counts_to_free = config.counts_to_free;
    counts_to_revert = config.counts_to_revert;

    conservative_connectivity.set_params(config.conservative_connectivity);
    if(config.aggressive_connectivity <= 26)
    {
        cluster_aggressive_twice = false;
        first_aggressive_connectivity.set_params(config.aggressive_connectivity);
    }
    else if(config.aggressive_connectivity == 80)
    {
        cluster_aggressive_twice = true;
        first_aggressive_connectivity.set_params(26);
        second_aggressive_connectivity.set_params(6);
    }
    else if(config.aggressive_connectivity == 124)
    {
        cluster_aggressive_twice = true;
        first_aggressive_connectivity.set_params(26);
        second_aggressive_connectivity.set_params(26);
    }
    else
    {
        std::cout << "Aggressive connectivity " << config.aggressive_connectivity << " not supproted" << std::endl;
    }

    local_raycast_flag = std::make_unique<LocalRaycastedFlag>(raycast_map.total_block_num);
    for (auto& flag : *local_raycast_flag)
        flag.store(false, std::memory_order_relaxed);

    local_raycast_blocks = std::make_unique<LocalRaycastBlockGrid>();
    local_raycast_blocks->reserve(raycast_map.total_block_num);
    for (unsigned int i = 0; i < raycast_map.total_block_num; ++i)
        local_raycast_blocks->emplace_back((voxel2blockMultiples_cubed_+63)/64);

    local_occupied_blocks = std::make_unique<LocalOccupiedBlockGrid>();
    local_occupied_blocks->reserve(raycast_map.total_block_num);
    for (unsigned int i = 0; i < raycast_map.total_block_num; ++i)
        local_occupied_blocks->emplace_back((voxel2blockMultiples_cubed_+63)/64);
}

void MRMap::scan_removal(ScanMap& scan)
{
    sensor_origin = scan.get_sensor_origin();
    getVoxelIdxFromPoint(sensor_origin,sensor_origin_voxel_idx);
    getBlockIdxFromPoint(sensor_origin,sensor_origin_block_idx);
    raycast_map.setLocalMapBound(sensor_origin);
    if(enable_local_map)
        local_map.setLocalMapBound(sensor_origin);

    VectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());
    Indices occ_in_freespace;
    Indices voxels_to_revert;

    std::vector<std::future<std::pair<Indices,Indices>>> conservative_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        conservative_threads.emplace_back(std::async(std::launch::async, [&]() {
            Indices thread_occ_in_freespace;
            Indices thread_voxels_to_revert;

            ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                ScanMap::ScanBlock& scan_block = *scan_block_ptr;

                FreeBlock* free_block_ptr = haveFreeBlock(scan_block.block_idx);
                if(free_block_ptr == nullptr)
                    continue;

                LinearIndex local_voxel_linear_idx;
                for(ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    getLocalVoxelLinearIdxFromVoxelIdx(scan_voxel.voxel_idx,local_voxel_linear_idx);
                    FreeVoxel& free_voxel = free_block_ptr->getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

                    ++ free_voxel.occ_counter;
                    free_voxel.free_counter = 0;

                    if(free_voxel.is_free)
                    {
                        thread_occ_in_freespace.push_back(scan_voxel.voxel_idx);
                        scan_voxel.dynamic_level = DynamicLevel::CONSERVATIVE_DYNAMIC;
                    }

                    if(free_voxel.occ_counter >= counts_to_revert)
                    {
                        if(free_voxel.is_free)
                        {
                            free_voxel.is_free = false;
                            free_block_ptr->decreaseFreeCount();
                        }

                        thread_voxels_to_revert.push_back(scan_voxel.voxel_idx);
                    }
                }
            }

            return std::make_pair(std::move(thread_occ_in_freespace), std::move(thread_voxels_to_revert));
        }));
    }

    for (auto& thread : conservative_threads)
    {
        auto pair = thread.get();
        occ_in_freespace.insert(occ_in_freespace.end(),std::make_move_iterator(pair.first.begin()),std::make_move_iterator(pair.first.end()));
        voxels_to_revert.insert(voxels_to_revert.end(),std::make_move_iterator(pair.second.begin()),std::make_move_iterator(pair.second.end()));
    }

    Indices moderate_update_list;
    for(const Index& conservative_voxel_idx : occ_in_freespace)
    {
        Index neighbour_voxel_idx;
        for(auto offset : conservative_connectivity.offsets)
        {
            neighbour_voxel_idx = conservative_voxel_idx + offset;

            ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
            if(!scan_voxel_ptr)
                continue;

            ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
            if(scan_voxel.dynamic_level < DynamicLevel::MODERATE_DYNAMIC)
            {
                scan_voxel.dynamic_level = DynamicLevel::MODERATE_DYNAMIC;
                moderate_update_list.emplace_back(neighbour_voxel_idx);
            }
        }
    }

    Indices aggressive_update_list;
    for(const Index& moderate_voxel_idx : moderate_update_list)
    {
        Index neighbour_voxel_idx;
        for(auto offset : first_aggressive_connectivity.offsets)
        {
            neighbour_voxel_idx = moderate_voxel_idx + offset;

            ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
            if(!scan_voxel_ptr)
                continue;

            ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
            if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
            {
                scan_voxel.dynamic_level = DynamicLevel::AGGRESSIVE_DYNAMIC;
                aggressive_update_list.emplace_back(neighbour_voxel_idx);
            }
        }
    }

    if(cluster_aggressive_twice)
    {
        for(const Index& aggressive_voxel_idx : aggressive_update_list)
        {
            Index neighbour_voxel_idx;
            for(auto offset : second_aggressive_connectivity.offsets)
            {
                neighbour_voxel_idx = aggressive_voxel_idx + offset;

                ScanMap::ScanVoxel* scan_voxel_ptr = scan.have_scan_voxel(neighbour_voxel_idx);
                if(!scan_voxel_ptr)
                    continue;

                ScanMap::ScanVoxel& scan_voxel = *scan_voxel_ptr;
                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                {
                    scan_voxel.dynamic_level = DynamicLevel::AGGRESSIVE_DYNAMIC;
                }
            }
        }
    }

    for(const Index& revert_voxel_idx : voxels_to_revert)
    {
        Index neighbour_voxel_idx;
        for(auto offset : conservative_connectivity.offsets)
        {
            neighbour_voxel_idx = revert_voxel_idx + offset;

            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            getBlockIdxFromVoxelIdx(neighbour_voxel_idx,block_idx);
            FreeBlock* free_block_ptr = haveFreeBlock(block_idx);

            if(free_block_ptr == nullptr)
                continue;

            getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_voxel_linear_idx);
            FreeVoxel& free_voxel = free_block_ptr->getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

            if(free_voxel.is_free)
            {
                free_voxel.is_free = false;
                free_block_ptr->decreaseFreeCount();
            }
        }
    }
}

void MRMap::freespace_estimation(const ScanMap& scan, const DepthImage& depth_image, Indices& freespace_incremental)
{
    constVectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());

    std::vector<std::future<void>> occ_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        occ_threads.emplace_back(std::async(std::launch::async, [&]() {
            LinearIndex local_block_linear_idx;

            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;

                if( scan_block.block_idx.x() < raycast_map.min_idx.x() || scan_block.block_idx.x() >= raycast_map.max_idx.x() ||
                    scan_block.block_idx.y() < raycast_map.min_idx.y() || scan_block.block_idx.y() >= raycast_map.max_idx.y() ||
                    scan_block.block_idx.z() < raycast_map.min_idx.z() || scan_block.block_idx.z() >= raycast_map.max_idx.z())
                    continue;

                raycast_map.getLocalBlockLinearIdxFromBlockIdx(scan_block.block_idx,local_block_linear_idx);
                LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];

                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    local_occupied_block.occ(scan_voxel.local_voxel_linear_idx);
                }
            }
        }));
    }

    for (auto& thread : occ_threads)
    {
        thread.get();
    }

    scan_block_getter.reset();
    std::vector<std::future<void>> raycast_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        raycast_threads.emplace_back(std::async(std::launch::async, [&]() {
            Index voxel_idx;
            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            LinearIndex local_block_linear_idx;

            RayCaster raycaster(sensor_origin,sensor_origin_voxel_idx,voxel_idx,voxel_size);

            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;
                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {
                    Point raycast_end = closestPointInRaycastRange(scan_voxel.center);
                    Index raycast_end_voxel_idx;
                    getVoxelIdxFromPoint(raycast_end,raycast_end_voxel_idx);

                    raycaster.setRayEnd(raycast_end,raycast_end_voxel_idx);

                    do
                    {
                        getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                        getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                        raycast_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

                        LocalRaycastBlock& local_raycast_block = (*local_raycast_blocks)[local_block_linear_idx];
                        LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];
                        std::atomic<bool>& local_raycasted_flag = (*local_raycast_flag)[local_block_linear_idx];

                        if(local_occupied_block.is_occ(local_voxel_linear_idx))
                            break;

                        local_raycast_block.free(local_voxel_linear_idx);

                        if(!local_raycasted_flag.load(std::memory_order_relaxed))
                            local_raycasted_flag.store(true, std::memory_order_relaxed);

                    }while(raycaster.step());
                }
            }
        }));
    }

    for (auto& thread : raycast_threads)
    {
        thread.get();
    }

    // raycast enhancement
    std::vector<std::future<void>> raycast_enhance_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        raycast_enhance_threads.emplace_back(std::async(std::launch::async, [&]() {
            Index voxel_idx;
            Index block_idx;
            LinearIndex local_voxel_linear_idx;
            LinearIndex local_block_linear_idx;

            RayCaster raycaster(sensor_origin,sensor_origin_voxel_idx,voxel_idx,voxel_size);

            for(const Point& point : depth_image.get_enhanced_pointcloud())
            {
                Point raycast_end = closestPointInRaycastRange(point);
                Index raycast_end_voxel_idx;
                getVoxelIdxFromPoint(raycast_end,raycast_end_voxel_idx);

                raycaster.setRayEnd(raycast_end,raycast_end_voxel_idx);

                do
                {
                    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                    raycast_map.getLocalBlockLinearIdxFromBlockIdx(block_idx,local_block_linear_idx);

                    LocalRaycastBlock& local_raycast_block = (*local_raycast_blocks)[local_block_linear_idx];
                    LocalOccupiedBlock& local_occupied_block = (*local_occupied_blocks)[local_block_linear_idx];
                    std::atomic<bool>& local_raycasted_flag = (*local_raycast_flag)[local_block_linear_idx];

                    if(local_occupied_block.is_occ(local_voxel_linear_idx))
                        break;

                    local_raycast_block.free(local_voxel_linear_idx);

                    if(!local_raycasted_flag.load(std::memory_order_relaxed))
                        local_raycasted_flag.store(true, std::memory_order_relaxed);

                }while(raycaster.step());
            }
        }));
    }

    for (auto& thread : raycast_enhance_threads)
    {
        thread.get();
    }

    std::vector<std::pair<LinearIndex, Index>, Eigen::aligned_allocator<std::pair<LinearIndex, Index>>> free_space_update_list;

    LinearIndex raycast_map_linear_block_idx = 0;
    Index raycast_map_block_idx = raycast_map.min_idx;
    for(const auto& flag : *local_raycast_flag)
    {
        if(flag)
        {
            FreeBlock& free_block = getFreeBlock(raycast_map_block_idx);

            if(!free_block.is_free())
                free_space_update_list.emplace_back(std::make_pair(raycast_map_linear_block_idx,raycast_map_block_idx));
        }
        ++ raycast_map_linear_block_idx;
        incrementIdx(raycast_map_block_idx,raycast_map.min_idx, raycast_map.max_idx);
    }

    std::vector<Indices> free_voxel_update_lists(num_threads);
    constVectorElementGetter<std::pair<LinearIndex,Index>> raycasted_block_idx_getter(free_space_update_list);

    std::vector<std::future<void>> free_space_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        free_space_threads.emplace_back(std::async(std::launch::async, [&,i]() {

            const std::pair<LinearIndex,Index>* raycasted_block_idx_ptr;
            while(raycasted_block_idx_getter.get_ptr(raycasted_block_idx_ptr))
            {
                const LocalRaycastBlock& raycast_block = (*local_raycast_blocks)[raycasted_block_idx_ptr->first];
                FreeBlock& free_block = free_space[raycasted_block_idx_ptr->second];

                LinearIndex local_voxel_linear_idx = 0;
                Index min_voxel_idx = voxel2blockMultiples_ * raycasted_block_idx_ptr->second;
                Index max_voxel_idx = min_voxel_idx + IndexBias(voxel2blockMultiples_,voxel2blockMultiples_,voxel2blockMultiples_);
                Index voxel_idx = min_voxel_idx;
                for(auto& voxels : raycast_block.trversed_voxels)
                {
                    for(unsigned int bit = 0; bit < 64; ++bit)
                    {
                        if (voxels & (1ULL << bit))
                        {
                            FreeVoxel& free_voxel = free_block.getFreeVoxelFromLocalVoxelLinearIdx(local_voxel_linear_idx);

                            free_voxel.free_counter ++;
                            free_voxel.occ_counter = 0;
                            if(!free_voxel.is_free && free_voxel.free_counter >= counts_to_free)
                            {
                                free_voxel_update_lists[i].emplace_back(voxel_idx);
                            }
                        }
                        ++ local_voxel_linear_idx;
                        incrementIdx(voxel_idx,min_voxel_idx,max_voxel_idx);
                    }
                }
            }
       }));
    }

    for (auto& thread : free_space_threads)
    {
        thread.get();
    }

    std::vector<std::future<Indices>> free_voxel_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        free_voxel_threads.emplace_back(std::async(std::launch::async, [&,i]() {
            Indices thread_freespace_incremental;
            for(const Index& voxel_idx : free_voxel_update_lists[i])
            {
                bool all_neighbours_are_free = true;

                Index neighbour_voxel_idx;
                Index neighbour_block_idx;
                LinearIndex local_neighbour_voxel_linear_idx;
                for(auto offset : conservative_connectivity.offsets)
                {
                    neighbour_voxel_idx = voxel_idx + offset;
                    getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                    FreeBlock* free_block_ptr = haveFreeBlock(neighbour_block_idx);

                    if(free_block_ptr == nullptr)
                    {
                        all_neighbours_are_free = false;
                        break;
                    }

                    getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                    FreeVoxel& free_voxel = free_block_ptr->getFreeVoxel(local_neighbour_voxel_linear_idx);
                    if(free_voxel.free_counter < counts_to_free)
                    {
                        all_neighbours_are_free = false;
                        break;
                    }
                }

                if(all_neighbours_are_free)
                {
                    Index block_idx;
                    LinearIndex local_voxel_linear_idx;
                    getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
                    getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);
                    FreeBlock& free_block = getFreeBlock(block_idx);
                    FreeVoxel& free_voxel =  free_block.getFreeVoxel(local_voxel_linear_idx);

                    free_voxel.is_free = true;
                    free_block.increaseFreeCount();

                    thread_freespace_incremental.emplace_back(voxel_idx);
                }
            }
            return thread_freespace_incremental;
        }));
    }

    for (auto& thread : free_voxel_threads)
    {
        Indices thread_freespace_incremental = thread.get();
        freespace_incremental.insert(freespace_incremental.end(),std::make_move_iterator(thread_freespace_incremental.begin()),std::make_move_iterator(thread_freespace_incremental.end()));
    }
}

void MRMap::map_removal(const Indices& freespace_incremental)
{
    typedef std::unordered_map<Index,DynamicLevel,IndexHash> DynamicLevelMap;
    std::unordered_map<unsigned int,DynamicLevelMap> scan2map;

    for(const Index& voxel_idx : freespace_incremental)
    {
        Index block_idx;
        getBlockIdxFromVoxelIdx(voxel_idx,block_idx);

        StaticBlock* static_block_ptr;
        static_block_ptr = haveStaticBlock(block_idx);

        if(static_block_ptr == nullptr)
            continue;

        LinearIndex local_voxel_linear_idx;
        getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);

        if(!static_block_ptr->is_static_voxel_allocated(local_voxel_linear_idx))
            continue;

        StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_voxel_linear_idx);

        std::vector<unsigned int> all_scans;
        for(unsigned int i = 0; i < subvoxel2voxelMultiples_cubed_; ++i)
        {
            unsigned int& scan_seq = static_voxel.scan_in_subvoxel[i];
            if(scan_seq == StaticVoxel::NOT_A_SCAN)
                continue;

            auto it = std::find(all_scans.begin(), all_scans.end(), scan_seq);
            if(it == all_scans.end())
                all_scans.emplace_back(scan_seq);
        }

        for(unsigned int i : all_scans)
        {
            scan2map[i][voxel_idx] = DynamicLevel::CONSERVATIVE_DYNAMIC;
        }
    }

    for(auto& [scan_seq,dynamic_level_map] : scan2map)
    {
        Indices conservative_voxel_list;
        Indices moderate_voxel_list;
        Indices aggressive_voxel_list;
        for(auto& [voxel_idx,dynamic_level] : dynamic_level_map)
        {
            conservative_voxel_list.emplace_back(voxel_idx);
        }

        for(Index& voxel_idx : conservative_voxel_list)
        {
            Index neighbour_voxel_idx;
            Index neighbour_block_idx;
            LinearIndex local_neighbour_voxel_linear_idx;
            for(auto offset : conservative_connectivity.offsets)
            {
                neighbour_voxel_idx = voxel_idx + offset;

                auto it = dynamic_level_map.find(neighbour_voxel_idx);
                if(it != dynamic_level_map.end())
                    continue;

                getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                if(static_block_ptr == nullptr)
                    continue;

                getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                    continue;

                StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                        == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                    continue;

                dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::MODERATE_DYNAMIC;
                moderate_voxel_list.emplace_back(neighbour_voxel_idx);
            }
        }

        for(Index& voxel_idx : moderate_voxel_list)
        {
            Index neighbour_voxel_idx;
            Index neighbour_block_idx;
            LinearIndex local_neighbour_voxel_linear_idx;
            for(auto offset : first_aggressive_connectivity.offsets)
            {
                neighbour_voxel_idx = voxel_idx + offset;

                auto it = dynamic_level_map.find(neighbour_voxel_idx);
                if(it != dynamic_level_map.end())
                    continue;

                getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                if(static_block_ptr == nullptr)
                    continue;

                getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                    continue;

                StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                        == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                    continue;

                dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::AGGRESSIVE_DYNAMIC;
                aggressive_voxel_list.emplace_back(neighbour_voxel_idx);
            }
        }

        if(!cluster_aggressive_twice)
        {
            for(Index& voxel_idx : aggressive_voxel_list)
            {
                Index neighbour_voxel_idx;
                Index neighbour_block_idx;
                LinearIndex local_neighbour_voxel_linear_idx;
                for(auto offset : second_aggressive_connectivity.offsets)
                {
                    neighbour_voxel_idx = voxel_idx + offset;

                    auto it = dynamic_level_map.find(neighbour_voxel_idx);
                    if(it != dynamic_level_map.end())
                        continue;

                    getBlockIdxFromVoxelIdx(neighbour_voxel_idx,neighbour_block_idx);
                    StaticBlock* static_block_ptr = haveStaticBlock(neighbour_block_idx);

                    if(static_block_ptr == nullptr)
                        continue;

                    getLocalVoxelLinearIdxFromVoxelIdx(neighbour_voxel_idx,local_neighbour_voxel_linear_idx);
                    if(!static_block_ptr->is_static_voxel_allocated(local_neighbour_voxel_linear_idx))
                        continue;

                    StaticVoxel& static_voxel = static_block_ptr->getStaticVoxel(local_neighbour_voxel_linear_idx);

                    if( std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq)
                            == static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_)
                        continue;

                    dynamic_level_map[neighbour_voxel_idx] = DynamicLevel::AGGRESSIVE_DYNAMIC;
                }
            }
        }

        Index block_idx;
        LinearIndex local_voxel_linear_idx;
        for(auto& [voxel_idx,target_dynamic_level] : dynamic_level_map)
        {
            getBlockIdxFromVoxelIdx(voxel_idx,block_idx);
            getLocalVoxelLinearIdxFromVoxelIdx(voxel_idx,local_voxel_linear_idx);

            StaticBlock& static_block = getStaticBlock(block_idx);
            StaticVoxel& static_voxel = static_block.getStaticVoxel(local_voxel_linear_idx);

            unsigned int idx = std::find(static_voxel.scan_in_subvoxel,static_voxel.scan_in_subvoxel + subvoxel2voxelMultiples_cubed_, scan_seq) - static_voxel.scan_in_subvoxel;
            DynamicLevel dynamic_level = static_voxel.dynamic_level[idx];

            if(dynamic_level >= target_dynamic_level)
                continue;

            unsigned int decreased_static_occ_count = 0;
            for(unsigned int i = 0; i < subvoxel2voxelMultiples_cubed_; ++i)
            {
                unsigned int& subvoxel_scan_seq = static_voxel.scan_in_subvoxel[i];
                if(subvoxel_scan_seq != scan_seq)
                    continue;

                static_voxel.dynamic_level[i] = target_dynamic_level;

                if(dynamic_level == DynamicLevel::STATIC)
                    ++ decreased_static_occ_count;

            }

            static_voxel.static_occ_count -= decreased_static_occ_count;
            if(static_voxel.is_free())
            {
                static_block.free_voxel(local_voxel_linear_idx);

                if(static_block.is_free())
                    static_space.erase(block_idx);
            }
        }
    }
}

void MRMap::staticspace_integration(const ScanMap& scan, unsigned int scan_seq)
{
    for(const ScanMap::ScanBlock& scan_block : scan.get_scan_blocks())
    {
        static_space[scan_block.block_idx];
    }

    constVectorElementGetter<ScanMap::ScanBlock> scan_block_getter(scan.get_scan_blocks());

    std::vector<std::future<void>> integrate_threads;
    for (size_t i = 0; i < num_threads; ++i) {
        integrate_threads.emplace_back(std::async(std::launch::async, [&]() {
            const ScanMap::ScanBlock* scan_block_ptr;
            while(scan_block_getter.get_ptr(scan_block_ptr))
            {
                const ScanMap::ScanBlock& scan_block = *scan_block_ptr;
                const ScanMap::LocalScanBlock& local_scan_block = scan.get_local_blocks()[scan_block.local_block_linear_idx];
                StaticBlock& static_block = getStaticBlock(scan_block.block_idx);
                for(const ScanMap::ScanVoxel& scan_voxel : scan_block.scan_voxels)
                {

                    if(!static_block.is_static_voxel_allocated(scan_voxel.local_voxel_linear_idx))
                        static_block.allocate_voxel(scan_voxel.local_voxel_linear_idx);

                    Point scan_voxel_coord = voxel_size * scan_voxel.voxel_idx.cast<double>();
                    StaticVoxel& static_voxel = static_block.getStaticVoxel(scan_voxel.local_voxel_linear_idx);

                    for(const LinearIndex& point_idx : scan_voxel.points)
                    {
                        const Point& point = local_scan_block.points[point_idx];

                        Index subvoxel_idx;
                        LinearIndex local_subvoxel_linear_idx;
                        getSubVoxelIdxFromPoint(point,subvoxel_idx);
                        getLocalSubVoxelLinearIdxFromSubvoxelIdx(subvoxel_idx,local_subvoxel_linear_idx);

                        unsigned int& subvoxel_scan_id = static_voxel.scan_in_subvoxel[local_subvoxel_linear_idx];
                        Pointf& subvoxel_point = static_voxel.points[local_subvoxel_linear_idx];
                        DynamicLevel& subvoxel_dynamic_level = static_voxel.dynamic_level[local_subvoxel_linear_idx];
                        if(subvoxel_scan_id != StaticVoxel::NOT_A_SCAN && subvoxel_scan_id != scan_seq)
                        {
                            if(subvoxel_dynamic_level > scan_voxel.dynamic_level)
                            {
                                subvoxel_scan_id = scan_seq;
                                subvoxel_point = (point - scan_voxel_coord).cast<float>();
                                subvoxel_dynamic_level = scan_voxel.dynamic_level;

                                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                                    static_voxel.static_occ_count ++;
                            }
                        }
                        else if(subvoxel_scan_id == scan_seq)
                            ;
                        else
                        {
                            subvoxel_scan_id = scan_seq;
                            subvoxel_point = (point - scan_voxel_coord).cast<float>();
                            subvoxel_dynamic_level = scan_voxel.dynamic_level;
                            static_voxel.occ_count ++;

                                if(scan_voxel.dynamic_level < DynamicLevel::AGGRESSIVE_DYNAMIC)
                                    static_voxel.static_occ_count ++;
                        }
                    }
                }
            }
        }));
    }

    for (auto& thread : integrate_threads)
    {
        thread.get();
    }
}

void MRMap::reset()
{
    for (auto& flag : *local_raycast_flag)
        flag.store(false, std::memory_order_relaxed);

    for(LocalRaycastBlock& block : *local_raycast_blocks)
    {
        block.reset();
    }

    for(LocalOccupiedBlock& block : *local_occupied_blocks)
    {
        block.reset();
    }
}

void MRMap::remove_map_out_of_bound()
{
    if(enable_local_map)
    {
        Indices free_space_remove_list;
        for(const auto& it : free_space)
        {
            if( it.first.x() < local_map.min_idx.x() || it.first.x() >= local_map.max_idx.x() ||
                it.first.y() < local_map.min_idx.y() || it.first.y() >= local_map.max_idx.y() ||
                it.first.z() < local_map.min_idx.z() || it.first.z() >= local_map.max_idx.z())
                free_space_remove_list.emplace_back(it.first);
        }
        for(const auto& idx : free_space_remove_list)
        {
            free_space.erase(idx);
        }

        Indices static_space_remove_list;
        for(const auto& it : static_space)
        {
            if( it.first.x() < local_map.min_idx.x() || it.first.x() >= local_map.max_idx.x() ||
                it.first.y() < local_map.min_idx.y() || it.first.y() >= local_map.max_idx.y() ||
                it.first.z() < local_map.min_idx.z() || it.first.z() >= local_map.max_idx.z())
                static_space_remove_list.emplace_back(it.first);
        }
        for(const auto& idx : static_space_remove_list)
        {
            static_space.erase(idx);
        }
    }
}
}