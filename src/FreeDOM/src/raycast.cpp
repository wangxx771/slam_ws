//
// Created by ZhiangQi on 25-5-5.
//

#include "FreeDOM-ROS2/raycast.h"

namespace freedom
{
    void RayCaster::setRayEnd(const Point& end_, const Index& end_idx_)
    {
        idx = start_idx;

        end = end_;
        end_idx = end_idx_;

        PointBias velocity = end - start;
        step_x = velocity.x() > 0? 1 : velocity.x()<0? -1 : 0;
        step_y = velocity.y() > 0? 1 : velocity.y()<0? -1 : 0;
        step_z = velocity.z() > 0? 1 : velocity.z()<0? -1 : 0;

        t_step_x = velocity.x() != 0 ? res / abs(velocity.x()) : MAX_TIME;
        t_step_y = velocity.y() != 0 ? res / abs(velocity.y()) : MAX_TIME;
        t_step_z = velocity.z() != 0 ? res / abs(velocity.z()) : MAX_TIME;

        t_to_next_bound_x = tToNextBound(start.x(),velocity.x(),t_step_x);
        t_to_next_bound_y = tToNextBound(start.y(),velocity.y(),t_step_y);
        t_to_next_bound_z = tToNextBound(start.z(),velocity.z(),t_step_z);
    }

    bool RayCaster::step()
    {
        if(t_to_next_bound_x < t_to_next_bound_y && t_to_next_bound_x < t_to_next_bound_z)
        {
            idx.x() += step_x;
            t_to_next_bound_x += t_step_x;
        }
        else if(t_to_next_bound_y < t_to_next_bound_z)
        {
            idx.y() += step_y;
            t_to_next_bound_y += t_step_y;
        }
        else
        {
            idx.z() += step_z;
            t_to_next_bound_z += t_step_z;
        }

        return (t_to_next_bound_x < MAX_TIME || t_to_next_bound_y < MAX_TIME || t_to_next_bound_z < MAX_TIME);





        // else


    }
}