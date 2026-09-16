//
// Created by ZhiangQi on 25-5-5.
//

#ifndef UTILS_H
#define UTILS_H

#include "slam_log/log.h"
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <FreeDOM-ROS2/common_types.h>

namespace freedom{
class Param{
public:
    Param(rclcpp::Node& node): node_(node){}

    template <typename T>
    void getParam(const std::string& param_name, T& param_value, const T& default_value)
    {
        if (!node_.has_parameter(param_name)) {
            node_.declare_parameter(param_name, default_value);
        }

        param_value = node_.get_parameter(param_name).get_value<T>();

        if (node_.get_parameter(param_name).get_value<T>() == default_value) {
            PL_WARN(node_.get_logger(), "param missing: %s, set default: %s",
                        param_name.c_str(), std::to_string(default_value).c_str());
        }

        return;
    }

    template <typename T>
    void getParam(const std::string& param_name, T& param_value)
    {
        if (!node_.has_parameter(param_name)) {
            node_.declare_parameter<T>(param_name);
        }

        if (node_.get_parameter(param_name, param_value)) {
            return;
        }

        PL_ERROR(node_.get_logger(), "param missing: %s, node exit", param_name.c_str());
        rclcpp::shutdown();
        std::exit(EXIT_FAILURE);
    }

private:
    rclcpp::Node& node_;
};

class Timer{
public:
    struct TimerData
    {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        double total_time = 0;
        int count = 0;
        bool is_running = false;

        TimerData(const std::string& timer_name = "") : name(timer_name) {}

        void start()
        {
            start_time = std::chrono::high_resolution_clock::now();
            is_running = true;
        }

        void stop() {
            if (!is_running) {
                std::cout << "Timer is not running!" << std::endl;
                return;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_time += elapsed;
            ++ count;
            is_running = false;

            std::cout << "[" << name << "] time: " << elapsed << " ms" << std::endl;
        }

        void reset()
        {
            total_time = 0.0;
            count = 0;
            is_running = false;
        }
    };

    TimerData& operator[](const std::string& timer_name)
    {
        for (auto& timer : timers_) {
            if (timer.name == timer_name) {
                return timer;
            }
        }
        timers_.emplace_back(timer_name);
        return timers_.back();
    }

    Timer(){}
    ~Timer()
    {
        if(timers_.size() > 0)
        {
            std::cout << "\n\033[1;33m================= Timer Summary =================\033[0m\n";
            double total_average_time = 0.0;
            for (const auto& timer : timers_) {
                if (timer.count > 0)
                {
                    std::cout << "  [" << timer.name << "] Average: " << (timer.total_time / timer.count) << " ms" << std::endl;
                    total_average_time += timer.total_time / timer.count;
                }
                else
                    std::cout << "  [" << timer.name << "] No recorded intervals" << std::endl;
            }
            std::cout << "  [total] Average: " << total_average_time << " ms" << std::endl;
            std::cout << "\033[1;33m=================================================\033[0m\n";
        }
    }

private:
    std::vector<TimerData> timers_;
};

inline void transformfromTFToEigen(const geometry_msgs::msg::TransformStamped& transform_stamped, Eigen::Isometry3d& transform)
{
    transform = Eigen::Isometry3d::Identity();

    Eigen::Vector3d t(transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z);
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);

    transform.translate(t);
    transform.rotate(q);


    return;
}

template <typename T>
class VectorElementGetter {
public:
    explicit VectorElementGetter(std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

template <typename T>
class constVectorElementGetter {
public:
    explicit constVectorElementGetter(const std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(const T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    const std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

inline void incrementIdx(Index& idx, const unsigned int& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx)
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx)
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

inline void incrementIdx(Index& idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

inline void incrementIdx(Index& idx, const Index& min_idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = min_idx.z();
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = min_idx.y();
            idx.x()++;
        }
    }
}

class Neighbours{
public:
    Neighbours(){}
    inline void set_params(unsigned int connectivity);
    std::vector<Index> offsets;
};

inline void Neighbours::set_params(unsigned int connectivity)
{
    offsets.reserve(26);
    offsets.emplace_back(Index(-1,  0,  0));
    offsets.emplace_back(Index( 1,  0,  0));
    offsets.emplace_back(Index( 0, -1,  0));
    offsets.emplace_back(Index( 0,  1,  0));
    offsets.emplace_back(Index( 0,  0, -1));
    offsets.emplace_back(Index( 0,  0,  1));
    offsets.emplace_back(Index(-1, -1,  0));
    offsets.emplace_back(Index(-1,  1,  0));
    offsets.emplace_back(Index( 1, -1,  0));
    offsets.emplace_back(Index( 1,  1,  0));
    offsets.emplace_back(Index( 0, -1, -1));
    offsets.emplace_back(Index( 0, -1,  1));
    offsets.emplace_back(Index( 0,  1, -1));
    offsets.emplace_back(Index( 0,  1,  1));
    offsets.emplace_back(Index(-1,  0, -1));
    offsets.emplace_back(Index( 1,  0, -1));
    offsets.emplace_back(Index(-1,  0,  1));
    offsets.emplace_back(Index( 1,  0,  1));
    offsets.emplace_back(Index(-1, -1, -1));
    offsets.emplace_back(Index(-1, -1,  1));
    offsets.emplace_back(Index(-1,  1, -1));
    offsets.emplace_back(Index(-1,  1,  1));
    offsets.emplace_back(Index( 1, -1, -1));
    offsets.emplace_back(Index( 1, -1,  1));
    offsets.emplace_back(Index( 1,  1, -1));
    offsets.emplace_back(Index( 1,  1,  1));

    if(connectivity == 6 || connectivity == 18 || connectivity == 26)
        offsets.resize(connectivity);
    else
        std::cout << "Connectivity not supproted" << std::endl;
}

class ProgressBar
{
public:
    ProgressBar(std::string prefix_, int bar_length_, int total_steps_, int skip_):
        prefix(prefix_),
        bar_length(bar_length_),
        total_steps(total_steps_),
        skip(skip_),
        end(total_steps_%skip_),
        current_step(0),
        start_time(rclcpp::Clock().now()),
        last_print_time(rclcpp::Clock().now()){}

    inline void step()
    {
        current_step++;

        if(!time_to_print())
            return;

        unsigned int cur_bar_length = (bar_length * current_step)/total_steps;
        std::string bar_string(cur_bar_length,'=');
        std::string empty_bar_string(bar_length - cur_bar_length,' ');

        double elapsed_seconds = (last_print_time - start_time).seconds();
        std::cout << std::left << std::setw(12) << prefix << "[" << bar_string << ">" << empty_bar_string << "]"
                 << (100 * current_step)/total_steps << "% " << std::fixed << std::setprecision(3)
                 << elapsed_seconds << "s\r";
        std::cout.flush();

        if(current_step == total_steps)
            std::cout << std::endl;
    }

    inline bool time_to_print()
    {
        rclcpp::Time now = rclcpp::Clock().now();

        if(current_step%skip == end || (now - last_print_time).seconds() > 0.0333)
        {
            last_print_time = now;
            return true;
        }

        return false;
    }

private:
    std::string prefix;
    unsigned int bar_length;
    unsigned int total_steps;
    unsigned int current_step;
    unsigned int skip;
    unsigned int end;

    rclcpp::Time start_time;
    rclcpp::Time last_print_time;
};

}
#endif //UTILS_H