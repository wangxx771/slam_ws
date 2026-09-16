#pragma once
// glog-backed logging shared by the workspace.
//
// The call shape of the RCLCPP_* macros is kept so migration is a rename: the
// logger argument is accepted and left unevaluated. glog is used instead of the
// ROS backend because it writes severity-separated files whose names carry the
// date, keeps current-run symlinks and caps file size, where the ROS logs were
// one flat directory per run under ~/.ros/log with no date in the name.
#include <glog/logging.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace slam_log
{

inline std::string fmt(const char * f, ...)
{
  va_list ap;
  va_start(ap, f);
  va_list ap2;
  va_copy(ap2, ap);
  const int n = vsnprintf(nullptr, 0, f, ap);
  va_end(ap);
  if (n <= 0) {va_end(ap2); return f;}
  std::string s(static_cast<size_t>(n) + 1, '\0');
  vsnprintf(&s[0], s.size(), f, ap2);
  va_end(ap2);
  s.resize(static_cast<size_t>(n));
  return s;
}

/// True at most once per period_ms for this call site.
inline bool throttle(std::chrono::steady_clock::time_point & last, double period_ms)
{
  const auto now = std::chrono::steady_clock::now();
  if (last.time_since_epoch().count() != 0 &&
      std::chrono::duration<double, std::milli>(now - last).count() < period_ms)
  {
    return false;
  }
  last = now;
  return true;
}

/// Create <base>/<date> and point glog at it. Returns the directory used, which
/// falls back to ~/.ros/log when base is not writable.
std::string init(const std::string & base, const char * argv0);

}  // namespace slam_log

// sizeof keeps the logger argument unevaluated, so call sites that build one on
// the fly (node->get_logger()) cost nothing.
#define SLAM_LOG_AT(sev, logger, ...) \
  do { \
    (void)sizeof(logger); \
    LOG(sev) << ::slam_log::fmt(__VA_ARGS__); \
  } while (0)

#define PL_INFO(logger, ...)  SLAM_LOG_AT(INFO, logger, __VA_ARGS__)
#define PL_WARN(logger, ...)  SLAM_LOG_AT(WARNING, logger, __VA_ARGS__)
#define PL_ERROR(logger, ...) SLAM_LOG_AT(ERROR, logger, __VA_ARGS__)
#define PL_FATAL(logger, ...) SLAM_LOG_AT(ERROR, logger, __VA_ARGS__)
#define PL_DEBUG(logger, ...) \
  do { (void)sizeof(logger); VLOG(1) << ::slam_log::fmt(__VA_ARGS__); } while (0)
#define PL_INFO_STREAM(logger, expr) \
  do { (void)sizeof(logger); LOG(INFO) << expr; } while (0)

// The clock and duration arguments of the ROS throttle macros are accepted and
// ignored; the period is taken from the millisecond argument, as in ROS.
#define SLAM_LOG_THROTTLE(sev, logger, clock, ms, ...) \
  do { \
    (void)sizeof(logger); \
    (void)sizeof(clock); \
    static std::chrono::steady_clock::time_point _slam_log_last{}; \
    if (::slam_log::throttle(_slam_log_last, static_cast<double>(ms))) { \
      LOG(sev) << ::slam_log::fmt(__VA_ARGS__); \
    } \
  } while (0)

#define PL_INFO_THROTTLE(logger, clock, ms, ...) \
  SLAM_LOG_THROTTLE(INFO, logger, clock, ms, __VA_ARGS__)
#define PL_WARN_THROTTLE(logger, clock, ms, ...) \
  SLAM_LOG_THROTTLE(WARNING, logger, clock, ms, __VA_ARGS__)
#define PL_ERROR_THROTTLE(logger, clock, ms, ...) \
  SLAM_LOG_THROTTLE(ERROR, logger, clock, ms, __VA_ARGS__)
#define PL_DEBUG_THROTTLE(logger, clock, ms, ...) \
  SLAM_LOG_THROTTLE(INFO, logger, clock, ms, __VA_ARGS__)
