#include "slam_log/log.h"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace slam_log
{

std::string init(const std::string & base, const char * argv0)
{
  const std::time_t now = std::time(nullptr);
  std::tm tmv{};
  localtime_r(&now, &tmv);
  char day[16];
  std::strftime(day, sizeof(day), "%Y-%m-%d", &tmv);

  std::filesystem::path dir = std::filesystem::path(base) / day;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  // /var/log is not writable for an unprivileged user; say so and fall back
  // rather than losing the logs silently.
  const std::filesystem::path probe = dir / ".write_test";
  bool ok = !ec || std::filesystem::is_directory(dir);
  if (ok) {
    std::ofstream f(probe);
    ok = static_cast<bool>(f);
  }
  if (!ok) {
    const char * home = std::getenv("HOME");
    dir = std::filesystem::path(home ? home : "/tmp") / ".ros" / "log" / day;
    std::filesystem::create_directories(dir, ec);
    std::cerr << "[log] " << base << " not writable, falling back to " << dir << std::endl;
  } else {
    std::filesystem::remove(probe, ec);
  }

  FLAGS_log_dir = dir.string();
  FLAGS_alsologtostderr = true;   // keep the console output people watch
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;           // flush immediately; a crash must not eat the tail
  FLAGS_max_log_size = 64;        // MB per file
  FLAGS_stop_logging_if_full_disk = true;
  google::InitGoogleLogging(argv0);
  // No InstallFailureSignalHandler: glog's handler also traps SIGTERM, so every
  // normal Ctrl-C printed an "*** Aborted ***" stack trace that reads as a crash.
  return dir.string();
}

}  // namespace slam_log
