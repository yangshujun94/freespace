#include "macro.h"
#include "switcher.h"

#if FS_CHECK(CFG_MULTITHREAD)
#include "fs_node_multithread.h"
#else
#include "fs_node_singlethread.h"
#endif

#if FS_CHECK(CFG_ROS2)
#include <csignal>
#include <vector>
#include <common/log.h>
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

// clang-format off
void handler(int signo, siginfo_t* info, void* context)
{
  static_cast<void>(info);
  static_cast<void>(context);

  // print stack
  UWARN << "signal handler, stopping." << signo;
  UWARN << "Backtrace:" << std::endl << boost::stacktrace::stacktrace();

  // exit
  raise(SIGINT);
}

void capture_signals()
{
  struct sigaction act{};
  act.sa_flags = SA_SIGINFO;
  act.sa_sigaction = handler;
  sigemptyset(&act.sa_mask);

  // load all signals which can create core dumps
  std::vector<int> signals{SIGILL, SIGTRAP, SIGABRT, SIGBUS, SIGFPE, SIGSEGV};
  for(int signal : signals)
  {
    if(sigaction(signal, &act, nullptr) == -1)
    {
      perror("sigaction");
      exit(EXIT_FAILURE);
    }
  }
}
#endif
// clang-format on

int main(int argc, char** argv)
{
#if FS_CHECK(CFG_ROS2)
  try
  {
    rclcpp::init(argc, argv);

#if FS_CHECK(CFG_MULTITHREAD)
    rclcpp::executors::MultiThreadedExecutor executor;
#else
    rclcpp::executors::StaticSingleThreadedExecutor executor;
#endif

//        capture_signals();

    auto node = std::make_shared<fs::FsNode>();

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
  }

  catch(const rclcpp::exceptions::InvalidQosOverridesException& e)
  {
    UERROR << "Caught an InvalidQosOverridesException: " << e.what() << std::endl;
    return 1; // 或其他适当的错误代码
  }
#else
  rclcpp::init(argc, argv);
#ifdef IECU_2_0
  rclcpp::executors::SingleThreadedExecutor executor;
#else
#if FS_CHECK(CFG_MULTITHREAD)
  rclcpp::executors::MultiThreadedExecutor executor;
#else
  rclcpp::executors::StaticSingleThreadedExecutor executor;
#endif
#endif

  auto node     = std::make_shared<fs::FsNode>();

#if !FS_CHECK(CFG_LOAD_RECORDING)
  auto thread0  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_DEFAULT, node);
  auto thread1  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_TRANSFORM, node);
  auto thread3  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_LIDAR_GRIDMAP, node);
  auto thread4  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_VCU_VEHICLE_INFO, node);
  auto thread5  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Planner_Gridmap_AIV, node);
  auto thread6  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_NORMAL_CAMERA_FS, node);
  auto thread7  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_LANE_CONTROL, node);
  auto thread8  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_VOT, node);
  auto thread9  = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_BEV_FS, node);
  auto thread10 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_FW, node);
  auto thread11 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_RW, node);
  auto thread12 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_FL, node);
  auto thread13 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_FR, node);
  auto thread14 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_RL, node);
  auto thread15 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_Image_RR, node);
  auto thread16 = std::async(std::launch::async, &fs::FsNode::lcmReceiverThread_LANE, node);
#endif

  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
#endif

  return 0;
}