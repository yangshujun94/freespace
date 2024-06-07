#include <csignal>
#include <vector>

#include "fs_node.h"
#include "debug/fs_log.h"

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
// clang-format on

int main(int argc, char** argv)
{
  try
  {
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    capture_signals();

    rclcpp::get_logger(fs::NODE_NAME).set_level(rclcpp::Logger::Level::Warn);

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

  return 0;
}