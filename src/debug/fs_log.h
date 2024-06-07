#ifndef FS_LOG_H_
#define FS_LOG_H_

#include <common/log.h>
#include "peripheral/defs.h"

#define LOG_ERROR(fmt, arg...) RCLCPP_ERROR(rclcpp::get_logger(fs::NODE_NAME), fmt, ##arg)
#define LOG_WARN(fmt, arg...)  RCLCPP_WARN(rclcpp::get_logger(fs::NODE_NAME), fmt, ##arg)
#define LOG_INFO(fmt, arg...)  RCLCPP_INFO(rclcpp::get_logger(fs::NODE_NAME), fmt, ##arg)
#define LOG_DEBUG(fmt, arg...) RCLCPP_DEBUG(rclcpp::get_logger(fs::NODE_NAME), fmt, ##arg)

#endif //FS_LOG_H_
