#ifndef UTILS_H_
#define UTILS_H_

#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <common/subscriber.h>

namespace fs
{
  class Utils
  {
  public:
    template<typename T>
    static std::unique_ptr<T> deserializeRosMessage(const rosbag2_storage::SerializedBagMessage& serializedBagMsg)
    {
      auto                     serializedMsg = rclcpp::SerializedMessage(*serializedBagMsg.serialized_data);
      auto                     msgPtr        = std::make_unique<T>();
      rclcpp::Serialization<T> serialization;
      serialization.deserialize_message(&serializedMsg, msgPtr.get());

      return msgPtr;
    }

    template<typename T>
    static std::unique_ptr<T> deserializeIdlMessage(const rosbag2_storage::SerializedBagMessage& serializedBagMsg)
    {
      auto                                             serializedMsg = rclcpp::SerializedMessage(*serializedBagMsg.serialized_data);
      uto::idl::SerializedProto                        idl;
      rclcpp::Serialization<uto::idl::SerializedProto> serialization;
      serialization.deserialize_message(&serializedMsg, &idl);

      auto msgPtr = std::make_unique<T>();
      msgPtr->ParseFromArray(idl.data.data(), idl.data.size());

      return msgPtr;
    }

    static bool isFisheye(const SensorId sensorId)
    {
#if FS_CHECK(CFG_USE_FISHEYE_FS)
      return SensorId::FISHEYE_FCF == sensorId ||
             SensorId::FISHEYE_FLL == sensorId ||
             SensorId::FISHEYE_FRR == sensorId ||
             SensorId::FISHEYE_BCB == sensorId ||
             SensorId::FISHEYE_BLL == sensorId ||
             SensorId::FISHEYE_BRR == sensorId;
#else
      return false;
#endif
    }

    static std::array<EVector2, 4> createCuboid(const float xCenter,
                                                const float yCenter,
                                                const float length,
                                                const float width,
                                                const float orientation)
    {
      std::array<EVector3, 4> shapePoints;
      shapePoints[ShapePointIndex::RL].x() = -0.5f * length;
      shapePoints[ShapePointIndex::RL].y() = 0.5f * width;
      shapePoints[ShapePointIndex::RL].z() = 1.f;
      shapePoints[ShapePointIndex::FL].x() = 0.5f * length;
      shapePoints[ShapePointIndex::FL].y() = 0.5f * width;
      shapePoints[ShapePointIndex::FL].z() = 1.f;
      shapePoints[ShapePointIndex::FR].x() = 0.5f * length;
      shapePoints[ShapePointIndex::FR].y() = -0.5f * width;
      shapePoints[ShapePointIndex::FR].z() = 1.f;
      shapePoints[ShapePointIndex::RR].x() = -0.5f * length;
      shapePoints[ShapePointIndex::RR].y() = -0.5f * width;
      shapePoints[ShapePointIndex::RR].z() = 1.f;

      const float cos = std::cos(orientation);
      const float sin = std::sin(orientation);

      Eigen::Matrix<float, 2, 3> transform;
      transform << cos, -sin, xCenter,
        sin, cos, yCenter;

      std::array<EVector2, 4> cuboid;
      for(int i = 0; i < 4; ++i)
      {
        cuboid[i] = transform * shapePoints[i];
      }

      return cuboid;
    }
  };
} // namespace fs

#endif //UTILS_H_
