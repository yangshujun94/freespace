1. 安装foxglove studio				

   ```bash
   sudo apt install ros-galactic-foxglove*
   ```

   

2. 在docker环境中进行一下操作安装相应的组件

   ```bash
   sudo apt install ros-galactic-rosbridge-suite
   sudo apt install ros-$ROS_DISTRO-foxglove-bridge
   ```

   

3. 然后需要source一下

   ```bash
   source /opt/ros/galactic/setup.bash
   ```

   

4. 接着我们需要启动相应的foxglove的通道节点

   ```bash
   ros2 run foxglove_bridge foxglove_bridge
   ```

   

5. 在本地打开foxglove软件，依次点击==Open connection==，==Foxglove WebSocket==，最后找到==LAYOUT==并选择==import from file==找到freespace.json加载，即可

6. 再开一个节点，设置一个tf变换，方便可视化

   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 vehicle ego
   ```

   

