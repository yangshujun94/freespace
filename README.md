# freespace-fusion

# file
* launch   : uto_per_fs node execution configure
* param    : uto_per_fs node startup configure
* src      : code file
* unit_test: unit test file
* fs.json  : foxglove configure
* fs.rviz  : rviz2 configure

# script
* build.sh <br>
  本地编译脚本，支持Debug和Release(默认)模式，支持分项目编译，支持打开可视化和打开load_log功能<br>
* clean_project.sh <br>
  清除工程目录下build/ install/ log/ 文件夹 <br>
* run.sh <br>
  本地运行脚本，需传入项目位置 <br>
* run_unittest.sh <br>
  本地运行单元测试脚本 <br>

# Release Notes
We have the following change categories:
* [Improvements]: new features, new algorithms
* [Fixes]: fix coding mistakes
* [Refactoring]: coding style, non-functional changes
* [Testing]: test cases

## WIP
---

## 4.0.0
### Improvements
* 全量诊断功能
* 故障降级模式
* 一定区域内透传
* 鱼眼过滤
* 闸机
* 路沿
* OD动静态
* 输出时提取边界
* 实现sub/pub
* 输出范围限制
* map sdk适配 + 电子围栏
### Testing
* 修复单元测试编译依赖缺少问题
* 增加单元测试编译运行脚本
* 更新本地debug使用的编译和运行脚本
