#!/bin/bash
# 守护进程方式启动 run_fs.sh : nohup ./run_fs.sh &
# 注意:
# 1. run_fs.sh 脚本 与 app_fs 程序在相同的目录下,否则报错退出
# 2. 操作系统重启后,需要重启 run_fs.sh,或将脚本放入大包中
# 3. run_fs.sh 的运行日志记录在 run_fs.log 中,日志格式: 2022-08-22 11:35:51 restart process [nohup ./send &] succ. 每隔5分钟打印一行
set -e

# init env and kill other run_fs.sh process
export LD_LIBRARY_PATH=/opt/ros/uto/pilot:/opt/ros/uto/pilot/uto_common_base/lib:/opt/ros/uto/pilot/uto_common_proto/lib:/opt/ros/uto/pilot/uto/lib:/usr/lib/aarch64-linux-gnu:/mnt/pegasus/YSG_Version/lib:/opt/ros/eloquent/lib:$LD_LIBRARY_PATH
selfpid=$$
other_pids=$(ps -ef | grep run_fs.sh | grep -v $selfpid | grep -v grep | awk -F' ' '{print $2}')
if [ -n "$other_pids" ]; then
  kill -9 $other_pids
fi

timestamp=$(date "+%Y-%m-%d_%H_%M_%S")
log_file="/home/nvidia/.ros/log/run_fs_$timestamp.log"
echo "$timestamp" >>$log_file

cd /mnt/pegasus/AIV_iecu2.0_Release/fusion_node/
if [ ! -f app_fs ]; then
  echo "Error: cannot access 'app_fs': No such file or directory" >>$log_file
  echo "mv run_fs.sh to 'send' file's dir" >>$log_file
  exit 1
fi

PROJECT_NAME=$(cat /mnt/pegasus/aiv_project_info.json | python2 -c "import json; import sys; obj=json.load(sys.stdin); print (obj['project_name'].encode('utf-8'))")
AIV_CAR_NUM=$(cat /mnt/pegasus/aiv_project_info.json | python2 -c "import json; import sys; obj=json.load(sys.stdin); print (obj['aiv_num'].encode('utf-8'))")
while [ 1 ]; do
  pid_no=$(ps -A | grep app_fs | awk '{print $1}')
  if [[ $pid_no == "" ]]; then
    timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    cd /mnt/pegasus/AIV_iecu2.0_Release/fusion_node/
    nohup ./app_fs --ros-args --log-level WARN -p vehicle_num:=$AIV_CAR_NUM >/dev/null 2>&1 &
    echo "$timestamp restart process [nohup /mnt/pegasus/AIV_iecu2.0_Release/fusion_node/app_fs &] succ." >>$log_file
  fi
  sleep 10
done
