在 T1（Ubuntu）上把 ROS2 的相机 topic 用 web_video_server 变成 HTTP 视频流，Mac 直接用浏览器打开就像监控一样看。

目标：三个画面（彩色/深度/对齐深度）
- /camera/camera/color/image_raw
- /camera/camera/depth/image_rect_colorized
- /camera/camera/aligned_depth_to_color/image_colorized

0）启动相机（RealSense，使用 realsense-ros）
推荐：通过 realsense-ros 获取数据
https://github.com/IntelRealSense/realsense-ros

注意：T1 开机后会通过系统服务 booster-daemon-perception.service 自动启动 RealSense ROS 节点。
如果需要手动启动相机节点，请先关闭自动服务，避免资源占用冲突：
sudo systemctl stop booster-daemon-perception.service

手动启动（示例路径）：
cd ~/ThirdParty/realsense-ros/
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

自动启动 RealSense ROS 节点发布的 topic 如下（节选）：
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/extrinsics/depth_to_color
/camera/camera/extrinsics/depth_to_depth

1）启动 web_video_server（浏览器监控）
sudo apt install -y ros-$ROS_DISTRO-web-video-server
ros2 run web_video_server web_video_server --ros-args -p port:=8080 -p address:=0.0.0.0

2）深度可视化（web_video_server 不支持 16UC1 原始深度）
深度/对齐深度原始 topic 是 16UC1，web_video_server 会报 cv_bridge 错误，需要先转成可视化 BGR8。
source /opt/ros/$ROS_DISTRO/setup.bash
python3 /home/booster/Workspace/booster_robot/Camera/depth_colorizer.py --ros-args -p input_topic:=/camera/camera/depth/image_rect_raw -p output_topic:=/camera/camera/depth/image_rect_colorized
python3 /home/booster/Workspace/booster_robot/Camera/depth_colorizer.py --ros-args -p input_topic:=/camera/camera/aligned_depth_to_color/image_raw -p output_topic:=/camera/camera/aligned_depth_to_color/image_colorized

3）启动自定义 webview（同屏三画面）
cd /home/booster/Workspace/booster_robot/Camera/webview
./serve.sh
（或 python3 -m http.server 8090 --bind 0.0.0.0）

4）Mac 浏览器打开
先在 T1 上查 IP：
ip a

三画面（自定义 webview）：
http://T1_IP:8090/

单独看彩色视频：
http://T1_IP:8080/stream_viewer?topic=/camera/camera/color/image_raw

单独看深度图：
http://T1_IP:8080/stream_viewer?topic=/camera/camera/depth/image_rect_colorized

单独看对齐深度：
http://T1_IP:8080/stream_viewer?topic=/camera/camera/aligned_depth_to_color/image_colorized

5）想更流畅：降分辨率/压缩质量（Wi-Fi 很关键）
http://T1_IP:8080/stream_viewer?topic=/camera/camera/color/image_raw&width=640&quality=70
这些 URL 参数（width/height/quality/bitrate 等）是 web_video_server 支持的。

6）常见坑
看不到画面但 topic 明明存在：可能是 ROS2 QoS 不匹配，试试在 URL 上加：
&qos_profile=sensor_data

深度图报错或黑屏：直接看 raw 深度会出现 16UC1 的 cv_bridge 错误，必须使用 colorized topic。
自定义 webview 默认走 colorized；需要改 topic 时可以加参数：
http://T1_IP:8090/?depth_topic=/camera/camera/depth/image_rect_raw&aligned_topic=/camera/camera/aligned_depth_to_color/image_raw

Mac 打不开 8080/8090：检查 T1 防火墙（如 ufw）是否放行 8080/tcp 和 8090/tcp。
