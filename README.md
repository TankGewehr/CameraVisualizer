# Visualizer

用于障碍物检测结果以及车道线检测结果可视化的工具，使用ros消息同步器来同步检测结果与对应的图像或激光点云的时间戳，从而使可视化结果更准确，但也降低了可视化结果的流畅性

#
## 软件环境

* ros noetic
* OpenCV 4.2.0

#
## 目录结构

```
Visualizer
├─.vscode           vscode配置文件，可供debug时参考
├─app               应用程序源文件
├─config            应用程序配置文件以及传感器标定参数文件
├─include           头文件
├─lib               库文件
├─script            一些方便使用的脚本，可供使用时参考
├─src               源文件
├─.gitignore        gitignore配置文件
├─CMakeLists.txt    CmakeLists配置文件
└─README.md         使用及说明文档
```

#
## 编译

```shell
cd {Visualizer_path}
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

完成编译后生成的可执行文件位于`{Visualizer_path}/bin`目录下，库文件位于`{Visualizer_path}/lib`目录下

## 仅编译可执行文件

如果没有源代码并且有已经编译过的库文件，可以使用以下方法编译可执行文件

```shell
cd {Visualizer_path}
mkdir build
cd build
cmake ..
make
```

#
## 参数说明

```
标定参数文件，可以参考{Visualizer_path}/config/sensors/cam_front_top.json
{
   "channel" : 传感器对应的rostopic,
   "modality" : 传感器自身类型,
   "image_size" : 图像大小 [w,h]，其中w代表图像的宽，h代表图像的高,
   "intrinsic" : 内参矩阵 [f/dx,skew,u0,0,f/dy,v0,0,0,1]，其中fx、fy代表使用像素来描述x轴、y轴方向焦距的长度，u0、v0代表光心（相机坐标系原点）在像素坐标系的像素坐标，skew代表扭曲参数,
   "distortion" : 畸变参数 [k1,k2,p1,p2[,k3]]，其中k代表径向畸变，p代表切向畸变,
   "undistort_intrinsic" : 去畸变后的图像再次标定的内参矩阵 [f/dx,skew,u0,0,f/dy,v0,0,0,1],
   "undistort_distortion" : 去畸变后的图像再次标定的畸变参数 [k1,k2,p1,p2[,k3]]，其中k代表径向畸变，p代表切向畸变,
   "target" : 平移和旋转相对的目标,
   "rotation" : 旋转矩阵，表示该传感器在目标的坐标系下的旋转,
   "translation" : 平移向量，表示该传感器在目标的坐标系下的平移 [x,y,z]
}
```

```
应用程序配置文件，可以参考{Visualizer_path}/config/config.json
{
    "calibration_params_path":{
        "cam_front_left": 左前相机标定参数文件路径,
        "cam_front": 前视相机标定参数文件路径,
        "cam_front_right": 右前相机标定参数文件路径,
        "cam_back_left": 左后相机标定参数文件路径,
        "cam_back": 后视相机标定参数文件路径,
        "cam_back_right": 右后相机标定参数文件路径,
        "lidar_top": 激光雷达标定参数文件路径
    },
    "obstacle_enable": 障碍物检测结果可视化开关,
    "obstacle_list_topic": 障碍物检测结果rostopic,
    "lane_enable": 车道线检测结果可视化开关,
    "lane_list_topic": 车道线检测结果rostopic,
    "frame_id": 可视化结果frame_id,
    "publish_topic": 可视化结果rostopic,
}
```

#
## 使用教程

主要分为环境配置、运行、可视化结果三个部分

## 1、环境配置

为了能够接受到各个传感器的数据以及检测结果，需要在PC上配置其他设备的IP、hostname和ROS_MASTER_URI

``` shell
sudo vi /etc/hosts
```

在打开的文件末尾换行后添加以下内容，如果有多个设备则重复此步骤

```
{其他设备的IP} {其他设备的hostname}
```

配置ROS_MASTER_URI，对于每一个新开启的终端都需要重新配置

``` shell
export ROS_MASTER_URI=http://{启动roscore设备的IP}:11311
```

## 2、运行

```shell
cd {Visualizer_path}
./bin/visualizer ./config/config.json
```

## 3、可视化结果

``` shell
rviz
```

依次点击rviz窗口左下角的`Add`、弹出窗口的`By topic`、`可视化结果rostopic`下的`MarkerArray`、`PointCloud2`、`Image`后，修改rviz左上角的`Displays`下`Global Options`中`Fixed Frame`为`可视化结果frame_id`，即可在rviz中观察到可视化结果，可以手动调整图像位置、点云视角以及点云中点的大小方便观察（修改rviz左上角的`Displays`下`PointCloud2`中`Size (m)`为`0.1`）