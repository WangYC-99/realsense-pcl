# realsenseD435、D435i 获取点云数据获取并实时显示

**by WangYC**

cmake、opencv、boost配置略（ububtu18直接apt安装即可）

## 前言

本文利用pcl对rs获取的深度信息进行点云化并实时显示，整个过程耗时较多，仅对深度信息进行处理生成点云0.6fps。

除去pcl外，也可利用OpenGL进行上述操作（官方样例中即采用此方式）

## 1 realsense开发环境配置

```shell
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

ps:以上仅为rs开发环境搭建，如需要请到GH下载官方sdk：

https://github.com/IntelRealSense/librealsense

在/librealsense/wrappers/pcl/中有现成的显示一帧率的demo，采用OpenGL与pcl结合的思路。

## 2 pcl安装

point cloud library

### 2.1 依赖安装

```shell
  sudo apt-get update
  sudo apt-get install git build-essential linux-libc-dev
  sudo apt-get install cmake cmake-gui 
  sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
  sudo apt-get install mpi-default-dev openmpi-bin openmpi-common  
  sudo apt-get install libflann1.8 libflann-dev
  sudo apt-get install libeigen3-dev
  sudo apt-get install libboost-all-dev
  sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
  sudo apt-get install libqhull* libgtest-dev
  sudo apt-get install freeglut3-dev pkg-config
  sudo apt-get install libxmu-dev libxi-dev 
  sudo apt-get install mono-complete
  sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
```

### 2.2 pcl编译安装

```shell
git clone https://github.com/PointCloudLibrary/pcl.git
```

如果网络原因受限可以挂梯子下载zip再解压是一样的。

解压后进入目录，创建release文件夹并且cmake。

（ps：文件夹的名字必须是release）

```shell
cd pcl
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
-DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \
-DCMAKE_INSTALL_PREFIX=/usr ..
```

```shell
make -j2
```

过程较为长久，着急的话建议先装着下面的，不着急的话可以休息一会。

```shell
sudo make install
```

### 2.3测试效果

pcl_test.cpp

```cpp
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
 
 
int main(int argc, char **argv) {
    std::cout << "Test PCL !!!" << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
      for (float angle(0.0); angle <= 360.0; angle += 5.0)
      {
	pcl::PointXYZRGB point;
	point.x = 0.5 * cosf (pcl::deg2rad(angle));
	point.y = sinf (pcl::deg2rad(angle));
	point.z = z;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
		static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point.rgb = *reinterpret_cast<float*>(&rgb);
	point_cloud_ptr->points.push_back (point);
      }
      if (z < 0.0)
      {
	r -= 12;
	g += 12;
      }
      else
      {
	g -= 12;
	b += 12;
      }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(point_cloud_ptr);
    while (!viewer.wasStopped()){ };
    return 0;
}
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.6)
project(pcl_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test pcl_test.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)
```

编译测试

将cpp文件和cmakelists放在同一文件夹

```shell
cmake .
make
./pcl_test
```

能看到圆环是正常。

如果一时看不到就动动鼠标。

## 3 OpenGL安装

open graphcs library

### 3.1 apt安装

```shell
$ sudo apt-get install build-essential
$ sudo apt-get install libgl1-mesa-dev
$ sudo apt-get install libglu1-mesa-dev
$ sudo apt-get install  freeglut3-dev
```

### 3.2 测试安装效果

#### test.c

```c
#include <GL/glut.h>


void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glOrtho(-5, 5, -5, 5, 5, 15);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);

    return;
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1.0, 0, 0);
    glutWireTeapot(3);
    glFlush();

    return;
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(300, 300);
    glutCreateWindow("OpenGL 3D View");
    init();
    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}
```

#### 编译执行

```shell
$ gcc -o test test.c -lGL -lGLU -lglut
$ ./test
```

弹出窗口显示茶壶说明正常。

## 4 GLFW安装

### 4.1 安装依赖

```shell
$ sudo apt-get build-dep glfw
$ sudo apt-get install cmake xorg-dev libglu1-mesa-dev 
```

### 4.2 下载lib并编译安装

#### 4.2.1 下载

www.glfw.org

#### 4.2.2 编译安装

进入 glfw3-3.x.x 目录，建立glfw-build子目录

```shell
$ sudo mkdir glfw-build
$ sudo cmake ../
$ sudo make && sudo make install
```

## 5 利用pcl对点云进行实时显示原理

### 5.1 打开rs数据管道

```cpp
rs2::pipeline pipe;
pipe.start();
auto frames = pipe.wait_for_frames();
```

### 5.2 定义点云并利用深度管道为点云赋值

```cpp
auto depth = frames.get_depth_frame();
rs2::pointcloud pc;
rs2::points points;
points = pc.calculate(depth);
```

### 5.3 将rs点云转化为pcl点云

```cpp
pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}
```

### 5.4 显示点云

```cpp
window app(1280, 720, "RealSense PCL Pointcloud Example");
state app_state;
register_glfw_callbacks(app, app_state);
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
viewer.showCloud (pcl_points);
```

## 6 代码实例

https://gitee.com/wang-yuanchun/realsense_pcl

## 7 仓库使用

```shell
$ mkdir build && cd build
$ cmake ..
$ make -j
```

