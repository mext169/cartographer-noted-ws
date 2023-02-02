# 0 装ROS

参考第一个链接，快速安装；参考第二个链接，解决rosdep问题。

[详细介绍如何在ubuntu20.04中安装ROS系统，超快完成安装（最新版教程）](https://blog.csdn.net/qq_44339029/article/details/120579608?spm=1001.2014.3001.5506)

[rosdep init/update 解决方法](https://blog.csdn.net/weixin_43311920/article/details/114796748?spm=1001.2014.3001.5506)

# 1 安装基本工具

```
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
sudo apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libceres-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    python3-sphinx \
    stow
```

下面这个库可以解决编译中遇到的*_test的错误

```
sudo apt install libgmock-dev
```

# 2 安装abseil-cpp

```
# git clone https://github.com/abseil/abseil-cpp.git
git clone https://gitee.com/run_zchenglong/abseil-cpp.git
git checkout 215105818dfde3174fe799600bb0f3cae233d0bf
mkdir build && cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl
```

# 3 安装protobuf

```
# Build and install proto3.
# git clone https://github.com/google/protobuf.git
git clone https://gitee.com/wowuming/protobuf.git
cd protobuf
git checkout tags/v3.4.1
mkdir build && cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install
```

# 4 build cartographer

```
cd catkin_ws
mkdir build && cd build
cmake .. -DCATKIN_DEVEL_PREFIX=../devel
make
```

# 5 clion打开

- `File | Settings... | Build, Execution, Deployment | Cmake`


- `Build type行选择 RelWithDebinfo 模式(为以后调试我们的carto代码做准备,如果不调试,平时你也可选择release)`


- `设置devel文件夹路径。在CMake options行输入：`

    ```
    -DCATKIN_DEVEL_PREFIX:PATH=/home/mext/code/clion_carto_ws/devel
    ```

- `设置build文件夹路径。在Generation path行输入：`

  ```
  /home/mext/code/clion_carto_ws/build
  ```

