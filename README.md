# RAIN_VIO

## 1.Description
The vision odometry

## 2.Prerequisites

### OpenCV
Dowload and install instructions can be found at: http://opencv.org.

Tested with OpenCV 3.2.

### glog
```sh
download new version gflag and compile.

$ git clone https://github.com/gflags/gflags.git

$ mkdir build & cd build

$ cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC ..

$ make & sudo make install

$ git clone https://github.com/google/glog.git

$ cd glog

$ mkdir build && cd build

maybe you need to install cmake3

$ sudo apt-get install cmake3

$ export CXXFLAGS="-fPIC" && cmake .. && make VERBOSE=1

$ make

$ sudo make install
```
### Eigen
 Download and install instructions can be found at: http://eigen.tuxfamily.org.

 Tested with Eigen 3.2.1

### Ceres
Dowload and install instructions can be found at: http://www.ceres-solver.org/installation.html

## 3.Building
```sh
$ mkdir build

$ cd build

$ make -j2
```

## 4.Test
1. **test_glog** \
make the test of the google glog. You should create the new folder called log and if not the log file can
not be saved.

2. **test_pangolin** \
make the test of the Pangloin. In order to make the display more concise, use the multiimage display.
you should add the datasheet in the project and the test will read the image from the datasheet and you also can modify
the code to add the your image. But you should have better to add the image come from the datasheet.

## 5.FAQ

1. can not find the Eigen3Config.cmake
maybe you have two versions cmake, you could find the Eigen3Config.cmake in the lower version cmake.

```sh
$ sudo cp /usr/share/cmake-2.8/Modules/FindEigen3.cmake /usr/share/cmake-3.2/Modules/
```