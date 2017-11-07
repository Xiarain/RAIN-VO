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

$mkdir build & cd build

$cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC ..

$ make & sudo make install

$ git clone https://github.com/google/glog.git

$ cd glog

$ mkdir build && cd build

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