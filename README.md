<div align="center">
  <a href="https://github.com/VIS4ROB-lab/HyperSLAM">
    <img src="https://drive.google.com/uc?export=view&id=1UAFr3tepqKwdnTomhKaeI2eIag3HOISY" alt="" style="width: 150px;">
  </a>

<h2><em>Hyper</em>SLAM</h2>
  <p>
    Modular, open-source implementations of continuous-time simultaneous localization and mapping algorithms.
    <br />
    <a href="https://github.com/VIS4ROB-lab/HyperSLAM/issues">Report Issues or Request Features</a>
  </p>
</div>
<br />

## About

*Hyper*SLAM provides an extensive and modular software stack which targets the challenging
task of Continuous-Time Simultaneous Localization and Mapping (CTSLAM) in a principled manner, and
aims to evolve into an indispensable link between discrete- and continuous-time optimizations for
fusing sensory information from arbitrary sensor suites in real-time. The current release focuses
on continuous-time representations (*i.e.* B-Splines) and formulates the continuous-time SLAM
problem as an online, sliding window, Non-Linear-Least-Squares (NLLS) optimization based on the
[Ceres](http://ceres-solver.org/) solver.

***Note:*** Development on HyperSLAM-related repositories has been discontinued.
If you are interested in obtaining the original software stack related to our publication
"Continuous-Time Stereo-Inertial Odometry" (provided as is), please feel free to contact us.

If you use the *Hyper*SLAM ecosystem, please cite it as below.

```
@article{RAL2022Hug,
    author={Hug, David and B\"anninger, Philipp and Alzugaray, Ignacio and Chli, Margarita},
    journal={IEEE Robotics and Automation Letters},
    title={Continuous-Time Stereo-Inertial Odometry},
    year={2022},
    volume={7},
    number={3},
    pages={6455-6462},
    doi={10.1109/LRA.2022.3173705}
}
```

## Setup & Installation

### Preliminaries

This framework relies on relatively new features in the C++ standard and, hence, relies on recent [GNU
(GCC/G++)](https://gcc.gnu.org/) compilers as well as novel language standards defined in
[C++17](https://en.cppreference.com/w/cpp/17) and [C++20](https://en.cppreference.com/w/cpp/20). Furthermore, the main
development and testing was done on
[Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/) (Focal Fossa) with
[ROS Noetic Ninjemys](http://wiki.ros.org/noetic), which can be set up as explained below.

* Install [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/) (Focal Fossa) or
  [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/) (Bionic Beaver).

* Install CMake and [GNU (GCC/G++)](https://gcc.gnu.org/) alternatives.

    ```
    # Install essentials.
    sudo apt update
    sudo apt install build-essential cmake git
    
    # Remove alternatives.
    sudo update-alternatives --remove-all gcc 
    sudo update-alternatives --remove-all g++
    
    # Install compilers.
    sudo apt install gcc-10 g++-10 gcc-9 g++-9
    
    # Set alternatives.
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 10
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 20
    sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
    
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 10
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 20
    sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
    
    sudo update-alternatives --set cc /usr/bin/gcc
    sudo update-alternatives --set c++ /usr/bin/g++
    
    # Configure alternatives.
    sudo update-alternatives --config gcc
    sudo update-alternatives --config g++
    ```

* Install [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) (20.04) or
  [ROS Melodic Morenia](http://wiki.ros.org/melodic) (18.04), depending on the selected Ubuntu version, by following the
  official [installation documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) or the outlined steps below.

    ```
    # Setup sources.
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros-latest.list'
    
    # Setup keys.
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
    --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    # Update index.
    sudo apt update
    
    # Install ROS.
    sudo apt install ros-noetic-desktop-full
    
    # Source the installation.
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
    echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
    source ~/.zshrc
    ```

### Dependencies

Aside from the previous requirements, *Hyper*SLAM also exposes several, additional dependencies:
[BLAS](http://www.netlib.org/blas/),
[LAPACK](http://www.netlib.org/lapack/),
[gflags](https://github.com/gflags/gflags) ([installation from source](https://gflags.github.io/gflags/)),
[glog](https://github.com/google/glog),
[gtest](https://github.com/google/googletest) ([installation from source](https://github.com/google/googletest/blob/master/googletest/README.md)),
[gbench](https://github.com/google/benchmark) ([installation from source](https://github.com/google/benchmark/blob/master/README.md)), [Boost](https://www.boost.org/), [Eigen](https://eigen.tuxfamily.org/dox/),
[OpenCV](https://opencv.org/) ([installation from source](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
or with additional modules), [Ceres Solver 2.0](http://ceres-solver.org/installation.html) (or higher versions)
and [yaml-cpp](https://github.com/jbeder/yaml-cpp), which are installed as follows. Note, this step may become obsolete
in the future with the introduction of an automated installation.

```
# Install glog.
# Note: MUST be installed from source!

# Install packages.
sudo apt install libatlas-base-dev libgflags-dev \
googletest libbenchmark-dev libboost-all-dev libeigen3-dev libopencv-dev \
libsuitesparse-dev libceres-dev libyaml-cpp-dev

# Install the Cerese solver.
# Note: MUST be installed from source (with version 2.0 or higher)!

```

### IDE Setup (optional)

We recommend using [CLion](https://www.jetbrains.com/clion/) as the IDE of choice. Having followed
[this](https://www.jetbrains.com/help/clion/installation-guide.html) installation guide, and having cloned and loaded
the *Hyper*SLAM repository, two additional steps are required to correctly source the ROS installation. Specifically,
`CMAKE_PREFIX_PATH=/opt/ros/noetic/share/` must be added to the environment variables under `Setting > Build, Execution,
Deployment > CMake`. A similar step is required to use the shared ROS libraries by adding
`LD_LIBRARY_PATH=/opt/ros/noetic/lib` to the runtime environment.

## Build & Run

All executables are written to the [bin](bin) folder, and the two flags *HYPER_ENABLE_TESTING*
and *HYPER_ENABLE_BENCHMARKING* are used to control whether the testing and benchmarking executables are compiled
alongside the main executable.

```
# Configuration.
mkdir build && cd build
cmake ..

# Compilation
make -j8 (-DHYPER_ENABLE_TESTING -DHYPER_ENABLE_BENCHMARKING)
```

We refer to the [README.md](evaluation/README.md) in the evaluation folder to run *Hyper*SLAM.

## Literature

1. [Continuous-Time Stereo-Inertial Odometry, Hug et al. (2022)](https://ieeexplore.ieee.org/document/9772323)
2. [HyperSLAM: A Generic and Modular Approach to Sensor Fusion and Simultaneous<br /> Localization And Mapping in Continuous-Time, Hug and Chli (2020)](https://ieeexplore.ieee.org/document/9320417)
3. [Efficient Derivative Computation for Cumulative B-Splines on Lie Groups, Sommer et al. (2020)](https://ieeexplore.ieee.org/document/9157639)
4. [A Micro Lie Theory for State Estimation in Robotics, Solà et al. (2018)](https://arxiv.org/abs/1812.01537)
5. [A Primer on the Differential Calculus of 3D Orientations, Bloesch et al. (2016)](https://arxiv.org/abs/1606.05285)
6. [A Generic Camera Model and Calibration Method for Conventional,<br /> Wide-Angle, and Fish-Eye Lenses, Kannala and Brandt (2006)](https://ieeexplore.ieee.org/document/1642666)
7. [Single View Point Omnidirectional Camera Calibration from Planar Grids, Mei and Rives (2007)](https://ieeexplore.ieee.org/document/4209702)

### Updates

24.10.22 Release of [*Hyper*SLAM](https://github.com/VIS4ROB-lab/HyperSLAM) (beta).<br/>
25.07.22 Release of [*Hyper*Sensors](https://github.com/VIS4ROB-lab/HyperSensors) submodule.<br/>
19.07.22 Release of [*Hyper*State](https://github.com/VIS4ROB-lab/HyperState) submodule.<br/>
17.06.22 Release of [*Hyper*Variables](https://github.com/VIS4ROB-lab/HyperVariables) submodule.

### Contact

Admin - [David Hug](mailto:dhug@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland  
Maintainer - [Philipp Bänninger](mailto:baephili@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland  
Maintainer - [Ignacio Alzugaray](mailto:aignacio@ethz.ch), Leonhardstrasse 21, 8092 Zürich, ETH Zürich, Switzerland

### License

*Hyper*SLAM is distributed under the [BSD-3-Clause License](LICENSE).
