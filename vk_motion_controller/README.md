# Tutorial-on-CasADi-with-CPP
This is a Tutorial on how to use CasADi with CPP.


This repo has been tested with:
* GCC 11.1.0, CMake 3.16.3, Ubuntu 20.04.5 LTS


I recommend going through the CasADi's [official documentation](https://web.casadi.org/docs/) first, then come to this repo to learn about how to use CasADi with C++.
Even though the offical documentation focuses on CasADi's usage in Python and MATLAB, there are a lot of concepts you can borrow from them when coding CasADi in C++.



Dependency
==========

* [ipopt](https://coin-or.github.io/Ipopt/)
* [CasADi](https://web.casadi.org/)


The instructions for dependency installation is shown below.


For Linux
```
$ sudo apt update
$ sudo apt install build-essential
$ sudo apt install coinor-libipopt-dev
$ pip3 install casadi
```


Instructions on upgrade GCC to Version 11
```
$ sudo apt install manpages-dev software-properties-common
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt update && sudo apt install gcc-11 g++-11
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
$ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110
```

To check GCC's version
```
$ gcc --version
$ g++ --version
```



Build
=====

To build casadi from source, see instructions [here](https://github.com/casadi/casadi/wiki/InstallationLinux).
```
$ sudo apt install gfortran liblapack-dev pkg-config --install-recommends
$ sudo apt install swig
$ cd
$ git clone https://github.com/casadi/casadi.git -b master casadi
$ cd casadi
$ mkdir build
$ cd build
$ cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
$ make
$ sudo make install
```


To build this repo,
```
$ cd
$ git clone https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP.git
$ cd <MAIN_DIRECTORY>
$ mkdir build code_gen
$ cd build
$ cmake ..
$ make
```

# Turtorial-on-Eigen-with CPP

Build
=====

```
$ cd eigen
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
