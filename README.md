<p align="center">
  <img src="http://iotdk.intel.com/misc/logos/mraa.png" height="150px" width="auto" algt="Mraa Logo"/>
</p>

libmraa - Low Level Skeleton Library for Communication on GNU/Linux platforms
=============================================================================

Libmraa is a C/C++ library with bindings to Java, Python and JavaScript to
interface with the IO on Galileo, Edison & other platforms, with a structured
and sane API where port names/numbering matches the board that you are on. Use
of libmraa does not tie you to specific hardware with board detection done at
runtime you can create portable code that will work across the supported
platforms.

The intent is to make it easier for developers and sensor manufacturers to map
their sensors & actuators on top of supported hardware and to allow control of
low level communication protocol by high level languages & constructs.

The MRAA project is joining the Eclipse Foundation as an Eclipse IoT project.
You can read more about this [here](https://projects.eclipse.org/proposals/eclipse-mraa).

[![Build Status](https://travis-ci.org/intel-iot-devkit/mraa.svg?branch=master)](https://travis-ci.org/intel-iot-devkit/mraa) [![Quality Gate](https://sonarcloud.io/api/project_badges/measure?project=mraa-master&metric=alert_status)](https://sonarcloud.io/dashboard?id=mraa-master)

Supported Boards
================
* LEC-PX30 with IPi-SMARC
* LEC-AL-AI with IPi-SMARC


## How to build & Install MRAA
The procedure to describes how to install MRAA on IPi-SAMRC with Ubuntu 18.04



#### **Prerequisites**

1. Install the required software package by MRAA on your IPi-SMARC board

   ```
   $ sudo apt-get update
   $ sudo apt-get install git build-essential swig3.0 python-dev python3-dev nodejs nodejs-dev cmake libjson-c-dev pkg-config libjson-c-dev
   ```

2. Download RMAA code from ADLINK GitHub to your working directory.
   ```
   $ git clone https://github.com/adlink/mraa
   ```

3. For LEC-AL-AI with IPi-SMARC, we have to insert 2C & GPIO driver

  ```
   $ sudo modprobe i2c_i801
   $ sudo modprobe gpio-pca953x
   $ echo "pca9535 0x20" > /sys/bus/i2c/devices/i2c-13/new_device
   $ echo "sx1509q 0x3e" > /sys/bus/i2c/devices/i2c-1/new_device
   ```

**Note:** Tested on Ubuntu 18.04.


#### Start to build & Install

1. Please go into mraa folder on your working directory
   ```
   $ cd mraa
   $ mkdir build
   $ cd build
   $ cmake .. -DBUILDSWIGNODE=OFF
   $ make
   $ make install
   ```

**Note**: 
  * only support C / Python languages
  * Please use root to build code








