# silhouette
Silhouette is a project contains IMU data fusion and interpolation algorithms.

# Usage
### Step 1 - build the src
run `make` to generate the executable file "silhouette".

### Step 2 - load the sensor data set
- create a fold named "data" under the root directory.
- Then put all the sensor data under this directory.
- The data format should look like this example:

```
Timestamp,Length,Gyro X,Gyro Y,Gyro Z,Accel X,Accel Y,Accel Z
113.746,   0.00,   0.98,   0.37,   2.01,-0.67,-0.02, 0.73
113.847,   0.96,  -0.37,   0.12,   2.32,-0.66,-0.01, 0.73
```

### Step 3 - build animation
using command `make anim` to plot the 3D trajectory. See details in the plot.py file.

# C and C++ version switch
The C library under the src/c/lib will be included and compiled as default setting. However, you can switch to C++ version by manually override macro definition from `_C_LIB_INCLUDE_` to `_CPP_LIB_INCLUDE_` at the beginning of the "main.cpp" file.

```

#define _C_LIB_INCLUDE_
// #define _CPP_LIB_INCLUDE_

```
