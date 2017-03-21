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
Length,Gyro X,Gyro Y,Gyro Z,Accel X,Accel Y,Accel Z
   0.15,   1.77,  -0.06,   0.18,-0.39,-0.05, 0.92
   0.22,   0.18,  -0.06,  -0.67,-0.39,-0.03, 0.91
   0.37,   6.04,  -1.04,  -0.37,-0.41,-0.04, 0.92
```

### Step 3 - build animation
using command `make anim` to plot the 3D trajectory. See details in the plot.py file.
