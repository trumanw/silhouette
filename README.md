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
Gyro X,Gyro Y,Gyro Z,Accel X,Accel Y,Accel Z,Degree,Time,Length
13.42,2.806,0.061,-0.147376,0.362828,0.8979199999999999,129,2043.2474399999999,223.21303322315137
```
