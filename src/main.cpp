#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
using namespace std;

#define _C_LIB_INCLUDE_
// #define _CPP_LIB_INCLUDE_

#define _SLERP_ENABLED_

#ifdef _C_LIB_INCLUDE_
extern "C" {
#include "c/lib/ahrs_rotor.h"
#include "c/lib/ahrs_tracklet.h"
};
#endif

#ifdef _CPP_LIB_INCLUDE_
#include "cpp/lib/ahrs_rotor.hpp"
#include "cpp/lib/ahrs_tracklet.hpp"
#endif

int readFromCSVFiles(
    string filename, float * gx, float * gy, float * gz,
    float * ax, float * ay, float * az, float * length) {
    ifstream data(filename);
    int lcounter = 0;
    string line;
    while (getline(data, line)) {
        if (0 == lcounter) {
            lcounter += 1;
            continue;
        }
        // line delimiter by ','
        int ldelimiter = 0;
        stringstream lstream(line);
        string cell;
        while (getline(lstream, cell, ',')) {
            float val = (float)atof(cell.c_str());
            switch (ldelimiter) {
                case 0:
                    break;
                case 1:
                    length[lcounter-1] = val;
                    break;
                case 2:
                    gx[lcounter-1] = val;
                    break;
                case 3:
                    gy[lcounter-1] = val;
                    break;
                case 4:
                    gz[lcounter-1] = val;
                    break;
                case 5:
                    ax[lcounter-1] = val;
                    break;
                case 6:
                    ay[lcounter-1] = val;
                    break;
                case 7:
                    az[lcounter-1] = val;
                    break;
                default:
                    break;
            }
            ldelimiter += 1;
        }

        // update line counter;
        lcounter += 1;
    }

    return lcounter;
}

class Point {
public:
    float x;
    float y;
    float z;
};

int main(int argc, char* argv[]) {
    // read filename from command
    string filename = "";
    if (argc > 1) {
        filename = argv[1];
    }

    // don't want to malloc any memory here. So the max size should be changed manually based on the data samples.
    const int MAX_INIT_ARRAY_SIZE = 500;
    float gx[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float gy[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float gz[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float ax[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float ay[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float az[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float length[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    Point pl[MAX_INIT_ARRAY_SIZE];

    // init sensor data from CSV files
    int num = readFromCSVFiles("data/" + filename, gx, gy, gz, ax, ay, az, length);

    // calculate the length of the each rolling step
    for (int i = 0; i < num - 1; i++) {
        length[i] = length[i+1] - length[i];
    }
    length[num - 1] = 0;

    #ifdef _C_LIB_INCLUDE_
    // init the AHRS tracklet
    #ifdef _SLERP_ENABLED_
    int intplNum = 10;
    AHRSTracklet * tracklet = newAHRSSlerpTracklet(intplNum);
    #else
    AHRSTracklet * tracklet = newAHRSTracklet();
    #endif

    // iterately update the points of the trajectory into pl
    for (int i = 0; i < num-2; i++) {
        tracklet -> update(tracklet, gx[i], gy[i], gz[i], ax[i], ay[i], az[i], length[i]);
        if (tracklet -> rotor -> isSlerp) {
            // enable to use SLERP interpolated points
            for (int j = 0; j < (tracklet -> slerpNum) - 1; j++) {
                cout << tracklet -> xSlerp[j] << "," \
                    << tracklet -> ySlerp[j] << "," \
                    << tracklet -> zSlerp[j] << "," \
                    << length[i]/((tracklet -> slerpNum) - 1) << "\n";
            }
        } else {
            // use points from sensor only
            pl[i].x = tracklet -> getX(tracklet);
            pl[i].y = tracklet -> getY(tracklet);
            pl[i].z = tracklet -> getZ(tracklet);
            cout << pl[i].x << "," << pl[i].y << "," << pl[i].z << "," << length[i] <<"\n";
        }
    }

    // free the memory for the tracklet
    deleteAHRSTracklet(tracklet);
    #endif

    #ifdef _CPP_LIB_INCLUDE_
    // init AHRSTracklet
    AHRSRotor *filter = new AHRSRotor();
    filter -> begin(10);
    AHRSTracklet *tracklet = new AHRSTracklet(*filter);

    // iterately update the points of the trajectory into pl
    for (int i = 0; i < num-2; i++) {
        tracklet -> update(gx[i], gy[i], gz[i], ax[i], ay[i], az[i], length[i]);
        pl[i].x = tracklet -> getX();
        pl[i].y = tracklet -> getY();
        pl[i].z = tracklet -> getZ();
        cout << pl[i].x << "," << pl[i].y << "," << pl[i].z << "," << length[i] <<"\n";
    }
    #endif
}
