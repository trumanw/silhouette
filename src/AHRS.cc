#include "AHRS.h"
using namespace std;

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
                    gx[lcounter-1] = val;
                    break;
                case 1:
                    gy[lcounter-1] = val;
                    break;
                case 2:
                    gz[lcounter-1] = val;
                    break;
                case 3:
                    ax[lcounter-1] = val;
                    break;
                case 4:
                    ay[lcounter-1] = val;
                    break;
                case 5:
                    az[lcounter-1] = val;
                    break;
                case 8:
                    length[lcounter-1] = val;
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

    const int MAX_INIT_ARRAY_SIZE = 255;
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
    cout << filename << "\n";

    // calculate the length of the each rolling step
    for (int i = 0; i < num - 1; i++) {
        length[i] = length[i+1] - length[i];
    }
    length[num - 1] = 0;

    // init AHRSTracklet
    AHRSMadgwick *filter = new AHRSMadgwick();
    filter -> begin(10);
    AHRSTracklet *tracklet = new AHRSTracklet(*filter);

    // iterately update the points of the trajectory into pl
    for (int i = 0; i < num; i++) {
        tracklet -> update(gx[i], gy[i], gz[i], ax[i], ay[i], az[i], length[i]);
        pl[i].x = tracklet -> getX();
        pl[i].y = tracklet -> getY();
        pl[i].z = tracklet -> getZ();
        cout << "X: " << pl[i].x << " - Y: " << pl[i].y << " - Z: " << pl[i].z << " - Len: " << length[i] <<"\n";
    }

    // malloc memory for keeping all the points of trajectory
    cout << "Total points: " << num << "\n";
}
