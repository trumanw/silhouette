#include "AHRS.h"
using namespace std;

void readFromCSVFiles(
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
}

int main() {
    const int MAX_INIT_ARRAY_SIZE = 255;
    float gx[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float gy[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float gz[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float ax[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float ay[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float az[MAX_INIT_ARRAY_SIZE] = { 0.0 };
    float length[MAX_INIT_ARRAY_SIZE] = { 0.0 };

    // init sensor data from CSV files
    readFromCSVFiles("data/linear-motion-1.csv", gx, gy, gz, ax, ay, az, length);

    
}
