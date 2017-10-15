//
// Created by rain on 17-10-10.
//

#ifndef RAIN_VIO_BENCHMARK_H
#define RAIN_VIO_BENCHMARK_H

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

namespace RAIN_VIO
{

struct Data
{
    Data(FILE *f)
    {
        fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
                    &px, &py, &pz,
                    &qw, &qx, &qy, &qz,
                    &vx, &vy, &vz,
                    &wx, &wy, &wz,
                    &ax, &ay, &az
                );
        t = t / 1e9;
    }

    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

class BenchMark
{
public:
    vector<Data> mvdata;
    bool ReadData(string strcsvfilename);

}; // class BenchMark

} // namespace RAIN_VIO
#endif //RAIN_VIO_BENCHMARK_H
