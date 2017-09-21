//
// Created by rain on 17-9-18.
//

#include <chrono>

#ifndef RAIN_VIO_TICTOC_H
#define RAIN_VIO_TICTOC_H

class TicTOC
{
public:
    TicTOC()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();

        // the past time between the Tic and Toc
        std::chrono::duration_cast<double> elapsed_seconds = end -start;

        return (elapsed_seconds.count() * 1000);
    }

private:

    std::chrono::time_point<std::chrono::system_clock> start, end;

};

#endif //RAIN_VIO_TICTOC_H
