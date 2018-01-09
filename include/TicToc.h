//
// Created by rain on 17-9-18.
//


#ifndef RAIN_VIO_TICTOC_H
#define RAIN_VIO_TICTOC_H

#include <chrono>

namespace RAIN_VIO
{


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
        std::chrono::duration<double> elapsed_seconds = end - start;

        return elapsed_seconds.count() * 1000;
    }

private:

    std::chrono::time_point<std::chrono::system_clock> start, end;

}; // class TicTOC

} // namespace RAIN_VIO
#endif //RAIN_VIO_TICTOC_H
