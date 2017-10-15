//
// Created by rain on 17-10-10.
//

#include "BenchMark.h"

namespace RAIN_VIO
{

bool BenchMark::ReadData(const string strcsvfilename)
{
    FILE *f = fopen(strcsvfilename.c_str(), "r");
    if (f == NULL)
    {
        cout << "Can't load the benchmark file, the path is wrong" <<endl;
        return false;
    }

    // read the a line of the file
    char tmp[10000];
    fgets(tmp, 10000, f);
    while(!feof(f))
        mvdata.emplace_back(f);

    fclose(f);

    mvdata.pop_back();
}

}
