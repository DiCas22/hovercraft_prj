#include "No.h"

#ifndef BUFFERCIRCULAR.H
#define BUFFERCIRCULAR .H

class bufferCircular
{
private:
    No *_begin_;
    int _numElems_;
    int _numMaxElems_;

public:
    bufferCircular(int maxElems = 0);

    int getLen() const;
    int getFirstType();
    int getFirstPriority();
    bool insertData(auto data, int type, int priority = 0);
    bool searchData(auto data);
    bool removeData(auto data);
    auto getFirstData();
};

#endif