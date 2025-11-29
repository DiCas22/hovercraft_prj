#include "No.h"

#ifndef BUFFERCIRCULAR.H
#define BUFFERCIRCULAR.H

class bufferCircular {
    private:
        No* _begin_;
        int _numElems_;
        int _numMaxElems_;
    
    public:
        bufferCircular(int maxElems = 0);

        int getLen()const;
        bool insertData(double data);
        bool searchData(double data);
        bool removeData(double data);
        double getFirstData();
};

#endif