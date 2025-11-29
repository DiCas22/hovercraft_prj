#ifndef NO_H
#define NO_H

#include <iostream>
using namespace std;

class No {
    private:
        double _data_;
        No* _next_;
    
    public:
        No(double initData = 0);

        void setData(double data);

        void setNext(No* next);

        double getData() const;

        No* getNext() const;

};

#endif