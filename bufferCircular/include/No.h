#ifndef NO_H
#define NO_H

#include <iostream>
using namespace std;

class No
{
private:
    auto _data_;
    No *_next_;
    int _type_;
    int _priority_;

public:
    No(auto initData = 0, int type, int priority = 0);

    void setData(auto data);

    void setNext(No *next);

    auto getData() const;

    int getType() const;

    int getPriority() const;

    No *getNext() const;
};

#endif