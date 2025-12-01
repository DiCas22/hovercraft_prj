#include "No.h"

No::No(auto initData = 0, int type, int priority = 0)
{
    _data_ = initData;
    _next_ = nullptr;
    _type_ = type;
    _priority_ = priority;
}

void No::setData(auto data)
{
    _data_ = data;
}

void No::setNext(No *next)
{
    _next_ = next;
}

auto No::getData() const
{
    return _data_;
}

int No::getType() const
{
    return _type_;
}

int No::getPriority() const
{
    return _priority_;
}

No *No::getNext() const
{
    return _next_;
}