#include "No.h"

No::No(const BufferMessage &initData, int type, int priority)
    : _data_(initData),
      _next_(nullptr),
      _type_(type),
      _priority_(priority)
{
}

void No::setData(const BufferMessage &data)
{
    _data_ = data;
}

void No::setNext(No *next)
{
    _next_ = next;
}

const BufferMessage &No::getData() const
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