#pragma once

#include "../host_tools/msg_types.h"

class No
{
private:
    BufferMessage _data_;
    No *_next_ = nullptr;

public:
    No(const BufferMessage &msg)
        : _data_(msg), _next_(nullptr) {}

    void setData(const BufferMessage &msg) { _data_ = msg; }
    void setNext(No *next) { _next_ = next; }

    const BufferMessage &getData() const { return _data_; }
    BufferMessage &getData() { return _data_; }

    No *getNext() const { return _next_; }

    int getType() const { return static_cast<int>(_data_.type); }
    int getPriority() const { return _data_.priority; }
};