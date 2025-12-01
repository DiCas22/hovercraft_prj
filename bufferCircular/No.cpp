#include "No.h"

No::No(const BufferMessage& data, int priority)
    : _data_(data),
      _priority_(priority),
      _next_(nullptr)
{}
