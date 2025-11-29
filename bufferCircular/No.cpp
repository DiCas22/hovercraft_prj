#include "No.h"

No::No(double initData = 0) {
    _data_ = initData;
    _next_ = nullptr;
}

void No::setData(double data) {
    _data_ = data;
}

void No::setNext(No* next) {
    _next_ = next;
}

double No::getData() const {
    return _data_;
}

No* No::getNext() const {
    return _next_;
}