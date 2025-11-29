#include "bufferCircular.h"

bufferCircular::bufferCircular(int maxElems = 0){
    this->_begin_ = nullptr;
    this->_numElems_ = 0;
    this->_numMaxElems_ = maxElems;
}

int bufferCircular::getLen() const{
    return _numElems_;
}

bool bufferCircular::insertData(double data) {
    if (_numElems_ == _numMaxElems_ && _numMaxElems_ != 0) return false;
    
    No* newNo = new No(data);
    
    if (_begin_ == nullptr) {
        newNo->setNext(newNo);
        _begin_ = newNo;
        _numElems_++;
        return true;
    } else {
        No* current = _begin_;
        while (current->getNext() != _begin_){
            current = current->getNext();
        }
        current->setNext(newNo);
        _numElems_++;
        return true;
    }

    return false;
}

double bufferCircular::getFirstData() {
    return _begin_->getData();
}

bool bufferCircular::searchData(double data) {
    if (_begin_ == nullptr) {
        return false;
    }

    No* current = _begin_;
    while (current != nullptr) {
        if (current->getData() == data) return true;
        current = current->getNext();
    }

    return false;
}

bool bufferCircular::removeData(double data) {
    if (_begin_ == nullptr) {
        return false;
    }

    if (_begin_->getData() == data) {
        No* temp = _begin_;
        _begin_ = _begin_->getNext();
        delete temp;
        _numElems_--;
        return true;
    }

    No* current = _begin_;

    while (current->getNext() != _begin_ && current->getNext()->getData() != data) {
        current = current->getNext();
    }

    if (current->getNext() == _begin_) return false;

    No* temp = current->getNext();
    current->setNext(temp->getNext());
    delete temp;
    _numElems_--;
    return true;
}