#include "bufferCircular.h"

bufferCircular::bufferCircular(int maxElems)
    : _begin_(nullptr),
      _numElems_(0),
      _numMaxElems_(maxElems)
{}

bufferCircular::~bufferCircular() {
    std::unique_lock<std::mutex> lock(_mtx_);
    No* cur = _begin_;
    if (!cur) return;

    No* start = _begin_;
    do {
        No* nxt = cur->getNext();
        delete cur;
        cur = nxt;
    } while (cur && cur != start);

    _begin_ = nullptr;
    _numElems_.store(0);
}

void bufferCircular::insert_sorted_nolock(const BufferMessage& msg) {
    No* newNo = new No(msg, msg.priority);

    if (_begin_ == nullptr) {
        newNo->setNext(newNo);
        _begin_ = newNo;
        _numElems_++;
        return;
    }

    No* current = _begin_;
    No* prev = nullptr;

    // lista ordenada por prioridade decrescente
    do {
        if (msg.priority > current->getPriority()) {
            break;
        }
        prev = current;
        current = current->getNext();
    } while (current != _begin_);

    if (!prev) {
        // inserir antes do _begin_
        No* last = _begin_;
        while (last->getNext() != _begin_) {
            last = last->getNext();
        }
        newNo->setNext(_begin_);
        last->setNext(newNo);
        _begin_ = newNo;
    } else {
        prev->setNext(newNo);
        newNo->setNext(current);
    }

    _numElems_++;
}

void bufferCircular::push(const BufferMessage& msg) {
    std::unique_lock<std::mutex> lock(_mtx_);

    if (_numMaxElems_ > 0 && _numElems_.load() >= _numMaxElems_) {
        // política simples: remove último (menor prioridade)
        if (_begin_) {
            No* cur = _begin_;
            No* prev = nullptr;
            while (cur->getNext() != _begin_) {
                prev = cur;
                cur = cur->getNext();
            }
            if (prev) {
                prev->setNext(_begin_);
            } else {
                _begin_ = nullptr;
            }
            delete cur;
            _numElems_--;
        }
    }

    insert_sorted_nolock(msg);
    _cv_not_empty.notify_one();
}

BufferMessage bufferCircular::pop_nolock() {
    No* node = _begin_;
    BufferMessage out = node->getData();

    if (_numElems_.load() == 1) {
        _begin_ = nullptr;
    } else {
        No* last = _begin_;
        while (last->getNext() != _begin_) {
            last = last->getNext();
        }
        _begin_ = node->getNext();
        last->setNext(_begin_);
    }

    delete node;
    _numElems_--;
    return out;
}

BufferMessage bufferCircular::pop() {
    std::unique_lock<std::mutex> lock(_mtx_);
    _cv_not_empty.wait(lock, [this]{ return _numElems_.load() > 0; });
    return pop_nolock();
}

int bufferCircular::getLen() const {
    return _numElems_.load();
}
