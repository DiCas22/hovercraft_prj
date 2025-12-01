#include "bufferCircular.h"
#include <chrono>

bufferCircular::bufferCircular(int maxElems)
    : _begin_(nullptr),
      _numElems_(0),
      _numMaxElems_(maxElems)
{
}

int bufferCircular::getLen() const
{
    return _numElems_;
}

bool bufferCircular::push(const BufferMessage &msg)
{
    std::unique_lock<std::mutex> lock(_mtx_);

    if (_numMaxElems_ != 0 && _numElems_ == _numMaxElems_)
    {
        // buffer cheio
        return false;
    }

    No *newNo = new No(msg);

    if (_begin_ == nullptr)
    {
        // primeiro elemento
        newNo->setNext(newNo);
        _begin_ = newNo;
    }
    else
    {
        // insere mantendo ordem decrescente de prioridade
        No *current = _begin_;
        // queremos parar no último nó com prioridade >= à nova
        while (current->getNext() != _begin_ &&
               current->getNext()->getPriority() >= msg.priority)
        {
            current = current->getNext();
        }
        No *temp = current->getNext();
        current->setNext(newNo);
        newNo->setNext(temp);

        // se o novo tem prioridade maior que o "begin", vira o novo início
        if (msg.priority > _begin_->getPriority())
        {
            _begin_ = newNo;
        }
    }

    ++_numElems_;
    lock.unlock();
    _cv_not_empty.notify_one(); // sinaliza que agora tem item
    return true;
}

bool bufferCircular::pop_nolock(BufferMessage &out)
{
    if (_begin_ == nullptr || _numElems_ == 0)
    {
        return false;
    }

    No *first = _begin_;

    if (_numElems_ == 1)
    {
        _begin_ = nullptr;
    }
    else
    {
        // achar o último (para manter circular)
        No *last = _begin_;
        while (last->getNext() != _begin_)
        {
            last = last->getNext();
        }
        _begin_ = _begin_->getNext();
        last->setNext(_begin_);
    }

    out = first->getData();
    delete first;
    --_numElems_;
    return true;
}

BufferMessage bufferCircular::pop()
{
    std::unique_lock<std::mutex> lock(_mtx_);
    _cv_not_empty.wait(lock, [&]
                       { return _numElems_ > 0; });

    BufferMessage msg{};
    pop_nolock(msg);
    return msg;
}

bool bufferCircular::pop_timed(BufferMessage &out, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(_mtx_);

    if (!_cv_not_empty.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            [&]
            { return _numElems_ > 0; }))
    {
        return false; // timeout
    }

    return pop_nolock(out);
}

bool bufferCircular::peek(BufferMessage &out)
{
    std::lock_guard<std::mutex> lock(_mtx_);
    if (_begin_ == nullptr || _numElems_ == 0)
        return false;
    out = _begin_->getData();
    return true;
}