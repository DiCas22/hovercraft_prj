#include "bufferCircular.h"

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

bool bufferCircular::insertData(const BufferMessage &data, int type, int priority)
{
    if (_numMaxElems_ != 0 && _numElems_ == _numMaxElems_)
        return false; // cheio

    No *newNo = new No(data, type, priority);

    if (_begin_ == nullptr)
    {
        newNo->setNext(newNo);
        _begin_ = newNo;
        _numElems_++;
        return true;
    }

    // lista circular ordenada por prioridade (maior primeiro)
    No *current = _begin_;

    // anda enquanto o próximo tiver prioridade >= e não voltar ao início
    while (current->getNext() != _begin_ &&
           current->getNext()->getPriority() >= priority)
    {
        current = current->getNext();
    }

    No *temp = current->getNext();
    current->setNext(newNo);
    newNo->setNext(temp);

    // se o novo nó tiver prioridade maior que o begin, vira o novo begin
    if (priority > _begin_->getPriority())
    {
        _begin_ = newNo;
    }

    _numElems_++;
    return true;
}

const BufferMessage &bufferCircular::getFirstData() const
{
    return _begin_->getData();
}

int bufferCircular::getFirstType() const
{
    return _begin_->getType();
}

int bufferCircular::getFirstPriority() const
{
    return _begin_->getPriority();
}

bool bufferCircular::searchData(const BufferMessage &data) const
{
    if (_begin_ == nullptr)
        return false;

    No *current = _begin_;
    do
    {
        if (&(current->getData()) == &data)
            return true; // comparação por endereço
        current = current->getNext();
    } while (current != _begin_);

    return false;
}

bool bufferCircular::removeData(const BufferMessage &data)
{
    if (_begin_ == nullptr)
        return false;

    No *current = _begin_;
    No *prev = nullptr;

    // procura nó cujo ponteiro de data é o mesmo
    do
    {
        if (&(current->getData()) == &data)
            break;
        prev = current;
        current = current->getNext();
    } while (current != _begin_);

    if (&(current->getData()) != &data)
        return false; // não achou

    if (current == _begin_)
    {
        if (_begin_->getNext() == _begin_)
        {
            // só 1 elemento
            delete _begin_;
            _begin_ = nullptr;
        }
        else
        {
            // achar último pra fechar o círculo
            No *last = _begin_;
            while (last->getNext() != _begin_)
                last = last->getNext();
            _begin_ = _begin_->getNext();
            last->setNext(_begin_);
            delete current;
        }
    }
    else
    {
        prev->setNext(current->getNext());
        delete current;
    }

    _numElems_--;
    return true;
}