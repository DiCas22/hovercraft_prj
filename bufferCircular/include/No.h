#ifndef NO_H
#define NO_H

#include "msg_types.h"

// Nó da lista circular, guarda uma BufferMessage
class No {
public:
    No(const BufferMessage& data, int priority);

    const BufferMessage& getData() const { return _data_; }
    void setData(const BufferMessage& d) { _data_ = d; }

    int getPriority() const { return _priority_; }
    void setPriority(int p) { _priority_ = p; }

    No* getNext() const { return _next_; }
    void setNext(No* n) { _next_ = n; }

private:
    BufferMessage _data_;
    int _priority_;
    No* _next_;
};

#endif // NO_H
