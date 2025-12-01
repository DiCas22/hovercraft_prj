#ifndef NO_H
#define NO_H

#include "../host_tools/msg_types.h"

class No
{
private:
    BufferMessage _data_;
    No *_next_;
    int _type_;
    int _priority_;

public:
    No(const BufferMessage &initData, int type, int priority = 0);

    void setData(const BufferMessage &data);
    void setNext(No *next);

    const BufferMessage &getData() const;
    int getType() const;
    int getPriority() const;
    No *getNext() const;
};

#endif