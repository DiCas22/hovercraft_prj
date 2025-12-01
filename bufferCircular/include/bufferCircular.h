#ifndef BUFFERCIRCULAR_H
#define BUFFERCIRCULAR_H

#include "No.h"

class bufferCircular
{
private:
    No *_begin_;
    int _numElems_;
    int _numMaxElems_;

public:
    explicit bufferCircular(int maxElems = 0);

    int getLen() const;

    int getFirstType() const;
    int getFirstPriority() const;
    const BufferMessage &getFirstData() const;

    // insere ordenado por prioridade (maior primeiro)
    bool insertData(const BufferMessage &data, int type, int priority = 0);

    bool searchData(const BufferMessage &data) const;
    bool removeData(const BufferMessage &data);
};

#endif