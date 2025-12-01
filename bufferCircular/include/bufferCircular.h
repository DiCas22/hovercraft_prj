#ifndef BUFFER_CIRCULAR_H
#define BUFFER_CIRCULAR_H

#include <atomic>
#include <mutex>
#include <condition_variable>
#include "msg_types.h"
#include "No.h"

// Buffer circular com prioridade para BufferMessage
class bufferCircular {
public:
    explicit bufferCircular(int maxElems = 0);
    ~bufferCircular();

    // Insere mensagem (maior prioridade é atendida primeiro)
    void push(const BufferMessage& msg);

    // Retorna próxima mensagem (bloqueia até existir algo)
    BufferMessage pop();

    int getLen() const;

private:
    No* _begin_;                       // início da lista circular
    std::atomic<int> _numElems_;       // número de elementos
    int _numMaxElems_;                 // 0 = sem limite
    mutable std::mutex _mtx_;
    std::condition_variable _cv_not_empty;

    void insert_sorted_nolock(const BufferMessage& msg);
    BufferMessage pop_nolock();
};

#endif // BUFFER_CIRCULAR_H
