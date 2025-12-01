#pragma once

#include <mutex>
#include <condition_variable>
#include "No.h"

class bufferCircular
{
private:
    No *_begin_;
    int _numElems_;
    int _numMaxElems_;
    std::mutex _mtx_;
    std::condition_variable _cv_not_empty; // “semáforo” de itens disponíveis

public:
    explicit bufferCircular(int maxElems = 0);

    int getLen() const;

    // Insere mantendo ordenação por prioridade (maior first).
    // Retorna false se cheio (quando maxElems > 0).
    bool push(const BufferMessage &msg);

    // Remove e devolve o primeiro (maior prioridade).
    // Bloqueia até ter algo.
    BufferMessage pop();

    // Versão com timeout em ms. Retorna false se esvaziou o tempo.
    bool pop_timed(BufferMessage &out, int timeout_ms);

    // Só espiar o primeiro, sem remover (não bloqueante, retorna false se vazio)
    bool peek(BufferMessage &out);

    // Remove primeiro elemento (usado internamente)
    bool pop_nolock(BufferMessage &out);
};