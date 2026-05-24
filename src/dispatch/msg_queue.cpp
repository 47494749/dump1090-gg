// msg_queue.cpp: Generic thread-safe message queue implementation.
// Uses a flat ring buffer protected by std::mutex.
// Replaces all SPSC lock-free ring buffers.

#include "msg_queue.h"

#include <mutex>
#include <cstring>
#include <cstdint>
#include <new>

struct MsgQueueImpl {
    uint8_t    *ring;
    size_t      item_size;
    size_t      capacity;
    size_t      head;
    size_t      tail;
    size_t      count;
    std::mutex  mtx;

    MsgQueueImpl(size_t isz, size_t cap)
        : item_size(isz), capacity(cap), head(0), tail(0), count(0)
    {
        ring = new (std::nothrow) uint8_t[isz * cap];
    }

    ~MsgQueueImpl() {
        delete[] ring;
    }

    bool push(const void *item) {
        std::lock_guard<std::mutex> lk(mtx);
        if (count >= capacity) return false;
        memcpy(ring + head * item_size, item, item_size);
        head = (head + 1) % capacity;
        count++;
        return true;
    }

    bool pop(void *item) {
        std::lock_guard<std::mutex> lk(mtx);
        if (count == 0) return false;
        memcpy(item, ring + tail * item_size, item_size);
        tail = (tail + 1) % capacity;
        count--;
        return true;
    }

    void clear() {
        std::lock_guard<std::mutex> lk(mtx);
        head = tail = count = 0;
    }
};

extern "C" {

msg_queue_t msg_queue_create(uint32_t item_size, uint32_t capacity) {
    auto *q = new (std::nothrow) MsgQueueImpl(item_size, capacity);
    if (q && !q->ring) { delete q; return nullptr; }
    return q;
}

void msg_queue_destroy(msg_queue_t q) {
    delete static_cast<MsgQueueImpl*>(q);
}

int32_t msg_queue_push(msg_queue_t q, const void *item) {
    return static_cast<MsgQueueImpl*>(q)->push(item) ? 1 : 0;
}

int32_t msg_queue_pop(msg_queue_t q, void *item) {
    return static_cast<MsgQueueImpl*>(q)->pop(item) ? 1 : 0;
}

void msg_queue_clear(msg_queue_t q) {
    static_cast<MsgQueueImpl*>(q)->clear();
}

} // extern "C"
