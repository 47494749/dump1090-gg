// decoder_queue.h: Thread-safe queue for inter-thread communication.
// Uses std::queue + std::mutex. Simple, correct, no custom ring buffers.

#ifndef DECODER_QUEUE_H
#define DECODER_QUEUE_H

#include <queue>
#include <mutex>
#include <cstddef>

template<typename T>
class DecoderQueue {
public:
    explicit DecoderQueue(size_t max_size = 4096)
        : max_size_(max_size) {}

    // Returns true if pushed, false if full
    bool push(const T& item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (q_.size() >= max_size_) return false;
        q_.push(item);
        return true;
    }

    // Returns true if popped, false if empty
    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (q_.empty()) return false;
        item = std::move(q_.front());
        q_.pop();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.empty();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        std::queue<T> empty;
        q_.swap(empty);
    }

private:
    std::queue<T> q_;
    mutable std::mutex mtx_;
    size_t max_size_;
};

#endif // DECODER_QUEUE_H
