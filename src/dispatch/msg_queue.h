// msg_queue.h: Generic thread-safe message queue with C-callable interface.
// Replaces all SPSC lock-free ring buffers with a mutex-protected queue.
// C++ implementation, C-compatible header.
//
// Part of dump1090-gg-light.
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MSG_QUEUE_H
#define MSG_QUEUE_H

#include <stddef.h>
#include <stdint.h>

// Opaque handle to a message queue
typedef void* msg_queue_t;

// Create a queue for items of `item_size` bytes, with room for `capacity` items.
// Returns NULL on failure.
msg_queue_t msg_queue_create(uint32_t item_size, uint32_t capacity);

// Destroy a queue and free its resources.
void msg_queue_destroy(msg_queue_t q);

// Push an item. Returns 1 on success, 0 if full.
int32_t msg_queue_push(msg_queue_t q, const void *item);

// Pop an item into `item`. Returns 1 on success, 0 if empty.
int32_t msg_queue_pop(msg_queue_t q, void *item);

// Clear all items from the queue.
void msg_queue_clear(msg_queue_t q);

#endif // MSG_QUEUE_H
