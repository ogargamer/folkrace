#ifndef SPSC_TRIPLE_BUFFER_H
#define SPSC_TRIPLE_BUFFER_H

#include <stdint.h>
#include <atomic>

/**
 * @brief Lock-Free Single-Producer Single-Consumer (SPSC) Triple Buffer.
 *
 * Guarantees zero-blocking synchronization between Core 0 and Core 1.
 * Producer never blocks waiting for consumer.
 * Consumer always reads the most recent completely written snapshot.
 */
template <typename T>
class SPSCTripleBuffer {
public:
    SPSCTripleBuffer() {
        // Upper 8 bits: read index (0)
        // Middle 8 bits: available index (1)
        // Lower 8 bits: has_new flag (0)
        flags_.store(encode(0, 1, false), std::memory_order_relaxed);
        write_idx_ = 2;
    }

    /**
     * @brief Producer retrieves mutable buffer to write into.
     */
    inline T& getWriteBuffer() {
        return buffers_[write_idx_];
    }

    /**
     * @brief Producer commits freshly written buffer.
     * Swaps write_idx_ with available index atomically.
     */
    inline void commitWrite() {
        uint32_t current = flags_.load(std::memory_order_relaxed);
        uint32_t next;
        do {
            uint8_t read_idx = (current >> 16) & 0xFF;
            // Swap write_idx_ with avail_idx, set has_new = true
            next = encode(read_idx, write_idx_, true);
        } while (!flags_.compare_exchange_weak(current, next, std::memory_order_release, std::memory_order_relaxed));

        // Available index from previous state becomes the new write index
        write_idx_ = (current >> 8) & 0xFF;
    }

    /**
     * @brief Consumer checks if new data is available and swaps read buffer.
     * @return true if fresh data was acquired, false if reading existing data.
     */
    inline bool updateRead() {
        uint32_t current = flags_.load(std::memory_order_relaxed);
        if (!(current & 0x01)) {
            return false; // No new data written since last updateRead
        }
        uint32_t next;
        do {
            uint8_t read_idx = (current >> 16) & 0xFF;
            uint8_t avail_idx = (current >> 8) & 0xFF;
            // Swap read_idx with avail_idx, set has_new = false
            next = encode(avail_idx, read_idx, false);
        } while (!flags_.compare_exchange_weak(current, next, std::memory_order_acquire, std::memory_order_relaxed));

        return true;
    }

    /**
     * @brief Consumer retrieves read-only access to latest completed buffer.
     */
    inline const T& getReadBuffer() const {
        uint32_t current = flags_.load(std::memory_order_relaxed);
        uint8_t read_idx = (current >> 16) & 0xFF;
        return buffers_[read_idx];
    }

private:
    static inline uint32_t encode(uint8_t read_idx, uint8_t avail_idx, bool has_new) {
        return (static_cast<uint32_t>(read_idx) << 16) | (static_cast<uint32_t>(avail_idx) << 8) | (has_new ? 1 : 0);
    }

    T buffers_[3];
    uint8_t write_idx_;
    std::atomic<uint32_t> flags_;
};

#endif // SPSC_TRIPLE_BUFFER_H

