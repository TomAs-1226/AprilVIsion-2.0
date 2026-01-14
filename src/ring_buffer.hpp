#pragma once
/**
 * @file ring_buffer.hpp
 * @brief Lock-free single-producer single-consumer ring buffer
 *
 * Designed for "latest frame wins" semantics - if consumer is slow,
 * older frames are automatically dropped. This is ideal for vision
 * processing where we always want the freshest data.
 */

#include <atomic>
#include <array>
#include <optional>
#include <memory>

namespace frc_vision {

/**
 * @brief SPSC ring buffer with "latest wins" semantics
 *
 * @tparam T Element type (must be movable)
 * @tparam N Buffer size (power of 2 recommended)
 */
template <typename T, size_t N = 4>
class RingBuffer {
public:
    RingBuffer() {
        for (auto& item : buffer_) {
            item = std::make_unique<Slot>();
        }
    }

    /**
     * @brief Push an item, overwriting oldest if full
     * @return true if pushed successfully, false if a slot was overwritten
     */
    bool push(T item) {
        size_t write_idx = write_pos_.load(std::memory_order_relaxed);
        size_t next_write = (write_idx + 1) % N;

        auto& slot = buffer_[write_idx];

        // Store the item
        {
            std::lock_guard<std::mutex> lock(slot->mutex);
            slot->data = std::move(item);
            slot->valid = true;
        }

        // Advance write position
        write_pos_.store(next_write, std::memory_order_release);

        // Check if we overwrote unread data
        size_t read_idx = read_pos_.load(std::memory_order_acquire);
        if (next_write == read_idx) {
            // Consumer is behind, advance read position (drop oldest)
            read_pos_.store((read_idx + 1) % N, std::memory_order_release);
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        return true;
    }

    /**
     * @brief Pop the oldest available item
     * @return Item if available, nullopt otherwise
     */
    std::optional<T> pop() {
        size_t read_idx = read_pos_.load(std::memory_order_relaxed);
        size_t write_idx = write_pos_.load(std::memory_order_acquire);

        // Buffer empty?
        if (read_idx == write_idx) {
            return std::nullopt;
        }

        auto& slot = buffer_[read_idx];
        T result;
        {
            std::lock_guard<std::mutex> lock(slot->mutex);
            if (!slot->valid) {
                return std::nullopt;
            }
            result = std::move(slot->data);
            slot->valid = false;
        }

        // Advance read position
        read_pos_.store((read_idx + 1) % N, std::memory_order_release);
        return result;
    }

    /**
     * @brief Get the latest item without removing it
     * @return Latest item if available, nullopt otherwise
     */
    std::optional<T> peek_latest() const {
        size_t write_idx = write_pos_.load(std::memory_order_acquire);
        if (write_idx == 0) {
            return std::nullopt;
        }

        size_t latest_idx = (write_idx - 1 + N) % N;
        auto& slot = buffer_[latest_idx];

        std::lock_guard<std::mutex> lock(slot->mutex);
        if (slot->valid) {
            return slot->data;
        }
        return std::nullopt;
    }

    /**
     * @brief Get the latest item, discarding all older items
     *
     * This is the "latest frame wins" semantic - ideal for vision.
     * @return Latest item if available, nullopt otherwise
     */
    std::optional<T> pop_latest() {
        size_t write_idx = write_pos_.load(std::memory_order_acquire);
        size_t read_idx = read_pos_.load(std::memory_order_relaxed);

        // Buffer empty?
        if (read_idx == write_idx) {
            return std::nullopt;
        }

        // Point to the slot just before write position (latest written)
        size_t latest_idx = (write_idx - 1 + N) % N;

        auto& slot = buffer_[latest_idx];
        T result;
        {
            std::lock_guard<std::mutex> lock(slot->mutex);
            if (!slot->valid) {
                return std::nullopt;
            }
            result = std::move(slot->data);
            slot->valid = false;
        }

        // Skip all older frames - move read position to just after latest
        size_t skipped = 0;
        while (read_idx != latest_idx) {
            auto& skip_slot = buffer_[read_idx];
            {
                std::lock_guard<std::mutex> lock(skip_slot->mutex);
                if (skip_slot->valid) {
                    skip_slot->valid = false;
                    skipped++;
                }
            }
            read_idx = (read_idx + 1) % N;
        }

        read_pos_.store(write_idx, std::memory_order_release);

        if (skipped > 0) {
            dropped_count_.fetch_add(skipped, std::memory_order_relaxed);
        }

        return result;
    }

    /**
     * @brief Check if buffer has items available
     */
    bool empty() const {
        return read_pos_.load(std::memory_order_acquire) ==
               write_pos_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get number of items currently in buffer
     */
    size_t size() const {
        size_t write_idx = write_pos_.load(std::memory_order_acquire);
        size_t read_idx = read_pos_.load(std::memory_order_acquire);
        return (write_idx - read_idx + N) % N;
    }

    /**
     * @brief Get total number of dropped items (due to slow consumer)
     */
    uint64_t dropped_count() const {
        return dropped_count_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Clear the buffer
     */
    void clear() {
        for (auto& slot : buffer_) {
            std::lock_guard<std::mutex> lock(slot->mutex);
            slot->valid = false;
        }
        read_pos_.store(0, std::memory_order_release);
        write_pos_.store(0, std::memory_order_release);
    }

    /**
     * @brief Get buffer capacity
     */
    static constexpr size_t capacity() { return N; }

private:
    struct Slot {
        T data;
        bool valid = false;
        mutable std::mutex mutex;
    };

    std::array<std::unique_ptr<Slot>, N> buffer_;
    std::atomic<size_t> read_pos_{0};
    std::atomic<size_t> write_pos_{0};
    std::atomic<uint64_t> dropped_count_{0};
};

/**
 * @brief Thread-safe wrapper for sharing the latest value
 *
 * Useful for sharing the latest fused pose or status between threads.
 */
template <typename T>
class LatestValue {
public:
    void set(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = std::move(value);
        has_value_ = true;
        version_++;
    }

    std::optional<T> get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_value_) {
            return value_;
        }
        return std::nullopt;
    }

    T get_or(const T& default_value) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_value_ ? value_ : default_value;
    }

    /**
     * @brief Get value only if it has changed since last check
     * @param last_version Previous version seen by caller
     * @return Pair of (value, new_version) if changed, nullopt otherwise
     */
    std::optional<std::pair<T, uint64_t>> get_if_changed(uint64_t last_version) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_value_ && version_ > last_version) {
            return std::make_pair(value_, version_);
        }
        return std::nullopt;
    }

    uint64_t version() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return version_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        has_value_ = false;
        version_++;
    }

private:
    mutable std::mutex mutex_;
    T value_;
    bool has_value_ = false;
    uint64_t version_ = 0;
};

} // namespace frc_vision
