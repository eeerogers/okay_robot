#pragma once

#include <deque>
#include <vector>

template <typename T> class CircularBuffer {
public:
    using iterator = typename std::deque<T>::iterator;
    using const_iterator = typename std::deque<T>::const_iterator;

    CircularBuffer(const size_t& size)
        : size_(size) { };

    void push_back(const T& item);
    size_t size() { return this->buffer_.size(); };

    iterator begin() { return this->buffer_.begin(); }
    iterator end() { return this->buffer_.end(); }

    const_iterator begin() const { return this->buffer_.begin(); }
    const_iterator end() const { return this->buffer_.end(); }

private:
    std::deque<T> buffer_;
    const size_t size_;
    int pointer_ = 0;
};

template <typename T> void CircularBuffer<T>::push_back(const T& item)
{
    this->buffer_.push_front(item);
    if (this->buffer_.size() > this->size_)
        this->buffer_.pop_back();
}