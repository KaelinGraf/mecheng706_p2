#include "Print.h"
#ifndef UTILS_H
#define UTILS_H

#include <stddef.h> // For size_t


//syntax clip<type>(val,min,max);
template<typename T>
T clip(T val, T min, T max){
  if (val <= min){
    val = min;
  }else if (val >= max){
    val = max;
  }
  return val;
}




// Templated Ring Buffer
// T: Data type (e.g., float, int, uint16_t)
// N: Maximum number of elements in the buffer
template <typename T, size_t N>
class RingBuffer {
  private:
    T _buffer[N];
    size_t _head;  // Index where the next element will be written
    size_t _count; // Current number of valid elements stored

  public:
    RingBuffer() : _head(0), _count(0) {
        // Initialize buffer with default values (usually 0)
        for (size_t i = 0; i < N; ++i) {
            _buffer[i] = T();
        }
    }

    // Insert a new value. Overwrites the oldest value if full.
    void push(T value) {
        _buffer[_head] = value;
        _head = (_head + 1) % N; // Wrap around
        if (_count < N) {
            _count++;
        }
    }

    // Returns the number of elements currently in the buffer
    size_t count() const {
        return _count;
    }

    // Empties the buffer logically
    void clear() {
        _head = 0;
        _count = 0;
    }

    // Access elements: index 0 is always the OLDEST element.
    T operator[](size_t index) const {
        if (index >= _count) return T(); // Out of bounds safety

        size_t actual_index;
        if (_count < N) {
            actual_index = index;
        } else {
            actual_index = (_head + index) % N;
        }
        return _buffer[actual_index];
    }

    // Helper: Calculates the average of the buffer (Great for filtering)
    float average() const {
        if (_count == 0) return -1.0f;
        T sum = T();
        for (size_t i = 0; i < _count; ++i) {
            sum += _buffer[i];
        }
        return static_cast<float>(sum) / _count;
    }

    float median() const {
        if (_count == 0) return -1.0f;

        // 1. Copy data to a temporary array (so we don't mess up the RingBuffer)
        T temp[N];
        for (size_t i = 0; i < _count; i++) {
            temp[i] = _buffer[(_head + i) % N];
        }

        // 2. Basic Bubble Sort (Standard C-style sorting)
        for (size_t i = 0; i < _count - 1; i++) {
            for (size_t j = 0; j < _count - i - 1; j++) {
                if (temp[j] > temp[j + 1]) {
                    T swap = temp[j];
                    temp[j] = temp[j + 1];
                    temp[j + 1] = swap;
                }
            }
        }

        // 3. Calculate Median
        if (_count % 2 == 0) {
            // Even: Average of the two middle elements
            return (float)(temp[(_count / 2) - 1] + temp[_count / 2]) / 2.0f;
        } else {
            // Odd: The middle element
            return (float)temp[_count / 2];
        }
    }

    void reset(){
        for(size_t i = 0; i<_count; i++){
            _buffer[i] = T();
        }
        _count = 0;
    }
};

#endif // UTILS_H
