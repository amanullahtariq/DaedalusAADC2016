#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include <vector>
#include <iostream>
#include <stdio.h>
#include <assert.h>

template <typename T>
class CircularBuffer {
 public:
  // -----------------------------------------------------------------------------------------------
  CircularBuffer(size_t size) : buffer_(size, false) {
  // -----------------------------------------------------------------------------------------------
    size_ = size;
    index_ = 0;
    lap_done_ = false;
  }
  
  // -----------------------------------------------------------------------------------------------
  void add(T item) {
  // -----------------------------------------------------------------------------------------------
    buffer_[index_] = item;
  
    // Increment the index and catch overflows
    index_++;
    if (index_ >= size_) {
      index_ = 0;
      lap_done_ = true;
    }
  }
  
  // -----------------------------------------------------------------------------------------------
  std::vector<T> getChronological() {
  // -----------------------------------------------------------------------------------------------
    std::vector<T> result;
  
    // Initialize the starting index to 0 or index_ if the vector is full already
    size_t current_index = 0;
    if (lap_done_) current_index = index_;
    
    // Initialize the end variable to index_ - 1 as standard, to size_ - 1 for underflow or to 0 else
    size_t end = 0;
    if (index_ != 0) end = index_ - 1;
    if (index_ == 0 && lap_done_) end = size_ - 1;
    
    while (current_index != end) {
      result.insert(result.begin(), buffer_[current_index]);
      current_index++;
      if (current_index >= size_) current_index = 0;
    }
    
    // Manually insert the last element
    result.insert(result.begin(), buffer_[end]);

    // Check whether the size is valid
    assert(result.size() <= size_);

    return result;
  }

 private:
  size_t size_;
  size_t index_;
  bool lap_done_;
  std::vector<T> buffer_;
};

#endif // __CIRCULAR_BUFFER_H__
