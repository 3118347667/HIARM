#pragma once
#include <memory>
#include <mutex>
/**
 * @brief CircularBuffer for certain command
 * @tparam T Certain Data type used in the buffer
 */
template <class T>
class CircularBuffer {
 public:
  using Ptr = std::shared_ptr<CircularBuffer<T>>;
  /**
   * @brief Constructor of threaded circular buffer
   * @param size
   */
  explicit CircularBuffer(size_t size) :
      buf_(std::unique_ptr<T[]>(new T[size])),
      max_size_(size)
  {}
  /**
   * @brief Push the data into the buffer
   * @param item Item to be pushed into the buffer
   */
  void push(T item)
  {

    buf_[head_] = item;

    if(full_)
    {
      tail_ = (tail_ + 1) % max_size_;
    }

    head_ = (head_ + 1) % max_size_;

    full_ = head_ == tail_;
  }
  /**
   * @brief Pop the data from the buffer and get the popped item
   * @param item Item to be popped from the buffer
   * @return True if buffer is not empty
   */
  bool pop(T &item)
  {

    if(isEmpty())
    {
      return false;
    }

    //Read data and advance the tail (we now have a free space)
    item = buf_[tail_];
    full_ = false;
    tail_ = (tail_ + 1) % max_size_;

    return true;
  }
  /**
   * @brief Reset the buffer
   */
  void reset()
  {
    head_ = tail_;
    full_ = false;
  }
  /**
   * @brief Decide whether the buffer is empty
   * @return True if empty
   */
  bool isEmpty() const
  {
    //if head and tail are equal, we are empty
    return (!full_ && (head_ == tail_));
  }
  /**
   * @brief Decide whether the buffer is full
   * @return True if full
   */
  bool isFull() const
  {
    //If tail is ahead the head by 1, we are full
    return full_;
  }
  /**
   * @brief Get the buffer capacity(max size of the buffer)
   * @return The buffer capacity
   */
  size_t getCapacity() const
  {
    return max_size_;
  }
  /**
   * @brief Get the current size of the buffer
   * @return The current size of the buffer
   */
  size_t getSize() const
  {
    size_t size = max_size_;

    if(!full_)
    {
      if(head_ >= tail_)
      {
        size = head_ - tail_;
      }
      else
      {
        size = max_size_ + head_ - tail_;
      }
    }

    return size;
  }

 private:
  //! buffer pointer
  std::unique_ptr<T[]> buf_;
  //! buffer head
  size_t head_ = 0;
  //! buffer tail
  size_t tail_ = 0;
  //! buffer capacity
  size_t max_size_;
  //! flag of full buffer
  bool full_ = 0;
};

/**
 * @brief ThreadedCircularBuffer for certain command
 * @tparam T Certain Data type used in the buffer
 */
template <class T>
class ThreadedCircularBuffer: public CircularBuffer<T>{
 public:
  /**
   * @brief Constructor of threaded circular buffer
   * @param size
   */
  explicit ThreadedCircularBuffer(size_t size) :
      CircularBuffer<T>(size){}
  /**
   * @brief Push the data into the buffer
   * @param item Item to be pushed into the buffer
   */
  void push(T item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CircularBuffer<T>::push(item);
  }
  /**
   * @brief Pop the data from the buffer and get the popped item
   * @param item Item to be popped from the buffer
   * @return True if buffer is not empty
   */
  bool pop(T &item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return CircularBuffer<T>::pop(item);
  }
  /**
   * @brief Reset the buffer
   */
  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CircularBuffer<T>::reset();
  }

 private:
  //! mutex of the buffer
  std::mutex mutex_;

};


