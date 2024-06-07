#pragma once

#include <memory>

#include "common/macros.h"
#include "common/math/math.h"
#include "ring_buffer_iterator.h"

namespace fs
{

  // A lightweight, highly efficient circular buffer that can hold (N - 1) elements, where N must be power of 2.
  // Note: RingBuffer is NOT thread-safe.
  template<class T, size_t N>
  class RingBuffer
  {
    static_assert(uto::isPowerOfTwo(N), "Size of buffer must be power of 2");

  public:
    using self                   = RingBuffer<T, N>;
    using value_type             = typename std::remove_const<T>::type;
    using const_type             = typename std::add_const<T>::type;
    using pointer                = value_type*;
    using iterator               = RingBufferIterator<value_type, self>;
    using const_iterator         = RingBufferIterator<const_type, self>;
    using reverse_iterator       = RingBufferReverseIterator<value_type, self>;
    using const_reverse_iterator = RingBufferReverseIterator<const_type, self>;

    RingBuffer() = default;

    RingBuffer(RingBuffer<T, N>&& rb) noexcept:
      begin_(rb.begin_),
      end_(rb.end_)
    {
      buffer_.swap(rb.buffer_);
      rb.begin_ = 0;
      rb.end_   = 0;
    }

    RingBuffer<T, N>& operator=(RingBuffer<T, N>&& rb) noexcept
    {
      buffer_.swap(rb.buffer_);
      begin_    = rb.begin_;
      end_      = rb.end_;
      rb.begin_ = 0;
      rb.end_   = 0;
      return *this;
    }

    ~RingBuffer() = default;

    static constexpr size_t capacity() { return N; }

    iterator       begin() { return iterator{&buffer_[begin_], *this}; }
    iterator       end() { return iterator{&buffer_[end_], *this}; }
    const_iterator begin() const { return const_iterator{&buffer_[begin_], *this}; }
    const_iterator end() const { return const_iterator{&buffer_[end_], *this}; }
    const_iterator cbegin() const { return begin(); }
    const_iterator cend() const { return end(); }

    reverse_iterator       rbegin() { return reverse_iterator{&buffer_[modN(end_ - 1)], *this}; }
    reverse_iterator       rend() { return reverse_iterator{&buffer_[modN(begin_ - 1)], *this}; }
    const_reverse_iterator rbegin() const { return const_reverse_iterator{&buffer_[modN(end_ - 1)], *this}; }
    const_reverse_iterator rend() const { return const_reverse_iterator{&buffer_[modN(begin_ - 1)], *this}; }
    const_reverse_iterator crbegin() const { return rbegin(); }
    const_reverse_iterator crend() const { return rend(); }

    // No bounds checking is performed for operator[]().
    T& operator[](size_t i) { return buffer_[modN(begin_ + i)]; }

    const T& operator[](size_t i) const { return buffer_[modN(begin_ + i)]; }

    // Bounds checking is performed for at().
    T& at(size_t i)
    {
      rangeCheck(i);
      return buffer_[modN(begin_ + i)];
    }

    const T& at(size_t i) const
    {
      rangeCheck(i);
      return buffer_[modN(begin_ + i)];
    }

    const T& front() const { return buffer_[begin_]; }

    T& front() { return buffer_[begin_]; }

    const T& back() const { return buffer_[modN(end_ - 1)]; }

    T& back() { return buffer_[modN(end_ - 1)]; }

    bool empty() const { return begin_ == end_; }

    bool full() const { return begin_ == modN(end_ + 1); }

    size_t size() const { return modN(end_ - begin_); }

    bool popBack()
    {
      if(empty())
        return false;
      end_ = modN(end_ - 1);
      return true;
    }

    bool popFront()
    {
      if(empty())
        return false;
      begin_ = modN(begin_ + 1);
      return true;
    }

    void clear()
    {
      begin_ = 0;
      end_   = 0;
    }

    template<class Type>
    bool pushBack(Type&& x)
    {
      if(full())
        return false;
      buffer_[end_] = std::forward<Type>(x);
      end_          = modN(end_ + 1);
      return true;
    }

    template<class Type>
    void pushBackForce(Type&& x)
    {
      if(full())
        popFront();
      buffer_[end_] = std::forward<Type>(x);
      end_          = modN(end_ + 1);
    }

    T* pushBackForce()
    {
      if(full())
        popFront();
      size_t end_previous = end_;
      end_                = modN(end_ + 1);
      return buffer_.get() + end_previous;
    }

    template<class Type>
    bool pushFront(Type&& x)
    {
      if(full())
        return false;
      begin_          = modN(begin_ - 1);
      buffer_[begin_] = std::forward<Type>(x);
      return true;
    }

    template<class Type>
    void pushFrontForce(Type&& x)
    {
      if(full())
        popBack();
      begin_          = modN(begin_ - 1);
      buffer_[begin_] = std::forward<Type>(x);
    }

    T* pushFrontForce()
    {
      if(full())
        popBack();
      begin_ = modN(begin_ - 1);
      return buffer_.get() + begin_;
    }

    // Buffer content could be modified via the pointers head() and tail().
    // These pointers are to support the iterators, and NOT supposed to be used by users of the RingBuffer.
    pointer head() const { return &buffer_[0]; }
    pointer tail() const { return &buffer_[N - 1]; }

  private:
    static constexpr size_t modN(size_t n) { return n & (N - 1); }

    void rangeCheck(size_t i) const
    {
      if(i >= size())
      {
        std::__throw_out_of_range_fmt(__N("index out of range"));
      }
    }

    std::unique_ptr<T[]> buffer_ = std::make_unique<T[]>(N);
    size_t               begin_  = 0;
    size_t               end_    = 0;

    DISALLOW_COPY_AND_ASSIGN(RingBuffer);
  };

} // namespace fs
