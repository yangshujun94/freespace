// Copyright (c) 2022 utopilot.ai, Ltd. All Rights Reserved.

// Provides methods for accessing data stored in a ring buffer. Enable the ring buffer be treated as a stl like
// container, compatible with range-based for loop, find and minmax elements functions etc.

#pragma once

#include <iterator>

namespace fs
{

  template<typename T, typename RingBufferType>
  class RingBufferReverseIterator;

  template<typename T, typename RingBufferType>
  class RingBufferIterator
  {
  public:
    using self = RingBufferIterator<T, RingBufferType>;

    using iterator_category = std::bidirectional_iterator_tag;
    using value_type        = typename std::remove_const<T>::type;
    using difference_type   = int;
    using pointer           = value_type*;
    using reference         = value_type&;

    RingBufferIterator()                           = default;
    RingBufferIterator(const self&)                = default;
    RingBufferIterator& operator=(const self&)     = default;
    RingBufferIterator(self&&) noexcept            = default;
    RingBufferIterator& operator=(self&&) noexcept = default;
    ~RingBufferIterator()                          = default;

    RingBufferIterator(const pointer& ptr, const RingBufferType& ring_buffer):
      ptr_(ptr),
      ring_buffer_ptr_(&ring_buffer) {}

    pointer   operator->() const { return ptr_; }
    reference operator*() const { return *ptr_; }

    bool operator==(const self& rhs) const { return ptr_ == rhs.ptr_; }
    bool operator!=(const self& rhs) const { return ptr_ != rhs.ptr_; }
    bool operator>(const self& rhs) const { return virtualAddr(ptr_) > virtualAddr(rhs.ptr_); }
    bool operator<(const self& rhs) const { return virtualAddr(ptr_) < virtualAddr(rhs.ptr_); }
    bool operator>=(const self& rhs) const { return virtualAddr(ptr_) >= virtualAddr(rhs.ptr_); }
    bool operator<=(const self& rhs) const { return virtualAddr(ptr_) <= virtualAddr(rhs.ptr_); }

    difference_type operator-(const self& rhs) const { return virtualAddr(ptr_) - virtualAddr(rhs.ptr_); }

    self& operator++()
    {
      ptr_ = ptr_ == ring_buffer_ptr_->tail() ? ring_buffer_ptr_->head() : ptr_ + 1;
      return *this;
    }

    self& operator--()
    {
      ptr_ = ptr_ == ring_buffer_ptr_->head() ? ring_buffer_ptr_->tail() : ptr_ - 1;
      return *this;
    }

    self operator+(const int i) const
    {
      self iterator{*this};
      iterator.ptr_ += i;
      while(iterator.ptr_ > ring_buffer_ptr_->tail())
      {
        iterator.ptr_ -= RingBufferType::capacity();
      }
      return iterator;
    }

    self operator-(const int i) const
    {
      self iterator{*this};
      iterator.ptr_ -= i;
      while(iterator.ptr_ < ring_buffer_ptr_->head())
      {
        iterator.ptr_ += RingBufferType::capacity();
      }
      return iterator;
    }

    RingBufferReverseIterator<T, RingBufferType> toReverseIterator() const
    {
      return RingBufferReverseIterator<T, RingBufferType>{ptr_, *ring_buffer_ptr_};
    }

  private:
    pointer virtualAddr(const pointer& ptr) const
    {
      return ptr >= &ring_buffer_ptr_->front() ? ptr : ptr + RingBufferType::capacity();
    }

    pointer               ptr_             = nullptr;
    const RingBufferType* ring_buffer_ptr_ = nullptr;
  };

  template<typename T, typename RingBufferType>
  class RingBufferReverseIterator
  {
  public:
    using self = RingBufferReverseIterator<T, RingBufferType>;

    using iterator_category = std::bidirectional_iterator_tag;
    using value_type        = typename std::remove_const<T>::type;
    using difference_type   = int;
    using pointer           = value_type*;
    using reference         = value_type&;

    RingBufferReverseIterator()                           = default;
    RingBufferReverseIterator(const self&)                = default;
    RingBufferReverseIterator& operator=(const self&)     = default;
    RingBufferReverseIterator(self&&) noexcept            = default;
    RingBufferReverseIterator& operator=(self&&) noexcept = default;
    ~RingBufferReverseIterator()                          = default;

    RingBufferReverseIterator(const pointer& ptr, const RingBufferType& ring_buffer):
      ptr_(ptr),
      ring_buffer_ptr_(&ring_buffer) {}

    pointer   operator->() const { return ptr_; }
    reference operator*() const { return *ptr_; }
    bool      operator==(const self& rhs) const { return ptr_ == rhs.ptr_; }
    bool      operator!=(const self& rhs) const { return ptr_ != rhs.ptr_; }
    bool      operator>(const self& rhs) const { return virtualAddr(ptr_) < virtualAddr(rhs.ptr_); }
    bool      operator<(const self& rhs) const { return virtualAddr(ptr_) > virtualAddr(rhs.ptr_); }
    bool      operator>=(const self& rhs) const { return virtualAddr(ptr_) <= virtualAddr(rhs.ptr_); }
    bool      operator<=(const self& rhs) const { return virtualAddr(ptr_) >= virtualAddr(rhs.ptr_); }

    difference_type operator-(const self& rhs) const { return virtualAddr(rhs.ptr_) - virtualAddr(ptr_); }

    self& operator++()
    {
      ptr_ = ptr_ == ring_buffer_ptr_->head() ? ring_buffer_ptr_->tail() : ptr_ - 1;
      return *this;
    }

    self& operator--()
    {
      ptr_ = ptr_ == ring_buffer_ptr_->tail() ? ring_buffer_ptr_->head() : ptr_ + 1;
      return *this;
    }

    self operator+(const int i) const
    {
      self iterator{*this};
      iterator.ptr_ -= i;
      while(iterator.ptr_ < ring_buffer_ptr_->head())
      {
        iterator.ptr_ += RingBufferType::capacity();
      }
      return iterator;
    }

    self operator-(const int i) const
    {
      self iterator{*this};
      iterator.ptr_ += i;
      while(iterator.ptr_ > ring_buffer_ptr_->tail())
      {
        iterator.ptr_ -= RingBufferType::capacity();
      }
      return iterator;
    }

    RingBufferIterator<T, RingBufferType> toIterator() const
    {
      return RingBufferIterator<T, RingBufferType>{ptr_, *ring_buffer_ptr_};
    }

  private:
    pointer virtualAddr(const pointer& ptr) const
    {
      return ptr <= &ring_buffer_ptr_->back() ? ptr : ptr - RingBufferType::capacity();
    }

    pointer               ptr_             = nullptr;
    const RingBufferType* ring_buffer_ptr_ = nullptr;
  };

} // namespace fs
