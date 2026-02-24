/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Louis Montaut */

#ifndef COAL_CONTAINER_ARRAY_VIEW_H
#define COAL_CONTAINER_ARRAY_VIEW_H

#include "coal/alloca.h"

#include <cassert>
#include <cstddef>

namespace coal {

#define COAL_MAKE_ALLOCA_ARRAY_VIEW(Type, Name, Size)   \
  Type* ptr_##Name = COAL_ALLOCA_TYPED_PTR(Type, Size); \
  ArrayView<Type> Name(ptr_##Name, Size);

/// @brief A very simple wrapper around a pointer given a size to represent
/// that does not a view that doest not manage memory.
/// @note To be used with COAL_ALLOCA_ARRAY_VIEW macro.
template <typename T>
struct ArrayView {
  /// @brief Default constructor, empty view.
  ArrayView() : data_(nullptr), size_(0) {}

  /// @brief Constructor from a pointer and a size.
  ArrayView(T* data, std::size_t size) : data_(data), size_(size) {}

  /// @brief Whether the internal data points towards a ptr.
  bool isValid() const { return data() != nullptr; }

  /// @brief Getter for the i-th element.
  T& operator[](std::size_t i) {
    assert(i < size_);
    return data_[i];
  }

  /// @brief Getter for the i-th element.
  T& operator[](int i) {
    assert(i >= 0);
    assert(i < int(size_));
    return data_[i];
  }

  /// @brief Const getter for the i-th element.
  const T& operator[](std::size_t i) const {
    assert(i < size_);
    return data_[i];
  }

  /// @brief Const getter for the i-th element.
  const T& operator[](int i) const {
    assert(i >= 0);
    assert(i < int(size_));
    return data_[i];
  }

  /// @brief Returns a pointer to the data pointer.
  T* data() { return data_; }

  /// @brief Returns a const pointer to the data pointer.
  const T* data() const { return data_; }

  /// @brief Returns a pointer to the first element of the view.
  T* begin() { return data_; }

  /// @brief Returns a const pointer to the first element of the view.
  const T* begin() const { return data_; }

  /// @brief Returns a pointer to the last element of the view.
  T* end() { return data_ + size_; }

  /// @brief Returns a const pointer to the last element of the view.
  const T* end() const { return data_ + size_; }

  /// @brief Returns the size of the view.
  std::size_t size() const { return size_; }

  /// @brief Assign a value to the n-th first elements of the view.
  void assign(std::size_t n, const T& val) {
    for (std::size_t i = 0; i < n && i < size_; ++i) {
      data_[i] = val;
    }
  }

 protected:
  /// @brief Pointer to data.
  T* data_;

  /// @brief Size of the view.
  std::size_t size_;
};

}  // namespace coal

#endif  // ifndef COAL_CONTAINER_ARRAY_VIEW_H
