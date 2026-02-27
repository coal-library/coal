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

/** This file is heavily inspired by Pinocchio */

#ifndef COAL_ALLOCA_H
#define COAL_ALLOCA_H

#include "coal/logging.h"
#include <boost/core/span.hpp>
#include <memory>

#ifdef WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif

/// @brief Maximum number of bytes that `COAL_MAKE_ALLOCA_BOOST_SPAN` will
/// allocate on the stack via `alloca`. Requests that exceed this threshold fall
/// back to a heap allocation managed by a `std::unique_ptr`.
#define COAL_ALLOCA_MAX_STACK_BYTES 65536

#define COAL_ALLOCA EIGEN_ALLOCA

#define COAL_ALLOCA_TYPED_PTR(Type, Size) \
  reinterpret_cast<Type*>(COAL_ALLOCA(sizeof(Type) * (Size)))

/// @brief Macro to create a pointer of type Type, with Size * sizeof(Type)
/// allocated elements, refered to by variable Name.
///
/// If `Size * sizeof(Type) <= COAL_ALLOCA_MAX_STACK_BYTES`, the buffer is
/// allocated on the stack via `alloca` (zero overhead, freed on function
/// return). Otherwise a heap buffer is allocated and owned by a
/// `std::unique_ptr` that is freed when it goes out of scope.
#define COAL_MAKE_ALLOCA_TYPED_PTR(Type, Name, Size)                  \
  const std::size_t _sz_##Name = static_cast<std::size_t>(Size);      \
  const std::size_t _bytes_##Name = _sz_##Name * sizeof(Type);        \
  std::unique_ptr<std::vector<Type>> _heap_buf_##Name;                \
  Type* ptr_##Name;                                                   \
  if (_bytes_##Name <= COAL_ALLOCA_MAX_STACK_BYTES) {                 \
    ptr_##Name = COAL_ALLOCA_TYPED_PTR(Type, _sz_##Name);             \
  } else {                                                            \
    COAL_LOG_WARNING(                                                 \
        "Exceeded COAL_ALLOCA_MAX_STACK_BYTES in "                    \
        "COAL_MAKE_ALLOCA_TYPED_PTR. Switching to heap allocation."); \
    _heap_buf_##Name.reset(new std::vector<Type>(_sz_##Name));        \
    ptr_##Name = _heap_buf_##Name.get()->data();                      \
  }

#define COAL_MAKE_ALLOCA_BOOST_SPAN(Type, Name, Size) \
  COAL_MAKE_ALLOCA_TYPED_PTR(Type, Name, Size);       \
  boost::span<Type> Name(ptr_##Name, _sz_##Name);

#endif  // ifndef COAL_ALLOCA_H
