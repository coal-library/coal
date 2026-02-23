/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, INRIA
 *  All rights reserved.
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
 *   * Neither the name of INRIA nor the names of its
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

#ifndef COAL_CONTACT_PATCH_SIMPLIFIER_H
#define COAL_CONTACT_PATCH_SIMPLIFIER_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "coal/collision_data.h"

namespace coal {

/// @brief Naive patch simplifier.
/// Computes the area of all possible combinations.
/// Returns the combination with the largest area.
/// This is very expensive and only meant to be used in tests to check
/// if other simplifiers return the correct answer.
class COAL_DLLAPI ContactPatchSimplifierNaive {
 public:
  ContactPatchSimplifierNaive() = default;

  /// @brief Compute the maximum-area subset of `patch_in` with
  /// `target_vertices` points and store it in `patch_out`.
  void compute(const ContactPatch& patch_in, std::size_t target_vertices,
               ContactPatch& patch_out);

  /// @brief In-place simplification helper that replaces the polygon with the
  /// max-area subset.
  void simplify(ContactPatch& patch, std::size_t target_vertices);

 private:
  std::vector<Vec2s> simplified_buffer_;
};

/// @brief Dynamic-programming simplifier that preserves the largest possible
/// polygon area with `target_vertices` points.
class COAL_DLLAPI ContactPatchSimplifierMaxArea {
 public:
  ContactPatchSimplifierMaxArea() = default;

  /// @brief Compute the maximum-area subset of `patch_in` with
  /// `target_vertices` points and store it in `patch_out`.
  void compute(const ContactPatch& patch_in, std::size_t target_vertices,
               ContactPatch& patch_out);

  /// @brief In-place simplification helper that replaces the polygon with the
  /// max-area subset.
  void simplify(ContactPatch& patch, std::size_t target_vertices);

 private:
  std::vector<Vec2s> simplified_buffer_;
};

/// @brief Greedy contact patch simplifier based on the Visvalingam–Whyatt rule.
class COAL_DLLAPI ContactPatchSimplifierGreedy {
 public:
  using Index = std::size_t;

  ContactPatchSimplifierGreedy() = default;

  /// @brief Compute a simplified approximation of `patch_in` and write it into
  /// `patch_out` while preserving the supporting frame metadata.
  void compute(const ContactPatch& patch_in, std::size_t target_vertices,
               ContactPatch& patch_out);

  /// @brief In-place simplification helper that replaces the polygon with a
  /// greedy approximation.
  void simplify(ContactPatch& patch, std::size_t target_vertices);

 private:
  struct HeapEntry {
    Index idx;
    Scalar area;
    Index version;
  };

  std::vector<Vec2s> simplified_buffer_;
  std::vector<int> prev_;
  std::vector<int> next_;
  std::vector<bool> removed_;
  std::vector<Index> versions_;
  std::vector<HeapEntry> heap_storage_;
};

}  // namespace coal

#endif  // COAL_CONTACT_PATCH_SIMPLIFIER_H
