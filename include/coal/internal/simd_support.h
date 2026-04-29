/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Asensus Surgical
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of Asensus Surgical nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COAL_INTERNAL_SIMD_SUPPORT_H
#define COAL_INTERNAL_SIMD_SUPPORT_H

#include "coal/data_types.h"

namespace coal {
namespace details {
namespace simd {

/// Return true when the current binary was built with SIMD support and the
/// current CPU supports the AVX2 implementation used by this translation unit.
COAL_DLLAPI bool isAvx2Enabled();

/// Return the first point index maximizing points[i].dot(dir), matching the
/// scalar support-function tie-breaking rule. maxdot receives that dot value.
/// Returns -1 only when count <= 0.
COAL_DLLAPI int maxDot(const Vec3s* points, int count, const Vec3s& dir,
                       Scalar& maxdot);

/// Same as maxDot, for points stored as separate x/y/z arrays.
COAL_DLLAPI int maxDotSoA(const Scalar* xs, const Scalar* ys, const Scalar* zs,
                          int count, const Vec3s& dir, Scalar& maxdot);

}  // namespace simd
}  // namespace details
}  // namespace coal

#endif  // COAL_INTERNAL_SIMD_SUPPORT_H
