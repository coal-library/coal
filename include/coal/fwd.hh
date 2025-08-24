//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2014, CNRS-LAAS
//  Copyright (c) 2022-2024, Inria
//  Author: Florent Lamiraux
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

#ifndef COAL_FWD_HH
#define COAL_FWD_HH

#include <cassert>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "coal/config.hh"
#include "coal/deprecated.hh"
#include "coal/warning.hh"

#if _WIN32
#define COAL_PRETTY_FUNCTION __FUNCSIG__
#else
#define COAL_PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#define COAL_UNUSED_VARIABLE(var) (void)(var)

#ifdef NDEBUG
#define COAL_ONLY_USED_FOR_DEBUG(var) COAL_UNUSED_VARIABLE(var)
#else
#define COAL_ONLY_USED_FOR_DEBUG(var)
#endif

#define COAL_THROW_PRETTY(message, exception)              \
  {                                                        \
    std::stringstream ss;                                  \
    ss << "From file: " << __FILE__ << "\n";               \
    ss << "in function: " << COAL_PRETTY_FUNCTION << "\n"; \
    ss << "at line: " << __LINE__ << "\n";                 \
    ss << "message: " << message << "\n";                  \
    throw exception(ss.str());                             \
  }

#ifdef COAL_TURN_ASSERT_INTO_EXCEPTION
#define COAL_ASSERT(check, message, exception) \
  do {                                         \
    if (!(check)) {                            \
      COAL_THROW_PRETTY(message, exception);   \
    }                                          \
  } while (0)
#else
#define COAL_ASSERT(check, message, exception) \
  {                                            \
    assert((check) && message);                \
  }
#endif

#if (__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#define COAL_WITH_CXX11_SUPPORT
#endif

#if defined(__GNUC__) || defined(__clang__)
#define COAL_COMPILER_DIAGNOSTIC_PUSH _Pragma("GCC diagnostic push")
#define COAL_COMPILER_DIAGNOSTIC_POP _Pragma("GCC diagnostic pop")
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")

// GCC version 4.6 and higher supports -Wmaybe-uninitialized
#if (defined(__GNUC__) && \
     ((__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)))
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED \
  _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
// Use __has_warning with clang. Clang introduced it in 2024 (3.5+)
#elif (defined(__clang__) && defined(__has_warning) && \
       __has_warning("-Wmaybe-uninitialized"))
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED \
  _Pragma("clang diagnostic ignored \"-Wmaybe-uninitialized\"")
#else
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
#endif
#elif defined(WIN32)
#define COAL_COMPILER_DIAGNOSTIC_PUSH _Pragma("warning(push)")
#define COAL_COMPILER_DIAGNOSTIC_POP _Pragma("warning(pop)")
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS \
  _Pragma("warning(disable : 4996)")
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED \
  _Pragma("warning(disable : 4700)")
#else
#define COAL_COMPILER_DIAGNOSTIC_PUSH
#define COAL_COMPILER_DIAGNOSTIC_POP
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#define COAL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
#endif  // __GNUC__

namespace coal {
using std::dynamic_pointer_cast;
using std::make_shared;
using std::shared_ptr;

class CollisionObject;
typedef shared_ptr<CollisionObject> CollisionObjectPtr_t;
typedef shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;
class CollisionGeometry;
typedef shared_ptr<CollisionGeometry> CollisionGeometryPtr_t;
typedef shared_ptr<const CollisionGeometry> CollisionGeometryConstPtr_t;
class Transform3s;

class AABB;

class BVHModelBase;
typedef shared_ptr<BVHModelBase> BVHModelPtr_t;

class OcTree;
typedef shared_ptr<OcTree> OcTreePtr_t;
typedef shared_ptr<const OcTree> OcTreeConstPtr_t;
}  // namespace coal

#ifdef COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
namespace hpp {
namespace fcl {
using namespace coal;
using Transform3f = Transform3s;  // For backward compatibility
}  // namespace fcl
}  // namespace hpp
#endif

#endif  // COAL_FWD_HH
