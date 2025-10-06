//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_MEMORY_H
#define COAL_SERIALIZATION_MEMORY_H

namespace coal {

namespace internal {
template <typename T>
struct memory_footprint_evaluator {
  static size_t run(const T&) { return sizeof(T); }
};
}  // namespace internal

/// \brief Returns the memory footpring of the input object.
/// For POD objects, this function returns the result of sizeof(T)
///
/// \param[in] object whose memory footprint needs to be evaluated.
///
/// \return the memory footprint of the input object.
template <typename T>
size_t computeMemoryFootprint(const T& object) {
  return internal::memory_footprint_evaluator<T>::run(object);
}

}  // namespace coal

#endif  // ifndef COAL_SERIALIZATION_MEMORY_H
