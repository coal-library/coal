/// Copyright 2025 INRIA

#include "coal/shape/geometric_shapes.h"
#include "coal/BVH/BVH_model.h"
#include "coal/serialization/memory.h"

#include "fwd.h"

using namespace coal;

template <typename T>
void defComputeMemoryFootprint(nb::module_& m) {
  m.def("computeMemoryFootprint", &computeMemoryFootprint<T>);
}

void exposeComputeMemoryFootprint(nb::module_& m) {
  defComputeMemoryFootprint<Sphere>(m);
  defComputeMemoryFootprint<Ellipsoid>(m);
  defComputeMemoryFootprint<Cone>(m);
  defComputeMemoryFootprint<Capsule>(m);
  defComputeMemoryFootprint<Cylinder>(m);
  defComputeMemoryFootprint<Box>(m);
  defComputeMemoryFootprint<Plane>(m);
  defComputeMemoryFootprint<Halfspace>(m);
  defComputeMemoryFootprint<TriangleP>(m);

  defComputeMemoryFootprint<BVHModel<OBB>>(m);
  defComputeMemoryFootprint<BVHModel<RSS>>(m);
  defComputeMemoryFootprint<BVHModel<OBBRSS>>(m);
}