//
// Copyright (c) 2021-2022 INRIA
//

#ifndef COAL_SERIALIZATION_BVH_MODEL_H
#define COAL_SERIALIZATION_BVH_MODEL_H

#include "coal/BVH/BVH_model.h"

#include "coal/serialization/fwd.h"
#include "coal/serialization/BV_node.h"
#include "coal/serialization/BV_splitter.h"
#include "coal/serialization/collision_object.h"
#include "coal/serialization/memory.h"
#include "coal/serialization/triangle.h"

namespace boost {
namespace serialization {

namespace internal {
struct BVHModelBaseAccessor : coal::BVHModelBase {
  typedef coal::BVHModelBase Base;
  using Base::num_tris_allocated;
  using Base::num_vertices_allocated;
};
}  // namespace internal

template <class Archive>
void save(Archive& ar, const coal::BVHModelBase& bvh_model,
          const unsigned int /*version*/) {
  using namespace coal;
  if (!(bvh_model.build_state == BVH_BUILD_STATE_PROCESSED ||
        bvh_model.build_state == BVH_BUILD_STATE_UPDATED) &&
      (bvh_model.getModelType() == BVH_MODEL_TRIANGLES)) {
    COAL_THROW_PRETTY(
        "The BVH model is not in a BVH_BUILD_STATE_PROCESSED or "
        "BVH_BUILD_STATE_UPDATED state.\n"
        "The BVHModel could not be serialized.",
        std::invalid_argument);
  }

  ar& make_nvp(
      "base",
      boost::serialization::base_object<coal::CollisionGeometry>(bvh_model));

  ar& make_nvp("num_vertices", bvh_model.num_vertices);
  ar& make_nvp("vertices", bvh_model.vertices);

  ar& make_nvp("num_tris", bvh_model.num_tris);
  ar& make_nvp("tri_indices", bvh_model.tri_indices);
  ar& make_nvp("build_state", bvh_model.build_state);

  ar& make_nvp("prev_vertices", bvh_model.prev_vertices);

  //      if(bvh_model.convex)
  //      {
  //        const bool has_convex = true;
  //        ar << make_nvp("has_convex",has_convex);
  //      }
  //      else
  //      {
  //        const bool has_convex = false;
  //        ar << make_nvp("has_convex",has_convex);
  //      }
}

template <class Archive>
void load(Archive& ar, coal::BVHModelBase& bvh_model,
          const unsigned int /*version*/) {
  using namespace coal;

  ar >> make_nvp("base",
                 boost::serialization::base_object<coal::CollisionGeometry>(
                     bvh_model));

  ar >> make_nvp("num_vertices", bvh_model.num_vertices);
  ar >> make_nvp("vertices", bvh_model.vertices);

  ar >> make_nvp("num_tris", bvh_model.num_tris);
  ar >> make_nvp("tri_indices", bvh_model.tri_indices);
  ar >> make_nvp("build_state", bvh_model.build_state);

  ar >> make_nvp("prev_vertices", bvh_model.prev_vertices);

  //      bool has_convex = true;
  //      ar >> make_nvp("has_convex",has_convex);
}

COAL_SERIALIZATION_SPLIT(coal::BVHModelBase)

namespace internal {
template <typename BV>
struct BVHModelAccessor : coal::BVHModel<BV> {
  typedef coal::BVHModel<BV> Base;
  using Base::bvs;
  using Base::num_bvs;
  using Base::num_bvs_allocated;
  using Base::primitive_indices;
};
}  // namespace internal

template <class Archive, typename BV>
void serialize(Archive& ar, coal::BVHModel<BV>& bvh_model,
               const unsigned int version) {
  split_free(ar, bvh_model, version);
}

template <class Archive, typename BV>
void save(Archive& ar, const coal::BVHModel<BV>& bvh_model_,
          const unsigned int /*version*/) {
  using namespace coal;
  typedef internal::BVHModelAccessor<BV> Accessor;
  typedef BVNode<BV> Node;

  const Accessor& bvh_model = reinterpret_cast<const Accessor&>(bvh_model_);
  ar& make_nvp("base",
               boost::serialization::base_object<BVHModelBase>(bvh_model));

  //      if(bvh_model.primitive_indices)
  //      {
  //        const bool with_primitive_indices = true;
  //        ar & make_nvp("with_primitive_indices",with_primitive_indices);
  //
  //        int num_primitives = 0;
  //        switch(bvh_model.getModelType())
  //        {
  //          case BVH_MODEL_TRIANGLES:
  //            num_primitives = bvh_model.num_tris;
  //            break;
  //          case BVH_MODEL_POINTCLOUD:
  //            num_primitives = bvh_model.num_vertices;
  //            break;
  //          default:
  //            ;
  //        }
  //
  //        ar & make_nvp("num_primitives",num_primitives);
  //        if(num_primitives > 0)
  //        {
  //          typedef Eigen::Matrix<unsigned int,1,Eigen::Dynamic>
  //          AsPrimitiveIndexVector; const Eigen::Map<const
  //          AsPrimitiveIndexVector>
  //          primitive_indices_map(reinterpret_cast<const unsigned int
  //          *>(bvh_model.primitive_indices),1,num_primitives); ar &
  //          make_nvp("primitive_indices",primitive_indices_map);
  ////          ar &
  /// make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
  //        }
  //      }
  //      else
  //      {
  //        const bool with_primitive_indices = false;
  //        ar & make_nvp("with_primitive_indices",with_primitive_indices);
  //      }
  //

  if (bvh_model.bvs.get()) {
    const bool with_bvs = true;
    ar& make_nvp("with_bvs", with_bvs);
    ar& make_nvp("num_bvs", bvh_model.num_bvs);
    ar& make_nvp(
        "bvs",
        make_array(
            reinterpret_cast<const char*>(bvh_model.bvs->data()),
            sizeof(Node) *
                (std::size_t)bvh_model.num_bvs));  // Assuming BVs are POD.
  } else {
    const bool with_bvs = false;
    ar& make_nvp("with_bvs", with_bvs);
  }
}

template <class Archive, typename BV>
void load(Archive& ar, coal::BVHModel<BV>& bvh_model_,
          const unsigned int /*version*/) {
  using namespace coal;
  typedef internal::BVHModelAccessor<BV> Accessor;
  typedef BVNode<BV> Node;

  Accessor& bvh_model = reinterpret_cast<Accessor&>(bvh_model_);

  ar >> make_nvp("base",
                 boost::serialization::base_object<BVHModelBase>(bvh_model));

  //      bool with_primitive_indices;
  //      ar >> make_nvp("with_primitive_indices",with_primitive_indices);
  //      if(with_primitive_indices)
  //      {
  //        int num_primitives;
  //        ar >> make_nvp("num_primitives",num_primitives);
  //
  //        delete[] bvh_model.primitive_indices;
  //        if(num_primitives > 0)
  //        {
  //          bvh_model.primitive_indices = new unsigned int[num_primitives];
  //          ar &
  //          make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
  //        }
  //        else
  //          bvh_model.primitive_indices = NULL;
  //      }

  bool with_bvs;
  ar >> make_nvp("with_bvs", with_bvs);
  if (with_bvs) {
    unsigned int num_bvs;
    ar >> make_nvp("num_bvs", num_bvs);

    if (num_bvs != bvh_model.num_bvs) {
      bvh_model.bvs.reset();
      bvh_model.num_bvs = num_bvs;
      if (num_bvs > 0)
        bvh_model.bvs.reset(new
                            typename BVHModel<BV>::bv_node_vector_t(num_bvs));
    }
    if (num_bvs > 0) {
      ar >> make_nvp("bvs",
                     make_array(reinterpret_cast<char*>(bvh_model.bvs->data()),
                                sizeof(Node) * (std::size_t)num_bvs));
    } else
      bvh_model.bvs.reset();
  }
}

}  // namespace serialization
}  // namespace boost

namespace coal {

namespace internal {
template <typename BV>
struct memory_footprint_evaluator<::coal::BVHModel<BV>> {
  static size_t run(const ::coal::BVHModel<BV>& bvh_model) {
    return static_cast<size_t>(bvh_model.memUsage(false));
  }
};
}  // namespace internal

}  // namespace coal

COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::AABB>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::OBB>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::RSS>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::OBBRSS>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::kIOS>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::KDOP<16>>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::KDOP<18>>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::BVHModel<::coal::KDOP<24>>)

#endif  // ifndef COAL_SERIALIZATION_BVH_MODEL_H
