//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
#define HPP_FCL_SERIALIZATION_BVH_MODEL_H

#include "hpp/fcl/BVH/BVH_model.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/BV_node.h"
#include "hpp/fcl/serialization/BV_splitter.h"
#include "hpp/fcl/serialization/BV_fitter.h"
#include "hpp/fcl/serialization/collision_object.h"

namespace boost
{
  namespace serialization
  {
  
    namespace internal
    {
      struct BVHModelBaseAccessor : hpp::fcl::BVHModelBase
      {
        typedef hpp::fcl::BVHModelBase Base;
        using Base::num_tris_allocated;
        using Base::num_vertices_allocated;
      };
    }
  
    template <class Archive>
    void serialize(Archive & ar,
                   hpp::fcl::Triangle & triangle,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("p0",triangle[0]);
      ar & make_nvp("p1",triangle[1]);
      ar & make_nvp("p2",triangle[2]);
    }
  
    template <class Archive>
    void save(Archive & ar,
              const hpp::fcl::BVHModelBase & bvh_model,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      if(bvh_model.build_state != BVH_BUILD_STATE_PROCESSED && bvh_model.build_state != BVH_BUILD_STATE_UPDATED)
      {
        throw std::invalid_argument("The BVH model is not in a BVH_BUILD_STATE_PROCESSED or BVH_BUILD_STATE_UPDATED state. Could not be serialized.");
      }
      
      ar & make_nvp("base",boost::serialization::base_object<hpp::fcl::CollisionGeometry>(bvh_model));
      ar & make_nvp("num_tris",bvh_model.num_tris);
      ar & make_nvp("num_vertices",bvh_model.num_vertices);
      ar & make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar & make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar & make_nvp("build_state",bvh_model.build_state);
      
      ar & make_nvp("num_tris_allocated",bvh_model.num_tris);
      ar & make_nvp("num_vertices_allocated",bvh_model.num_vertices);
      
      if(bvh_model.prev_vertices)
      {
        const bool has_prev_vertices = true;
        ar << make_nvp("has_prev_vertices",has_prev_vertices);
        ar << make_nvp("prev_vertices",make_array(bvh_model.prev_vertices,bvh_model.num_vertices));
      }
      else
      {
        const bool has_prev_vertices = false;
        ar & make_nvp("has_prev_vertices",has_prev_vertices);
      }
      
      if(bvh_model.convex)
      {
        const bool has_convex = true;
        ar << make_nvp("has_convex",has_convex);
      }
      else
      {
        const bool has_convex = false;
        ar << make_nvp("has_convex",has_convex);
      }
    }
    
    template <class Archive>
    void load(Archive & ar,
              hpp::fcl::BVHModelBase & bvh_model,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      
      ar >> make_nvp("base",boost::serialization::base_object<hpp::fcl::CollisionGeometry>(bvh_model));
      
      ar >> make_nvp("num_tris",bvh_model.num_tris);
      delete[] bvh_model.tri_indices;
      if(bvh_model.num_tris > 0)
        bvh_model.tri_indices = new Triangle[bvh_model.num_tris];
      else
        bvh_model.tri_indices = NULL;
      
      ar >> make_nvp("num_vertices",bvh_model.num_vertices);
      delete[] bvh_model.vertices;
      if(bvh_model.num_vertices > 0)
        bvh_model.vertices = new Vec3f[bvh_model.num_vertices];
      else
        bvh_model.vertices = NULL;
      
      ar >> make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar >> make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar >> make_nvp("build_state",bvh_model.build_state);
      
      typedef internal::BVHModelBaseAccessor Accessor;
      ar >> make_nvp("num_tris_allocated",reinterpret_cast<Accessor &>(bvh_model).num_tris_allocated);
      ar >> make_nvp("num_vertices_allocated",reinterpret_cast<Accessor&>(bvh_model).num_vertices_allocated);
      
      bool has_prev_vertices;
      ar >> make_nvp("has_prev_vertices",has_prev_vertices);
      delete[] bvh_model.prev_vertices;
      if(has_prev_vertices)
      {
        bvh_model.prev_vertices = new Vec3f[bvh_model.num_vertices];
        ar >> make_nvp("prev_vertices",make_array(bvh_model.prev_vertices,bvh_model.num_vertices));
      }
      else
        bvh_model.prev_vertices = NULL;
      
      bool has_convex = true;
      ar >> make_nvp("has_convex",has_convex);
    }
    
    HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::BVHModelBase)
  
    namespace internal
    {
      template<typename BV>
      struct BVHModelAccessor : hpp::fcl::BVHModel<BV>
      {
        typedef hpp::fcl::BVHModel<BV> Base;
        using Base::num_bvs_allocated;
        using Base::primitive_indices;
        using Base::bvs;
        using Base::num_bvs;
      };
    }
  
    template <class Archive, typename BV>
    void serialize(Archive & ar,
                   hpp::fcl::BVHModel<BV> & bvh_model,
                   const unsigned int version)
    {
      split_free(ar,bvh_model,version);
    }
  
    template <class Archive, typename BV>
    void save(Archive & ar,
              const hpp::fcl::BVHModel<BV> & bvh_model_,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      typedef internal::BVHModelAccessor<BV> Accessor;
      
      const Accessor & bvh_model = reinterpret_cast<const Accessor &>(bvh_model_);
      ar & make_nvp("base",boost::serialization::base_object<BVHModelBase>(bvh_model));
      
      if(bvh_model.primitive_indices)
      {
        const bool with_primitive_indices = true;
        ar & make_nvp("with_primitive_indices",with_primitive_indices);
        
        int num_primitives = 0;
        switch(bvh_model.getModelType())
        {
          case BVH_MODEL_TRIANGLES:
            num_primitives = bvh_model.num_tris;
            break;
          case BVH_MODEL_POINTCLOUD:
            num_primitives = bvh_model.num_vertices;
            break;
          default:
            ;
        }
        
        ar & make_nvp("num_primitives",num_primitives);
        if(num_primitives > 0)
        {
          ar & make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
        }
      }
      else
      {
        const bool with_primitive_indices = false;
        ar & make_nvp("with_primitive_indices",with_primitive_indices);
      }
      
      if(bvh_model.bvs)
      {
        const bool with_bvs = true;
        ar & make_nvp("with_bvs",with_bvs);
        ar & make_nvp("num_bvs",bvh_model.num_bvs);
        ar & make_nvp("bvs",make_array(bvh_model.bvs,bvh_model.num_bvs));
      }
      else
      {
        const bool with_bvs = false;
        ar & make_nvp("with_bvs",with_bvs);
      }
      
    }
  
    template <class Archive, typename BV>
    void load(Archive & ar,
              hpp::fcl::BVHModel<BV> & bvh_model_,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      typedef internal::BVHModelAccessor<BV> Accessor;
      
      Accessor & bvh_model = reinterpret_cast<Accessor &>(bvh_model_);
      
      ar >> make_nvp("base",boost::serialization::base_object<BVHModelBase>(bvh_model));
      
      bool with_primitive_indices;
      ar >> make_nvp("with_primitive_indices",with_primitive_indices);
      if(with_primitive_indices)
      {
        int num_primitives;
        ar >> make_nvp("num_primitives",num_primitives);
        
        delete[] bvh_model.primitive_indices;
        if(num_primitives > 0)
        {
          bvh_model.primitive_indices = new unsigned int[num_primitives];
          ar & make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
        }
        else
          bvh_model.primitive_indices = NULL;
      }
      
      bool with_bvs;
      ar >> make_nvp("with_bvs",with_bvs);
      if(with_bvs)
      {
        ar >> make_nvp("num_bvs",bvh_model.num_bvs);
        delete[] bvh_model.bvs;
        if(bvh_model.num_bvs > 0)
        {
          bvh_model.bvs = new BVNode<BV>[bvh_model.num_bvs];
          ar >> make_nvp("bvs",make_array(bvh_model.bvs,bvh_model.num_bvs));
        }
        else
          bvh_model.bvs = NULL;
      }
    }
  
  }
}

#endif // ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H