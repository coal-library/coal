//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_COLLISION_DATA_H
#define COAL_SERIALIZATION_COLLISION_DATA_H

#include "coal/collision_data.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const coal::Contact& contact,
          const unsigned int /*version*/) {
  ar& make_nvp("b1", contact.b1);
  ar& make_nvp("b2", contact.b2);
  ar& make_nvp("normal", contact.normal);
  ar& make_nvp("nearest_points", contact.nearest_points);
  ar& make_nvp("pos", contact.pos);
  ar& make_nvp("penetration_depth", contact.penetration_depth);
}

template <class Archive>
void load(Archive& ar, coal::Contact& contact, const unsigned int /*version*/) {
  ar >> make_nvp("b1", contact.b1);
  ar >> make_nvp("b2", contact.b2);
  ar >> make_nvp("normal", contact.normal);
  std::array<coal::Vec3s, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  contact.nearest_points[0] = nearest_points[0];
  contact.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("pos", contact.pos);
  ar >> make_nvp("penetration_depth", contact.penetration_depth);
  contact.o1 = NULL;
  contact.o2 = NULL;
}

COAL_SERIALIZATION_SPLIT(coal::Contact)

template <class Archive>
void serialize(Archive& ar, coal::QueryRequest& query_request,
               const unsigned int /*version*/) {
  ar& make_nvp("gjk_initial_guess", query_request.gjk_initial_guess);
  // TODO: use gjk_initial_guess instead
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  ar& make_nvp("enable_cached_gjk_guess",
               query_request.enable_cached_gjk_guess);
  COAL_COMPILER_DIAGNOSTIC_POP
  ar& make_nvp("cached_gjk_guess", query_request.cached_gjk_guess);
  ar& make_nvp("cached_support_func_guess",
               query_request.cached_support_func_guess);
  ar& make_nvp("gjk_max_iterations", query_request.gjk_max_iterations);
  ar& make_nvp("gjk_tolerance", query_request.gjk_tolerance);
  ar& make_nvp("gjk_variant", query_request.gjk_variant);
  ar& make_nvp("gjk_convergence_criterion",
               query_request.gjk_convergence_criterion);
  ar& make_nvp("gjk_convergence_criterion_type",
               query_request.gjk_convergence_criterion_type);
  ar& make_nvp("epa_max_iterations", query_request.epa_max_iterations);
  ar& make_nvp("epa_tolerance", query_request.epa_tolerance);
  ar& make_nvp("collision_distance_threshold",
               query_request.collision_distance_threshold);
  ar& make_nvp("enable_timings", query_request.enable_timings);
}

template <class Archive>
void serialize(Archive& ar, coal::QueryResult& query_result,
               const unsigned int /*version*/) {
  ar& make_nvp("cached_gjk_guess", query_result.cached_gjk_guess);
  ar& make_nvp("cached_support_func_guess",
               query_result.cached_support_func_guess);
}

template <class Archive>
void serialize(Archive& ar, coal::CollisionRequest& collision_request,
               const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<coal::QueryRequest>(
                           collision_request));
  ar& make_nvp("num_max_contacts", collision_request.num_max_contacts);
  ar& make_nvp("enable_contact", collision_request.enable_contact);
  ar& make_nvp("enable_distance_lower_bound",
               collision_request.enable_distance_lower_bound);
  ar& make_nvp("security_margin", collision_request.security_margin);
  ar& make_nvp("break_distance", collision_request.break_distance);
  ar& make_nvp("distance_upper_bound", collision_request.distance_upper_bound);
}

template <class Archive>
void save(Archive& ar, const coal::CollisionResult& collision_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<coal::QueryResult>(
                           collision_result));
  ar& make_nvp("contacts", collision_result.getContacts());
  ar& make_nvp("distance_lower_bound", collision_result.distance_lower_bound);
  ar& make_nvp("nearest_points", collision_result.nearest_points);
  ar& make_nvp("normal", collision_result.normal);
}

template <class Archive>
void load(Archive& ar, coal::CollisionResult& collision_result,
          const unsigned int /*version*/) {
  ar >> make_nvp("base", boost::serialization::base_object<coal::QueryResult>(
                             collision_result));
  std::vector<coal::Contact> contacts;
  ar >> make_nvp("contacts", contacts);
  collision_result.clear();
  for (size_t k = 0; k < contacts.size(); ++k)
    collision_result.addContact(contacts[k]);
  ar >> make_nvp("distance_lower_bound", collision_result.distance_lower_bound);
  std::array<coal::Vec3s, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  collision_result.nearest_points[0] = nearest_points[0];
  collision_result.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("normal", collision_result.normal);
}

COAL_SERIALIZATION_SPLIT(coal::CollisionResult)

template <class Archive>
void serialize(Archive& ar, coal::DistanceRequest& distance_request,
               const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<coal::QueryRequest>(
                           distance_request));
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  ar& make_nvp("enable_nearest_points", distance_request.enable_nearest_points);
  COAL_COMPILER_DIAGNOSTIC_POP
  ar& make_nvp("enable_signed_distance",
               distance_request.enable_signed_distance);
  ar& make_nvp("rel_err", distance_request.rel_err);
  ar& make_nvp("abs_err", distance_request.abs_err);
}

template <class Archive>
void save(Archive& ar, const coal::DistanceResult& distance_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<coal::QueryResult>(
                           distance_result));
  ar& make_nvp("min_distance", distance_result.min_distance);
  ar& make_nvp("nearest_points", distance_result.nearest_points);
  ar& make_nvp("normal", distance_result.normal);
  ar& make_nvp("b1", distance_result.b1);
  ar& make_nvp("b2", distance_result.b2);
}

template <class Archive>
void load(Archive& ar, coal::DistanceResult& distance_result,
          const unsigned int /*version*/) {
  ar >> make_nvp("base", boost::serialization::base_object<coal::QueryResult>(
                             distance_result));
  ar >> make_nvp("min_distance", distance_result.min_distance);
  std::array<coal::Vec3s, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  distance_result.nearest_points[0] = nearest_points[0];
  distance_result.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("normal", distance_result.normal);
  ar >> make_nvp("b1", distance_result.b1);
  ar >> make_nvp("b2", distance_result.b2);
  distance_result.o1 = NULL;
  distance_result.o2 = NULL;
}

COAL_SERIALIZATION_SPLIT(coal::DistanceResult)

}  // namespace serialization
}  // namespace boost

COAL_SERIALIZATION_DECLARE_EXPORT(::coal::CollisionRequest)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::CollisionResult)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::DistanceRequest)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::DistanceResult)

#endif  // ifndef COAL_SERIALIZATION_COLLISION_DATA_H
