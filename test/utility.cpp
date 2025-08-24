#include "utility.h"

#include "coal/BV/BV.h"
#include "coal/BVH/BVH_model.h"
#include "coal/shape/geometric_shape_to_BVH_model.h"
#include "coal/collision_utility.h"
#include "coal/fwd.hh"

#include "coal/collision.h"
#include "coal/distance.h"

#include <cstdio>
#include <cstddef>
#include <fstream>
#include <iostream>

namespace coal {

BenchTimer::BenchTimer() {
#ifdef _WIN32
  QueryPerformanceFrequency(&frequency);
  startCount.QuadPart = 0;
  endCount.QuadPart = 0;
#else
  startCount.tv_sec = startCount.tv_usec = 0;
  endCount.tv_sec = endCount.tv_usec = 0;
#endif

  stopped = 0;
  startTimeInMicroSec = 0;
  endTimeInMicroSec = 0;
}

BenchTimer::~BenchTimer() {}

void BenchTimer::start() {
  stopped = 0;  // reset stop flag
#ifdef _WIN32
  QueryPerformanceCounter(&startCount);
#else
  gettimeofday(&startCount, NULL);
#endif
}

void BenchTimer::stop() {
  stopped = 1;  // set timer stopped flag

#ifdef _WIN32
  QueryPerformanceCounter(&endCount);
#else
  gettimeofday(&endCount, NULL);
#endif
}

double BenchTimer::getElapsedTimeInMicroSec() {
#ifdef _WIN32
  if (!stopped) QueryPerformanceCounter(&endCount);

  startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
  endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
  if (!stopped) gettimeofday(&endCount, NULL);

  startTimeInMicroSec =
      ((double)startCount.tv_sec * 1000000.0) + (double)startCount.tv_usec;
  endTimeInMicroSec =
      ((double)endCount.tv_sec * 1000000.0) + (double)endCount.tv_usec;
#endif

  return endTimeInMicroSec - startTimeInMicroSec;
}

double BenchTimer::getElapsedTimeInMilliSec() {
  return this->getElapsedTimeInMicroSec() * 0.001;
}

double BenchTimer::getElapsedTimeInSec() {
  return this->getElapsedTimeInMicroSec() * 0.000001;
}

double BenchTimer::getElapsedTime() { return this->getElapsedTimeInMilliSec(); }

const Eigen::IOFormat vfmt = Eigen::IOFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
const Eigen::IOFormat pyfmt = Eigen::IOFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

const Vec3s UnitX = Vec3s(1, 0, 0);
const Vec3s UnitY = Vec3s(0, 1, 0);
const Vec3s UnitZ = Vec3s(0, 0, 1);

Scalar rand_interval(Scalar rmin, Scalar rmax) {
  Scalar t = Scalar(rand()) / ((Scalar)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void loadOBJFile(const char* filename, std::vector<Vec3s>& points,
                 std::vector<Triangle32>& triangles) {
  FILE* file = fopen(filename, "rb");
  if (!file) {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while (fgets(line_buffer, 2000, file)) {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if (!first_token || first_token[0] == '#' || first_token[0] == 0) continue;

    switch (first_token[0]) {
      case 'v': {
        if (first_token[1] == 'n') {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        } else if (first_token[1] == 't') {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        } else {
          Scalar x = (Scalar)atof(strtok(NULL, "\t "));
          Scalar y = (Scalar)atof(strtok(NULL, "\t "));
          Scalar z = (Scalar)atof(strtok(NULL, "\t "));
          Vec3s p(x, y, z);
          points.push_back(p);
        }
      } break;
      case 'f': {
        Triangle32 tri;
        char* data[30];
        int n = 0;
        while ((data[n] = strtok(NULL, "\t \r\n")) != NULL) {
          if (strlen(data[n])) n++;
        }

        for (int t = 0; t < (n - 2); ++t) {
          if ((!has_texture) && (!has_normal)) {
            tri[0] = (Triangle32::IndexType)(atoi(data[0]) - 1);
            tri[1] = (Triangle32::IndexType)(atoi(data[1]) - 1);
            tri[2] = (Triangle32::IndexType)(atoi(data[2]) - 1);
          } else {
            const char* v1;
            for (Triangle32::IndexType i = 0; i < 3; i++) {
              // vertex ID
              if (i == 0)
                v1 = data[0];
              else
                v1 = data[(Triangle32::IndexType)t + i];

              tri[i] = (Triangle32::IndexType)(atoi(v1) - 1);
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

void saveOBJFile(const char* filename, std::vector<Vec3s>& points,
                 std::vector<Triangle32>& triangles) {
  std::ofstream os(filename);
  if (!os) {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  for (std::size_t i = 0; i < points.size(); ++i) {
    os << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2]
       << std::endl;
  }

  for (std::size_t i = 0; i < triangles.size(); ++i) {
    os << "f " << triangles[i][0] + 1 << " " << triangles[i][1] + 1 << " "
       << triangles[i][2] + 1 << std::endl;
  }

  os.close();
}

#ifdef COAL_HAS_OCTOMAP
OcTree loadOctreeFile(const std::string& filename, const Scalar& resolution) {
  octomap::OcTreePtr_t octree(new octomap::OcTree(filename));
  if (octree->getResolution() != resolution) {
    std::ostringstream oss;
    oss << "Resolution of the OcTree is " << octree->getResolution()
        << " and not " << resolution;
    throw std::invalid_argument(oss.str());
  }
  return coal::OcTree(octree);
}
#endif

void eulerToMatrix(Scalar a, Scalar b, Scalar c, Matrix3s& R) {
  Scalar c1 = cos(a);
  Scalar c2 = cos(b);
  Scalar c3 = cos(c);
  Scalar s1 = sin(a);
  Scalar s2 = sin(b);
  Scalar s3 = sin(c);

  R << c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3,
      -c2 * s3, s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

void generateRandomTransform(Scalar extents[6], Transform3s& transform) {
  Scalar x = rand_interval(extents[0], extents[3]);
  Scalar y = rand_interval(extents[1], extents[4]);
  Scalar z = rand_interval(extents[2], extents[5]);

  const Scalar pi = Scalar(3.1415926);
  Scalar a = rand_interval(0, 2 * pi);
  Scalar b = rand_interval(0, 2 * pi);
  Scalar c = rand_interval(0, 2 * pi);

  Matrix3s R;
  eulerToMatrix(a, b, c, R);
  Vec3s T(x, y, z);
  transform.setTransform(R, T);
}

void generateRandomTransforms(Scalar extents[6],
                              std::vector<Transform3s>& transforms,
                              std::size_t n) {
  transforms.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    Scalar x = rand_interval(extents[0], extents[3]);
    Scalar y = rand_interval(extents[1], extents[4]);
    Scalar z = rand_interval(extents[2], extents[5]);

    const Scalar pi = Scalar(3.1415926);
    Scalar a = rand_interval(0, 2 * pi);
    Scalar b = rand_interval(0, 2 * pi);
    Scalar c = rand_interval(0, 2 * pi);

    {
      Matrix3s R;
      eulerToMatrix(a, b, c, R);
      Vec3s T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}

void generateRandomTransforms(Scalar extents[6], Scalar delta_trans[3],
                              Scalar delta_rot,
                              std::vector<Transform3s>& transforms,
                              std::vector<Transform3s>& transforms2,
                              std::size_t n) {
  transforms.resize(n);
  transforms2.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    Scalar x = rand_interval(extents[0], extents[3]);
    Scalar y = rand_interval(extents[1], extents[4]);
    Scalar z = rand_interval(extents[2], extents[5]);

    const Scalar pi = Scalar(3.1415926);
    Scalar a = rand_interval(0, 2 * pi);
    Scalar b = rand_interval(0, 2 * pi);
    Scalar c = rand_interval(0, 2 * pi);

    {
      Matrix3s R;
      eulerToMatrix(a, b, c, R);
      Vec3s T(x, y, z);
      transforms[i].setTransform(R, T);
    }

    Scalar deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    Scalar deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    Scalar deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    Scalar deltaa = rand_interval(-delta_rot, delta_rot);
    Scalar deltab = rand_interval(-delta_rot, delta_rot);
    Scalar deltac = rand_interval(-delta_rot, delta_rot);

    {
      Matrix3s R;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R);
      Vec3s T(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setTransform(R, T);
    }
  }
}

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2,
                              void* cdata_) {
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const CollisionRequest& request = cdata->request;
  CollisionResult& result = cdata->result;

  if (cdata->done) return true;

  collide(o1, o2, request, result);

  if ((result.isCollision()) &&
      (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2,
                             void* cdata_, Scalar& dist) {
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const DistanceRequest& request = cdata->request;
  DistanceResult& result = cdata->result;

  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if (dist <= 0) return true;  // in collision or in touch

  return cdata->done;
}

std::string getNodeTypeName(NODE_TYPE node_type) {
  if (node_type == BV_UNKNOWN)
    return std::string("BV_UNKNOWN");
  else if (node_type == BV_AABB)
    return std::string("BV_AABB");
  else if (node_type == BV_OBB)
    return std::string("BV_OBB");
  else if (node_type == BV_RSS)
    return std::string("BV_RSS");
  else if (node_type == BV_kIOS)
    return std::string("BV_kIOS");
  else if (node_type == BV_OBBRSS)
    return std::string("BV_OBBRSS");
  else if (node_type == BV_KDOP16)
    return std::string("BV_KDOP16");
  else if (node_type == BV_KDOP18)
    return std::string("BV_KDOP18");
  else if (node_type == BV_KDOP24)
    return std::string("BV_KDOP24");
  else if (node_type == GEOM_BOX)
    return std::string("GEOM_BOX");
  else if (node_type == GEOM_SPHERE)
    return std::string("GEOM_SPHERE");
  else if (node_type == GEOM_CAPSULE)
    return std::string("GEOM_CAPSULE");
  else if (node_type == GEOM_CONE)
    return std::string("GEOM_CONE");
  else if (node_type == GEOM_CYLINDER)
    return std::string("GEOM_CYLINDER");
  else if (node_type == GEOM_CONVEX)
    return std::string("GEOM_CONVEX");
  else if (node_type == GEOM_PLANE)
    return std::string("GEOM_PLANE");
  else if (node_type == GEOM_HALFSPACE)
    return std::string("GEOM_HALFSPACE");
  else if (node_type == GEOM_TRIANGLE)
    return std::string("GEOM_TRIANGLE");
  else if (node_type == GEOM_OCTREE)
    return std::string("GEOM_OCTREE");
  else
    return std::string("invalid");
}

Quats makeQuat(Scalar w, Scalar x, Scalar y, Scalar z) {
  Quats q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}

std::ostream& operator<<(std::ostream& os, const Transform3s& tf) {
  return os << "[ " << tf.getTranslation().format(vfmt) << ", "
            << tf.getQuatRotation().coeffs().format(vfmt) << " ]";
}

std::size_t getNbRun(const int& argc, char const* const* argv,
                     std::size_t defaultValue) {
  for (int i = 0; i < argc; ++i)
    if (strcmp(argv[i], "--nb-run") == 0)
      if (i + 1 != argc) return (std::size_t)strtol(argv[i + 1], NULL, 10);
  return defaultValue;
}

void generateEnvironments(std::vector<CollisionObject*>& env, Scalar env_scale,
                          std::size_t n) {
  Scalar extents[] = {-env_scale, env_scale,  -env_scale,
                      env_scale,  -env_scale, env_scale};
  std::vector<Transform3s> transforms(n);

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Box* box = new Box(5, 10, 20);
    env.push_back(
        new CollisionObject(shared_ptr<CollisionGeometry>(box), transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Sphere* sphere = new Sphere(30);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(sphere),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Cylinder* cylinder = new Cylinder(10, 40);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(cylinder),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

void generateEnvironmentsMesh(std::vector<CollisionObject*>& env,
                              Scalar env_scale, std::size_t n) {
  Scalar extents[] = {-env_scale, env_scale,  -env_scale,
                      env_scale,  -env_scale, env_scale};
  std::vector<Transform3s> transforms;

  generateRandomTransforms(extents, transforms, n);
  Box box(5, 10, 20);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3s::Identity());
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  Sphere sphere(30);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3s::Identity(), 16, 16);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  Cylinder cylinder(10, 40);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3s::Identity(), 16, 16);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

ConvexTpl<Quadrilateral32> buildBox(Scalar l, Scalar w, Scalar d) {
  std::shared_ptr<std::vector<Vec3s>> pts(new std::vector<Vec3s>(
      {Vec3s(l, w, d), Vec3s(l, w, -d), Vec3s(l, -w, d), Vec3s(l, -w, -d),
       Vec3s(-l, w, d), Vec3s(-l, w, -d), Vec3s(-l, -w, d),
       Vec3s(-l, -w, -d)}));

  std::shared_ptr<std::vector<Quadrilateral32>> polygons(
      new std::vector<Quadrilateral32>(6));
  (*polygons)[0].set(0, 2, 3, 1);  // x+ side
  (*polygons)[1].set(2, 6, 7, 3);  // y- side
  (*polygons)[2].set(4, 5, 7, 6);  // x- side
  (*polygons)[3].set(0, 1, 5, 4);  // y+ side
  (*polygons)[4].set(1, 3, 7, 5);  // z- side
  (*polygons)[5].set(0, 2, 6, 4);  // z+ side

  return ConvexTpl<Quadrilateral32>(pts,  // points
                                    8,    // num points
                                    polygons,
                                    6  // number of polygons
  );
}

/// Takes a point and projects it onto the surface of the unit sphere
void toSphere(Vec3s& point) {
  assert(point.norm() > 1e-8);
  point /= point.norm();
}

/// Takes a point, projects it on the unit sphere and applies an ellipsoid
/// transformation to it. A point x belongs to the surface of an ellipsoid if
/// x^T * A * x = 1, where A = diag(1/r**2) with r being the radii of the
/// ellipsoid. Thus, the point y = A^(1/2) * x belongs to the unit sphere if y *
/// y = 1. Therefore, the tranformation which brings y to x is A^(-1/2) =
/// diag(r).
void toEllipsoid(Vec3s& point, const Ellipsoid& ellipsoid) {
  toSphere(point);
  point[0] *= ellipsoid.radii[0];
  point[1] *= ellipsoid.radii[1];
  point[2] *= ellipsoid.radii[2];
}

ConvexTpl<Triangle32> constructPolytopeFromEllipsoid(
    const Ellipsoid& ellipsoid) {
  Scalar PHI = (1 + std::sqrt(Scalar(5))) / 2;

  // vertices
  std::shared_ptr<std::vector<Vec3s>> pts(new std::vector<Vec3s>({
      Vec3s(-1, PHI, 0),
      Vec3s(1, PHI, 0),
      Vec3s(-1, -PHI, 0),
      Vec3s(1, -PHI, 0),

      Vec3s(0, -1, PHI),
      Vec3s(0, 1, PHI),
      Vec3s(0, -1, -PHI),
      Vec3s(0, 1, -PHI),

      Vec3s(PHI, 0, -1),
      Vec3s(PHI, 0, 1),
      Vec3s(-PHI, 0, -1),
      Vec3s(-PHI, 0, 1),
  }));

  std::vector<Vec3s>& pts_ = *pts;
  for (size_t i = 0; i < 12; ++i) {
    toEllipsoid(pts_[i], ellipsoid);
  }

  // faces
  std::shared_ptr<std::vector<Triangle32>> tris(
      new std::vector<Triangle32>(20));
  (*tris)[0].set(0, 11, 5);
  (*tris)[1].set(0, 5, 1);
  (*tris)[2].set(0, 1, 7);
  (*tris)[3].set(0, 7, 10);
  (*tris)[4].set(0, 10, 11);

  (*tris)[5].set(1, 5, 9);
  (*tris)[6].set(5, 11, 4);
  (*tris)[7].set(11, 10, 2);
  (*tris)[8].set(10, 7, 6);
  (*tris)[9].set(7, 1, 8);

  (*tris)[10].set(3, 9, 4);
  (*tris)[11].set(3, 4, 2);
  (*tris)[12].set(3, 2, 6);
  (*tris)[13].set(3, 6, 8);
  (*tris)[14].set(3, 8, 9);

  (*tris)[15].set(4, 9, 5);
  (*tris)[16].set(2, 4, 11);
  (*tris)[17].set(6, 2, 10);
  (*tris)[18].set(8, 6, 7);
  (*tris)[19].set(9, 8, 1);
  return ConvexTpl<Triangle32>(pts,   // points
                               12,    // num_points
                               tris,  // triangles
                               20     // number of triangles
  );
}

Box makeRandomBox(Scalar min_size, Scalar max_size) {
  return Box(Vec3s(rand_interval(min_size, max_size),
                   rand_interval(min_size, max_size),
                   rand_interval(min_size, max_size)));
}

Sphere makeRandomSphere(Scalar min_size, Scalar max_size) {
  return Sphere(rand_interval(min_size, max_size));
}

Ellipsoid makeRandomEllipsoid(Scalar min_size, Scalar max_size) {
  return Ellipsoid(Vec3s(rand_interval(min_size, max_size),
                         rand_interval(min_size, max_size),
                         rand_interval(min_size, max_size)));
}

Capsule makeRandomCapsule(std::array<Scalar, 2> min_size,
                          std::array<Scalar, 2> max_size) {
  return Capsule(rand_interval(min_size[0], max_size[0]),
                 rand_interval(min_size[1], max_size[1]));
}

Cone makeRandomCone(std::array<Scalar, 2> min_size,
                    std::array<Scalar, 2> max_size) {
  return Cone(rand_interval(min_size[0], max_size[0]),
              rand_interval(min_size[1], max_size[1]));
}

Cylinder makeRandomCylinder(std::array<Scalar, 2> min_size,
                            std::array<Scalar, 2> max_size) {
  return Cylinder(rand_interval(min_size[0], max_size[0]),
                  rand_interval(min_size[1], max_size[1]));
}

ConvexTpl<Triangle32> makeRandomConvex(Scalar min_size, Scalar max_size) {
  Ellipsoid ellipsoid = makeRandomEllipsoid(min_size, max_size);
  return constructPolytopeFromEllipsoid(ellipsoid);
}

Plane makeRandomPlane(Scalar min_size, Scalar max_size) {
  return Plane(Vec3s::Random().normalized(), rand_interval(min_size, max_size));
}

Halfspace makeRandomHalfspace(Scalar min_size, Scalar max_size) {
  return Halfspace(Vec3s::Random().normalized(),
                   rand_interval(min_size, max_size));
}

std::shared_ptr<ShapeBase> makeRandomGeometry(NODE_TYPE node_type) {
  switch (node_type) {
    case GEOM_TRIANGLE:
      COAL_THROW_PRETTY(std::string(COAL_PRETTY_FUNCTION) + " for " +
                            std::string(get_node_type_name(node_type)) +
                            " is not yet implemented.",
                        std::invalid_argument);
      break;
    case GEOM_BOX:
      return std::make_shared<Box>(makeRandomBox(Scalar(0.1), Scalar(1)));
      break;
    case GEOM_SPHERE:
      return std::make_shared<Sphere>(makeRandomSphere(Scalar(0.1), Scalar(1)));
      break;
    case GEOM_ELLIPSOID:
      return std::make_shared<Ellipsoid>(
          makeRandomEllipsoid(Scalar(0.1), Scalar(1)));
      break;
    case GEOM_CAPSULE:
      return std::make_shared<Capsule>(makeRandomCapsule(
          {Scalar(0.1), Scalar(0.2)}, {Scalar(0.8), Scalar(1)}));
      break;
    case GEOM_CONE:
      return std::make_shared<Cone>(makeRandomCone({Scalar(0.1), Scalar(0.2)},
                                                   {Scalar(0.8), Scalar(1.0)}));
      break;
    case GEOM_CYLINDER:
      return std::make_shared<Cylinder>(makeRandomCylinder(
          {Scalar(0.1), Scalar(0.2)}, {Scalar(0.8), Scalar(1.0)}));
      break;
    case GEOM_CONVEX:
      return std::make_shared<ConvexTpl<Triangle32>>(
          makeRandomConvex(Scalar(0.1), Scalar(1)));
      break;
    case GEOM_PLANE:
      return std::make_shared<Plane>(makeRandomPlane(Scalar(0.1), Scalar(1)));
      break;
    case GEOM_HALFSPACE:
      return std::make_shared<Halfspace>(
          makeRandomHalfspace(Scalar(0.1), Scalar(1)));
      break;
    default:
      COAL_THROW_PRETTY(std::string(get_node_type_name(node_type)) +
                            " is not a primitive shape.",
                        std::invalid_argument);
      break;
  }
}

}  // namespace coal
