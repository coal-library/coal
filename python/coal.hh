#include "fwd.hh"

void exposeVersion();

void exposeMaths();

void exposeCollisionGeometries();

void exposeCollisionObject();

void exposeMeshLoader();

void exposeCollisionAPI();

void exposeContactPatchAPI();

void exposeDistanceAPI();

void exposeGJK();

void exposeTraversalStats();

#ifdef COAL_HAS_OCTOMAP
void exposeOctree();
#endif

void exposeBroadPhase();
