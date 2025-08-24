/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-2021 CNRS - LAAS, INRIA
 *  All rights reserved.
 *
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#define nullptr NULL
#endif
#include "coal/mesh_loader/assimp.h"

// Assimp >= 5.0 is forcing the use of C++11 keywords. A fix has been submitted
// https://github.com/assimp/assimp/pull/2758. The next lines fixes the bug for
// current version of Coal.
#include <assimp/defs.h>
#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600)) && \
    defined(AI_NO_EXCEPT)
#undef AI_NO_EXCEPT
#define AI_NO_EXCEPT
#endif

#include <assimp/DefaultLogger.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace coal {

namespace internal {

Loader::Loader() : importer(new Assimp::Importer()) {
  // set list of ignored parameters (parameters used for rendering)
  importer->SetPropertyInteger(
      AI_CONFIG_PP_RVC_FLAGS,
      aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
          aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
          aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_TEXTURES |
          aiComponent_TEXCOORDS | aiComponent_MATERIALS | aiComponent_NORMALS);

  // remove LINES and POINTS
  importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                               aiPrimitiveType_LINE | aiPrimitiveType_POINT);
}

Loader::~Loader() {
  if (importer) delete importer;
}

void Loader::load(const std::string& resource_path) {
  scene = importer->ReadFile(
      resource_path.c_str(),
      aiProcess_SortByPType | aiProcess_Triangulate |
          aiProcess_RemoveComponent | aiProcess_ImproveCacheLocality |
          aiProcess_FindDegenerates | aiProcess_JoinIdenticalVertices);

  if (!scene) {
    const std::string exception_message(
        std::string("Could not load resource ") + resource_path +
        std::string("\n") + importer->GetErrorString() + std::string("\n") +
        "Hint: the mesh directory may be wrong.");
    COAL_THROW_PRETTY(exception_message.c_str(), std::invalid_argument);
  }

  if (!scene->HasMeshes())
    COAL_THROW_PRETTY(
        (std::string("No meshes found in file ") + resource_path).c_str(),
        std::invalid_argument);
}

/**
 * @brief      Recursive procedure for building a mesh
 *
 * @param[in]  scale           Scale to apply when reading the ressource
 * @param[in]  scene           Pointer to the assimp scene
 * @param[in]  node            Current node of the scene
 * @param[in]  vertices_offset Current number of vertices in the model
 * @param      tv              Triangles and Vertices of the mesh submodels
 */
unsigned recurseBuildMesh(const coal::Vec3s& scale, const aiScene* scene,
                          const aiNode* node, unsigned vertices_offset,
                          TriangleAndVertices& tv) {
  if (!node) return 0;

  aiMatrix4x4 transform = node->mTransformation;
  aiNode* pnode = node->mParent;
  while (pnode) {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL) {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }

  unsigned nbVertices = 0;
  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      tv.vertices_.push_back(
          coal::Vec3s(p.x * scale[0], p.y * scale[1], p.z * scale[2]));
    }

    // add the indices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace& face = input_mesh->mFaces[j];
      assert(face.mNumIndices == 3 && "The size of the face is not valid.");
      tv.triangles_.push_back(
          coal::Triangle32(vertices_offset + face.mIndices[0],
                           vertices_offset + face.mIndices[1],
                           vertices_offset + face.mIndices[2]));
    }

    nbVertices += input_mesh->mNumVertices;
  }

  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    nbVertices +=
        recurseBuildMesh(scale, scene, node->mChildren[i], nbVertices, tv);
  }

  return nbVertices;
}

void buildMesh(const coal::Vec3s& scale, const aiScene* scene,
               unsigned vertices_offset, TriangleAndVertices& tv) {
  recurseBuildMesh(scale, scene, scene->mRootNode, vertices_offset, tv);
}

}  // namespace internal
}  // namespace coal
