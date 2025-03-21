set(DOXYGEN_FILE_PATTERNS "*.h  *.hh  *.hxx" PARENT_SCOPE)
set(DOXYGEN_GENERATE_HTML "NO" PARENT_SCOPE)
set(DOXYGEN_XML_OUTPUT "doxygen-xml" PARENT_SCOPE)
set(DOXYGEN_GENERATE_XML "YES" PARENT_SCOPE)
set(DOXYGEN_EXPAND_ONLY_PREDEF "NO" PARENT_SCOPE)
set(DOXYGEN_ENABLE_PREPROCESSING "YES" PARENT_SCOPE)
set(DOXYGEN_MACRO_EXPANSION "YES" PARENT_SCOPE)
set(DOXYGEN_INPUT "${coal_INCLUDE_DIRS}/coal" PARENT_SCOPE)
set(DOXYGEN_INCLUDE_PATH "\"${coal_INCLUDE_DIRS}\"" PARENT_SCOPE)
set(DOXYGEN_STRIP_FROM_PATH  "${coal_INCLUDE_DIRS}/.." PARENT_SCOPE)
set(DOXYGEN_STRIP_FROM_INC_PATH  "${coal_INCLUDE_DIRS}" PARENT_SCOPE)

set(DOXYGEN_EXCLUDE "${coal_INCLUDE_DIRS}/include/hpp/")

# We must not document octree if Octomap is not setup.
# This create a build issue when building the bindings because doxygen-autodoc will
# include octree.h that will include octomap.h.
if(NOT TARGET octomap)
  set(
    DOXYGEN_EXCLUDE
    "${DOXYGEN_EXCLUDE} ${coal_INCLUDE_DIRS}/coal/octree.h"
  )
  set(
    DOXYGEN_EXCLUDE
    "${DOXYGEN_EXCLUDE} ${coal_INCLUDE_DIRS}/coal/serialization/octree.h"
  )
  set(
    DOXYGEN_EXCLUDE
    "${DOXYGEN_EXCLUDE} ${coal_INCLUDE_DIRS}/coal/internal/traversal_node_octree.h"
  )
endif()
set(DOXYGEN_EXCLUDE ${DOXYGEN_EXCLUDE} PARENT_SCOPE)

set(DOXYGEN_PREDEFINED "IS_DOXYGEN" PARENT_SCOPE)
