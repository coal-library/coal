SET(DOXYGEN_XML_OUTPUT          "doxygen-xml"      PARENT_SCOPE)
SET(DOXYGEN_FILE_PATTERNS       "*.h  *.hh  *.hxx" PARENT_SCOPE)
SET(DOXYGEN_GENERATE_XML        "YES"              PARENT_SCOPE)
SET(DOXYGEN_EXPAND_ONLY_PREDEF  "NO"               PARENT_SCOPE)
SET(DOXYGEN_ENABLE_PREPROCESSING "YES" PARENT_SCOPE)
SET(DOXYGEN_MACRO_EXPANSION      "YES" PARENT_SCOPE)
SET(DOXYGEN_EXCLUDE "${PROJECT_SOURCE_DIR}/include/hpp/")

# We must not document octree if Octomap is not setup.
# This create a build issue when building the bindings because doxygen-autodoc will
# include octree.h that will include octomap.h.
IF(NOT COAL_HAS_OCTOMAP)
  SET(DOXYGEN_EXCLUDE "${DOXYGEN_EXCLUDE} ${PROJECT_SOURCE_DIR}/include/coal/octree.h")
  SET(DOXYGEN_EXCLUDE "${DOXYGEN_EXCLUDE} ${PROJECT_SOURCE_DIR}/include/coal/serialization/octree.h")
  SET(DOXYGEN_EXCLUDE "${DOXYGEN_EXCLUDE} ${PROJECT_SOURCE_DIR}/include/coal/internal/traversal_node_octree.h")
ENDIF()
SET(DOXYGEN_EXCLUDE ${DOXYGEN_EXCLUDE} PARENT_SCOPE)

SET(DOXYGEN_PREDEFINED          "IS_DOXYGEN"       PARENT_SCOPE)
