# Doxygen configuration.
#
# Here are only the options that differ from the Doxygen defaults, or from what
# doxygen_add_docs() already presets. Defaults are kept
# aligned with jrl-cmakemodules/doxygen.cmake (v1).

# Project related configuration options
# The autodoc parser rebuilds "coal/<...>.h" from paths relative to the project root
set(DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR})
# src/ is stripped too, so the autodoc parser emits the private headers as
# <narrowphase/details.h> rather than a path relative to the project root
set(DOXYGEN_STRIP_FROM_INC_PATH ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)
set(DOXYGEN_BUILTIN_STL_SUPPORT YES)

# Build related configuration options
set(DOXYGEN_EXTRACT_ALL YES)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_SORT_MEMBER_DOCS NO)
set(DOXYGEN_SORT_MEMBERS_CTORS_1ST YES)

# Configuration options related to warning and progress messages
set(DOXYGEN_QUIET YES)
set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)
set(DOXYGEN_WARN_LOGFILE doxygen.log)

# Configuration options related to the input files
set(DOXYGEN_FILE_PATTERNS *.h *.hh *.hxx)
set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/images)

# Exclude the hpp/fcl compatibility headers
set(DOXYGEN_EXCLUDE ${PROJECT_SOURCE_DIR}/include/hpp)

# We must not document octree if Octomap is not setup.
# This create a build issue when building the bindings because doxygen-autodoc will
# include octree.h that will include octomap.h.
if(NOT COAL_BUILD_WITH_OCTOMAP)
  list(
    APPEND DOXYGEN_EXCLUDE
    ${PROJECT_SOURCE_DIR}/include/coal/octree.h
    ${PROJECT_SOURCE_DIR}/include/coal/serialization/octree.h
    ${PROJECT_SOURCE_DIR}/include/coal/internal/traversal_node_octree.h
  )
endif()

# Configuration options related to source browsing
set(DOXYGEN_SOURCE_BROWSER YES)

# Configuration options related to the HTML output
set(DOXYGEN_HTML_OUTPUT doxygen-html)
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_MATHJAX_VERSION MathJax_3)
set(DOXYGEN_MATHJAX_FORMAT SVG)

# Configuration options related to the XML output
# The XML output is parsed by doxygen_xml_parser.py (ENABLE_PYTHON_DOXYGEN_AUTODOC)
set(DOXYGEN_GENERATE_XML YES)
set(DOXYGEN_XML_OUTPUT doxygen-xml)

# Configuration options related to the preprocessor
set(DOXYGEN_MACRO_EXPANSION YES)
set(
  DOXYGEN_INCLUDE_PATH
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_BINARY_DIR}/generated/include
)

# Configuration options related to the dot tool
set(DOXYGEN_DOTFILE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
set(DOXYGEN_DOT_IMAGE_FORMAT SVG)

# Configure Doxygen
# src/ is documented as well: ENABLE_PYTHON_DOXYGEN_AUTODOC picks up the
# coal::details free functions declared in src/narrowphase/details.h
doxygen_add_docs(
  doc
  ALL
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src
  COMMENT "Generating API documentation with Doxygen"
)

if(INSTALL_DOCUMENTATION)
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_HTML_OUTPUT}
      ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_XML_OUTPUT}
    DESTINATION share/doc/${PROJECT_NAME}
    COMPONENT doc
  )
endif()
