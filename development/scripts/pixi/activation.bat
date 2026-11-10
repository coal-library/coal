:: Set default build value only if not previously set
if not defined COAL_BUILD_TYPE (set COAL_BUILD_TYPE=Release)
if not defined COAL_PYTHON_STUBS (set COAL_PYTHON_STUBS=ON)
if not defined COAL_PYTHON_NANOBIND (set COAL_PYTHON_NANOBIND=ON)
if not defined COAL_ENABLE_LOGGING (set COAL_ENABLE_LOGGING=OFF)
if not defined COAL_HAS_QHULL (set COAL_HAS_QHULL=OFF)
if not defined COAL_BUILD_WITH_OCTOMAP (set COAL_BUILD_WITH_OCTOMAP=OFF)
if not defined COAL_BUILD_WITH_TRACY (set COAL_BUILD_WITH_TRACY=OFF)
