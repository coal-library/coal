:: Find Python executable path for nanobind
for /f "tokens=*" %%i in ('python -c "import sys; print(sys.executable)"') do set PYTHON_EXECUTABLE=%%i

:: Set default build value only if not previously set
if not defined COAL_BUILD_TYPE (set COAL_BUILD_TYPE=Release)
if not defined COAL_PYTHON_STUBS (set COAL_PYTHON_STUBS=ON)
if not defined COAL_PYTHON_NANOBIND (set COAL_PYTHON_NANOBIND=ON)
if not defined COAL_HAS_QHULL (set COAL_HAS_QHULL=OFF)
