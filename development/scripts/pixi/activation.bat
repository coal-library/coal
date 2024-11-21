:: Setup ccache
set CMAKE_CXX_COMPILER_LAUNCHER=ccache

:: Create compile_commands.json for language server
set CMAKE_EXPORT_COMPILE_COMMANDS=1

:: Activate color output with Ninja
set CMAKE_COLOR_DIAGNOSTICS=1

:: Set default build value only if not previously set
if not defined COAL_BUILD_TYPE (set COAL_BUILD_TYPE=Release)
if not defined COAL_PYTHON_STUBS (set COAL_PYTHON_STUBS=ON)
if not defined COAL_HAS_QHULL (set COAL_HAS_QHULL=OFF)
