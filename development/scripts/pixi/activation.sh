#! /bin/bash
# Activation script

# Remove flags setup from cxx-compiler
unset CFLAGS
unset CPPFLAGS
unset CXXFLAGS
unset DEBUG_CFLAGS
unset DEBUG_CPPFLAGS
unset DEBUG_CXXFLAGS
unset LDFLAGS

if [[ $host_alias == *"apple"* ]];
then
  # On OSX setting the rpath and -L it's important to use the conda libc++ instead of the system one.
  # If conda-forge use install_name_tool to package some libs, -headerpad_max_install_names is then mandatory
  export LDFLAGS="-Wl,-headerpad_max_install_names -Wl,-rpath,$CONDA_PREFIX/lib -L$CONDA_PREFIX/lib"
elif [[ $host_alias == *"linux"* ]];
then
  # On GNU/Linux, I don't know if these flags are mandatory with g++ but
  # it allow to use clang++ as compiler
  export LDFLAGS="-Wl,-rpath,$CONDA_PREFIX/lib -Wl,-rpath-link,$CONDA_PREFIX/lib -L$CONDA_PREFIX/lib"
  # Conda compiler is named x86_64-conda-linux-gnu-c++, ccache can't resolve it
  # (https://ccache.dev/manual/latest.html#config_compiler_type)
  export CCACHE_COMPILERTYPE=gcc
fi
# Without -isystem, some LSP can't find headers
export COAL_CXX_FLAGS="$CXXFLAGS -isystem $CONDA_PREFIX/include"

# Set default build value only if not previously set
export COAL_BUILD_TYPE=${COAL_BUILD_TYPE:=Release}
export COAL_PYTHON_STUBS=${COAL_PYTHON_STUBS:=ON}
export COAL_PYTHON_NANOBIND=${COAL_PYTHON_NANOBIND:=ON}
export COAL_ENABLE_LOGGING=${COAL_ENABLE_LOGGING:=OFF}
export COAL_HAS_QHULL=${COAL_HAS_QHULL:=OFF}
export COAL_BUILD_WITH_OCTOMAP=${COAL_BUILD_WITH_OCTOMAP:=OFF}
export COAL_BUILD_WITH_TRACY=${COAL_BUILD_WITH_TRACY:=OFF}
