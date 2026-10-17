#! /bin/bash
# Activation script

if [[ $PIXI_ENVIRONMENT_PLATFORMS == *"linux"* ]];
then
  # Conda compiler is named x86_64-conda-linux-gnu-c++, ccache can't resolve it
  # (https://ccache.dev/manual/latest.html#config_compiler_type)
  export CCACHE_COMPILERTYPE=gcc
fi

# Without -isystem, some LSP can't find headers
export COAL_CXX_FLAGS="$COAL_CXX_FLAGS -isystem $CONDA_PREFIX/include"

# Set Python interpreter path for nanobind
export PYTHON_EXECUTABLE=$(which python)

# Set default build value only if not previously set
export COAL_BUILD_TYPE=${COAL_BUILD_TYPE:=Release}
export COAL_PYTHON_STUBS=${COAL_PYTHON_STUBS:=ON}
export COAL_PYTHON_NANOBIND=${COAL_PYTHON_NANOBIND:=ON}
export COAL_HAS_QHULL=${COAL_HAS_QHULL:=OFF}
