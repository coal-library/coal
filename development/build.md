# Build and develop with pixi

The easiest way to set up a development environment is to use [pixi](https://pixi.sh/latest/#installation).

[pixi](https://pixi.sh/latest/) is a cross-platform package manager for developers.
It installs all required dependencies in the `.pixi` directory.
It's used by our CI, so you get the same stable and tested dependencies.

Run the following command to install dependencies, configure, build and test the project:

```bash
pixi run test
```

The project is built in the `build` directory.

The typical workflow is:

```bash
pixi shell
pixi run configure
ninja -C build
```

After `pixi run configure`, use `cmake` and `ninja` manually to reconfigure and build the project.

## Environments

The pixi manifest contains many environments. The most common ones are:

- **default**: core coal and Python bindings with nanobind
- **all**: all coal features and Python bindings with nanobind
- **all-boost-python**: all coal features and Python bindings with Boost.Python

To activate a specific environment, run:

```bash
pixi shell -e all-boost-python
```

Using **all-boost-python** makes it easy to choose which features to build.
In this case, use the following CMake options:

- `COAL_HAS_QHULL`: Qhull support
- `BUILD_PYTHON_INTERFACE`: Python bindings
- `GENERATE_PYTHON_STUBS`: Python stubs generation

With the **all-boost-python** environment, all these options are ON.
To turn one off, pass the corresponding `-D` flag to `cmake`:

```bash
cmake -B build -DBUILD_PYTHON_INTERFACE=OFF
```
