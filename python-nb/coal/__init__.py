# ruff: noqa: F401, F403

# On Windows, if coal.dll is not in the same directory than
# the .pyd, it will not be loaded.
# We first try to load coal, then, if it fail and we are on Windows:
#  1. We add all paths inside COAL_WINDOWS_DLL_PATH to DllDirectory
#  2. If COAL_WINDOWS_DLL_PATH we add the relative path from the
#     package directory to the bin directory to DllDirectory
# This solution is inspired from:
#  - https://github.com/PixarAnimationStudios/OpenUSD/pull/1511/files
#  - https://stackoverflow.com/questions/65334494/python-c-extension-packaging-dll-along-with-pyd
# More resources on https://github.com/diffpy/pyobjcryst/issues/33
try:
    from .coal_pywrap_nb import *  # noqa
    from .coal_pywrap_nb import __version__
except ImportError:
    import platform

    if platform.system() == "Windows":
        from .windows_dll_manager import build_directory_manager, get_dll_paths

        with build_directory_manager() as dll_dir_manager:
            for p in get_dll_paths():
                dll_dir_manager.add_dll_directory(p)
            from .coal_pywrap_nb import *  # noqa
            from .coal_pywrap_nb import __version__  # noqa
    else:
        raise
