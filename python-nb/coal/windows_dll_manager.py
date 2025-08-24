import contextlib
import os
import sys


def get_dll_paths():
    coal_paths = os.getenv("COAL_WINDOWS_DLL_PATH")
    if coal_paths is None:
        # From https://peps.python.org/pep-0250/#implementation
        # lib/python-version/site-packages/package
        RELATIVE_DLL_PATH1 = "..\\..\\..\\..\\bin"
        # lib/site-packages/package
        RELATIVE_DLL_PATH2 = "..\\..\\..\\bin"
        # For unit test
        RELATIVE_DLL_PATH3 = "..\\..\\bin"
        return [
            os.path.join(os.path.dirname(__file__), RELATIVE_DLL_PATH1),
            os.path.join(os.path.dirname(__file__), RELATIVE_DLL_PATH2),
            os.path.join(os.path.dirname(__file__), RELATIVE_DLL_PATH3),
        ]
    else:
        return coal_paths.split(os.pathsep)


class PathManager(contextlib.AbstractContextManager):
    """Restore PATH state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        os.environ["PATH"] += os.pathsep + dll_dir

    def __enter__(self):
        self.old_path = os.environ["PATH"]
        return self

    def __exit__(self, *exc_details):
        os.environ["PATH"] = self.old_path


class DllDirectoryManager(contextlib.AbstractContextManager):
    """Restore DllDirectory state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        # add_dll_directory can fail on relative path and non
        # existing path.
        # Since we don't know all the fail criterion we just ignore
        # thrown exception
        try:
            self.dll_dirs.append(os.add_dll_directory(dll_dir))
        except OSError:
            pass

    def __enter__(self):
        self.dll_dirs = []
        return self

    def __exit__(self, *exc_details):
        for d in self.dll_dirs:
            d.close()


def build_directory_manager():
    if sys.version_info >= (3, 8):
        return DllDirectoryManager()
    else:
        return PathManager()
