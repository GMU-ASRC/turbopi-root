# project file management for recording run information

# pyright: reportImplicitOverride=false

__version__ = "0.0.1"

import os
import re
import sys
import time
import shutil
import pathlib
import platform
import yaml


RE_CONTAINS_SEP = re.compile(r"[/\\]")
DEFAULT_HOME = pathlib.Path("/home/pi")
DEFAULT_PROJECT_BASEPATH = DEFAULT_HOME / 'logs'
LOGFILE_NAME = "running.log"
RUNINFO_NAME = "runinfo.yaml"
ARTIFACTS_DIR_NAME = "artifacts"

if hasattr(os, 'geteuid') and os.geteuid() == 0:
    os.umask(0o000)


def _NONE1(x):
    pass


def is_project_dir(path):
    return path.is_dir() and (path / RUNINFO_NAME).is_file()


def find_lastmodified_dir(basepath):
    basepath = pathlib.Path(basepath)
    projectdirs = [(child, os.path.getmtime(child)) for child in basepath.iterdir() if is_project_dir(child)]
    return max(projectdirs, key=lambda x: x[1])[0]


def check_if_writable(path):
    if not os.access(path, os.W_OK):
        msg = f"{path} could not be accessed. Check that you have permissions to write to it."
        raise PermissionError(msg)


def inquire_project(root=None):
    if root is None:
        root = DEFAULT_PROJECT_BASEPATH
    from InquirerPy import inquirer
    from InquirerPy.base import Choice
    from InquirerPy.separator import Separator
    projects = list(root.iterdir())
    if not list:
        return None
    projects = sorted(projects)
    newest = max(projects, key=lambda f: f.stat().st_mtime)
    projects.insert(0, Choice(newest, name=f"Suggested (newest): {newest.name}"))
    return inquirer.fuzzy(choices=projects, message="Select a project").execute()


def inquire_size(file: pathlib.Path, limit=300e6):  # 300 MB
    size = file.stat().st_size
    if size < limit:
        return
    mb = round(size / 1e6, 3)
    message = f"The file {file} is {mb} MB. Do you want to continue?"
    try:
        from InquirerPy import inquirer
        return not inquirer.confirm(message=message, default=True).execute()
    except ImportError:
        print(message)
        input("Press enter to continue, ctrl-c to cancel.")


def get_dict(obj):
    if hasattr(obj, "as_dict"):
        return obj.as_dict()
    if hasattr(obj, "asdict"):
        return obj.asdict()
    if isinstance(obj, dict):
        return obj
    if isinstance(obj, list):
        return {i: get_dict(val) for i, val in enumerate(obj)}
    if hasattr(obj, "__dict__"):
        return vars(obj)

    return obj


def get_config_dict(obj):
    if hasattr(obj, "as_config_dict"):
        return obj.as_config_dict()
    return get_dict(obj)


cache = {}


def ensure_dir_exists(path: os.PathLike, parents=True, exist_ok=True, **kwargs):
    global cache
    path = pathlib.Path(path)
    fullpath = path.resolve()
    key = str(fullpath)
    if not cache.get(key, False):
        path.mkdir(parents=parents, exist_ok=exist_ok, **kwargs)
        cache[key] = path.is_dir()


class File:
    def __init__(self, path: os.PathLike):
        self.path = pathlib.Path(path)

    def write(self, s):
        with open(self.path, 'w') as f:
            f.write(s)

    def append(self, s):
        with open(self.path, 'a') as f:
            f.write(s)

    def __add__(self, s):
        self.append(s)
        return self

    def __str__(self):
        return str(self.path)

    def as_config_dict(self):
        return self.as_dict()

    def as_dict(self):
        return {'path': str(self)}


class Logger(File):
    def __init__(self, path, firstcall=None):
        super().__init__(path)
        self._initialized = False
        self.firstcall = _NONE1 if firstcall is None else firstcall

    def append(self, s):
        if not self._initialized:
            self._initialized = True
            self.firstcall()
        super().append(s)

    def as_dict(self):
        d = super().as_dict()
        d.update({'firstcall': repr(self.firstcall)})
        return d


class FolderlessProject:
    isproj = False

    def __init__(self, logfile_path, name=None):
        self.name = name
        self.logfile = Logger(logfile_path)


class Project(FolderlessProject):
    isproj = True

    def __init__(self, name=None, path=None):
        self.name = name
        self.root = pathlib.Path(path) if path is not None else None
        if name is None and isinstance(path, pathlib.Path):
            # TODO: check if path contains runinfo.yaml and try to use name from that
            # try to determine name from path
            self.name = self.root
            raise NotImplementedError("TODO: determine project name from path")
        elif name is not None and path is None:
            # if name is given, use as project directory name
            self.root = pathlib.Path(DEFAULT_PROJECT_BASEPATH / name)
        self.logfile = Logger(self.root / LOGFILE_NAME)

    def _default_firstcall(self, f):
        f.write(f"{time.time()}\t{0}\t[]\n")

    def make_root_interactive(self):
        create_parents = False
        if self.root is None:
            raise ValueError("Project root is None. Cannot make root interactive.")

        creating_default_project = (
            self.root.parent == DEFAULT_PROJECT_BASEPATH  # it's the default project basepath
            and self.root.parent.resolve().expanduser().is_relative_to(DEFAULT_HOME)  # and it's in /home/pi
            and DEFAULT_HOME.is_dir()  # and /home/pi exists  (in case we're not on the pi)
        )

        if self.root.is_dir():
            s = input(f"Project folder already exists:\n\t{str(self.root)}\n'y' to continue, 'rm' to delete the contents of the folder, anything else to exit. ")  # noqa: E501
            if s.lower() not in ('y', 'yes', 'rm'):
                print("Exiting. Your filesystem has not been modified.")
                sys.exit(1)
            if s.lower() == 'rm':
                shutil.rmtree(self.root)   # type: ignore[reportArgumentType]
                print(f"Deleted {self.root}.")
        elif not self.root.parent.is_dir() and not creating_default_project:
            # if the place to put the project doesn't exist, we need to ask the user if they want to create it
            # unless we're creating the default project basepath, in which case skip asking and just create it
            print("WARNING: You're trying to put the project in")
            print(str(self.root.parent))
            print("but some part of it does not exist! Would you like to create it?")
            s = input("Type 'y' to create it, anything else to exit. ")
            if s.lower() not in ('y', 'yes'):
                print("Exiting. Your filesystem has not been modified.")
                sys.exit(1)
            else:
                create_parents = True
        print(f"Creating project folder at {self.root}")
        ensure_dir_exists(self.root, parents=create_parents)

    @property
    def logfile_path(self):
        return self.root / LOGFILE_NAME

    @property
    def runinfo_path(self):
        return self.root / RUNINFO_NAME

    def ensure_dir(self, relpath, parents=True, exist_ok=True, **kwargs):
        path = self.root / relpath
        ensure_dir_exists(path, parents=parents, exist_ok=exist_ok, **kwargs)
        return path

    def ensure_file_parents(self, relpath, parents=True, exist_ok=True, **kwargs):
        path = self.root / relpath
        ensure_dir_exists(path.parent, parents=parents, exist_ok=exist_ok, **kwargs)
        return path

    def save_yaml_artifact(self, name, obj):
        artifacts = pathlib.Path(ARTIFACTS_DIR_NAME)
        with open(self.ensure_file_parents(artifacts / name), "w", ) as f:
            dat = get_config_dict(obj)
            yaml.dump(dat, f)


def make_default_project(name_or_path, root=DEFAULT_PROJECT_BASEPATH, cls=Project, suffix='', hostname=None):
    # don't destroy reference to root
    if root is not DEFAULT_PROJECT_BASEPATH:
        root = DEFAULT_PROJECT_BASEPATH if root is None else pathlib.Path(root)
    if name_or_path is None:
        # no project name specified, so use the experiment name and timestamp
        name = f"{time.strftime('%y%m%d-%H%M%S')}"
        if suffix:
            name += f"-{suffix}"
        if hostname is None:
            name += f"-{platform.node()}"
        elif hostname:
            name += f"-{hostname}"
        path = root / name
    elif RE_CONTAINS_SEP.search(str(name_or_path)):  # project name contains a path separator
        name = pathlib.Path(name_or_path).name
        path = name_or_path
        if root is not DEFAULT_PROJECT_BASEPATH:
            print("WARNING: You seem to have specified a root path AND a full project path.")
            print(f"The root path will be ignored; path={path}")
    else:
        name = pathlib.Path(name_or_path)
        path = root / name
    return cls(path=path, name=name)
