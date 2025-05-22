from os import environ
from time import asctime
from pathlib import Path
from subprocess import Popen, PIPE
from setuptools import setup, find_packages
from os.path import exists, dirname, join, abspath

PROJECT_DIR = Path(__file__).parent
version_file = "airsim/version.py"


def get_git_hash():

    def _minimal_ext_cmd(cmd):
        """
        Construct minimal environment
        """

        env = {}
        for k in ["SYSTEMROOT", "PATH", "HOME"]:
            v = environ.get(k)
            if v is not None:
                env[k] = v
        # LANGUAGE is used on win32
        env["LANGUAGE"] = 'C'
        env["LANG"] = 'C'
        env["LC_ALL"] = 'C'
        out = Popen(cmd, stdout = PIPE, env = env).communicate()[0]
        return out

    try:
        out = _minimal_ext_cmd(["git", "rev-parse", "HEAD"])
        sha = out.strip().decode("ascii")
    except OSError:
        sha = "unknown"

    return sha


def get_hash():
    if exists(".git"):
        sha = get_git_hash()[:7]
    else:
        sha = "unknown"

    return sha


def write_version_py():
    content = """# GENERATED VERSION FILE\n \
# TIME: {}
__version__ = "{}"
__gitsha__ = "{}"
version_info = ({})
"""
    
    sha = get_hash()
    with open("VERSION.txt", 'r') as f:
        SHORT_VERSION = f.read().strip()
    VERSION_INFO = ", ".join([x if x.isdigit() else f"\"{x}\"" for x in SHORT_VERSION.split('.')])

    version_file_str = content.format(asctime(), SHORT_VERSION, sha, VERSION_INFO)
    with open(version_file, 'w') as f:
        f.write(version_file_str)


def get_version():
    version_file_path = join(abspath(dirname(__file__)), version_file)
    with open(version_file_path, "r") as f:
        for line in f:
            if line.startswith("__version__"):
                delim = '"' if '"' in line else "'"
                return line.split(delim)[1]
        else:
            raise RuntimeError("Version string not found")
        

def get_requirements():
    reqs = []
    requirements_file = PROJECT_DIR / "requirements.txt"

    for line in requirements_file.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        # Split for packet and marker:
        parts = line.split(';', 1)
        dep = parts[0].strip()
        marker = parts[1].strip() if len(parts) > 1 else ''
        pkg_base = dep.split('@')[0].split('[')[0].strip()  # Базовое имя зависимости

        # VCS-links processing:
        vcs_link = dep[len(pkg_base):]
        link_start = vcs_link.find("git+")
        if link_start != -1:
            link = vcs_link[link_start:]  # Link
            dep = f"{pkg_base}@{link}"

        # New markers processing:
        marker = marker.replace("platform_system", "sys_platform")
        marker = marker.replace("Linux", "linux")
        marker = marker.replace("Windows", "win32")

        if marker:
            dep += f" ; {marker}"
        
        reqs.append(dep)
    
    return reqs



if __name__ == "__main__":

    if not exists(version_file):
        write_version_py()

    version = get_version()  # Get version after file created

    with open("README.md", "r") as fh:
        long_description = fh.read()

    setup(
        name = "airsim",
        version = version,
        author = "Shital Shah",
        author_email = "shitals@microsoft.com",
        description = "Open source simulator based on Unreal Engine for autonomous vehicles from Microsoft AI & Research",
        long_description = long_description,
        long_description_content_type = "text/markdown",
        url = "https://github.com/microsoft/airsim",
        packages = find_packages(),
        license = "MIT",
        classifiers = (
            "Programming Language :: Python :: 3",
            "License :: OSI Approved :: MIT License",
            "Operating System :: OS Independent",
        ),
        install_requires = get_requirements()
    )