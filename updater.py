"""Check GitHub releases and safely fast-forward a cloned installation."""

import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from PyQt6.QtCore import QThread, pyqtSignal

import config


def _version_tuple(version):
    """Parse the stable three-part version format used by GitHub release tags."""
    match = re.fullmatch(r"v?(\d+)\.(\d+)\.(\d+)", version.strip())
    if not match:
        raise ValueError(f"Unsupported version tag: {version}")
    return tuple(int(part) for part in match.groups())


def _git(*arguments, timeout=10):
    """Run Git without a shell so paths and arguments remain unambiguous."""
    return subprocess.run(
        ["git", *arguments],
        cwd=Path(__file__).resolve().parent,
        capture_output=True,
        text=True,
        env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
        timeout=timeout,
        check=False,
    )


def _validate_checkout():
    """Require the expected clone; never let an update target an arbitrary repo."""
    if not shutil.which("git"):
        raise RuntimeError("Git was not found. Install Git for Windows and try again.")

    root = Path(__file__).resolve().parent
    result = _git("rev-parse", "--show-toplevel")
    if result.returncode != 0:
        raise RuntimeError("This copy is not inside a Git clone; automatic updates are unavailable.")
    if Path(result.stdout.strip()).resolve() != root:
        raise RuntimeError("The app folder is not the root of its Git clone.")

    remote = _git("remote", "get-url", config.UPDATE_REMOTE)
    if remote.returncode != 0 or not _remote_matches_repository(remote.stdout.strip()):
        raise RuntimeError(
            f"Git remote '{config.UPDATE_REMOTE}' must point to "
            f"https://github.com/{config.GITHUB_REPOSITORY}.git."
        )
    return root


def _remote_matches_repository(remote_url):
    """Accept standard GitHub HTTPS and SSH clone URLs for this repository."""
    remote_url = remote_url.strip().lower()
    if remote_url.startswith("git@github.com:"):
        repository = remote_url.split(":", 1)[1]
    elif remote_url.startswith(("https://github.com/", "http://github.com/")):
        repository = remote_url.split("github.com/", 1)[1]
    else:
        return False
    return repository.removesuffix(".git").rstrip("/") == config.GITHUB_REPOSITORY.lower()


def _get_latest_release_version():
    """Read the latest stable release; GitHub's latest endpoint excludes prereleases."""
    url = f"https://api.github.com/repos/{config.GITHUB_REPOSITORY}/releases/latest"
    request = Request(
        url,
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": "VibroApp-Updater",
        },
    )
    try:
        with urlopen(request, timeout=10) as response:
            release = json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        if error.code == 404:
            raise RuntimeError("No published stable GitHub release was found yet.") from error
        if error.code == 403:
            raise RuntimeError("GitHub temporarily limited update checks. Try again later.") from error
        raise RuntimeError(f"GitHub returned HTTP {error.code} while checking for updates.") from error
    except URLError as error:
        raise RuntimeError(f"Could not connect to GitHub: {error.reason}") from error
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError("GitHub returned an invalid release response.") from error

    tag = release.get("tag_name")
    if not isinstance(tag, str):
        raise RuntimeError("The latest GitHub release does not have a valid version tag.")
    return tag


def check_for_update():
    """Return (status, version, message) for the latest stable release."""
    _validate_checkout()
    latest_version = _get_latest_release_version()
    if _version_tuple(latest_version) > _version_tuple(config.APP_VERSION):
        return "available", latest_version, "A newer stable version is available."
    return "current", latest_version, "You are using the latest stable version."


def install_update():
    """Fast-forward the clean main-branch clone to the published update."""
    _validate_checkout()

    branch = _git("branch", "--show-current")
    if branch.returncode != 0 or branch.stdout.strip() != config.UPDATE_BRANCH:
        raise RuntimeError(
            f"Updates require the '{config.UPDATE_BRANCH}' branch. "
            "Re-clone the repository on its main branch to enable updates."
        )

    changes = _git("status", "--porcelain", "--untracked-files=normal")
    if changes.returncode != 0:
        raise RuntimeError(changes.stderr.strip() or "Could not check the Git working tree.")
    if changes.stdout.strip():
        raise RuntimeError(
            "Your clone has local changes. Commit, stash, or back them up before updating."
        )

    result = _git(
        "pull", "--ff-only", config.UPDATE_REMOTE, config.UPDATE_BRANCH, timeout=60
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(detail or "Git could not fast-forward the repository.")
    return result.stdout.strip() or "The repository is already up to date."


class UpdateWorker(QThread):
    """Keep GitHub and Git operations off the Qt dialog's UI thread."""

    check_finished = pyqtSignal(str, str, str)
    update_finished = pyqtSignal(bool, str)

    def __init__(self, action):
        super().__init__()
        self.action = action

    def run(self):
        if self.action == "check":
            try:
                status, version, message = check_for_update()
            except Exception as error:
                self.check_finished.emit("error", "", str(error))
            else:
                self.check_finished.emit(status, version, message)
            return

        try:
            message = install_update()
        except Exception as error:
            self.update_finished.emit(False, str(error))
        else:
            self.update_finished.emit(True, message)