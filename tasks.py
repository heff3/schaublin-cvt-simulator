from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

from invoke import Exit, task


REPO_ROOT = Path(__file__).resolve().parent
DOCKERFILE = REPO_ROOT / "docker" / "halcompile.Dockerfile"


def _fail(message: str) -> None:
    raise Exit(message, code=1)


def _require_docker() -> str:
    docker = shutil.which("docker")
    if docker is None:
        _fail("docker is not on PATH")
    return docker


def _resolve_component(component: str) -> Path:
    candidate = Path(component)
    if not candidate.is_absolute():
        candidate = REPO_ROOT / candidate

    resolved = candidate.resolve(strict=False)
    try:
        resolved.relative_to(REPO_ROOT)
    except ValueError:
        _fail(f"component must be inside repo root: {component}")

    if not resolved.is_file():
        _fail(f"component file is missing: {resolved}")
    return resolved


def _workspace_component(component_path: Path) -> str:
    relative_path = component_path.relative_to(REPO_ROOT)
    return f"/workspace/{relative_path.as_posix()}"


def _component_name(component_path: Path) -> str:
    match = re.search(
        r"^\s*component\s+([A-Za-z0-9_-]+)\b",
        component_path.read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    if match is None:
        _fail(f"unable to determine component name from {component_path}")
    return match.group(1)


def _prepare_compile_source(component_path: Path) -> tuple[Path, bool]:
    component_name = _component_name(component_path)
    expected_name = f"{component_name}.comp"
    if component_path.name == expected_name:
        return component_path, False

    staged_path = component_path.with_name(expected_name)
    if staged_path.exists():
        _fail(
            "cannot stage component for halcompile because the expected "
            f"filename already exists: {staged_path}"
        )

    shutil.copyfile(component_path, staged_path)
    return staged_path, True


def _build_image(docker: str, image: str) -> None:
    result = subprocess.run(
        [docker, "build", "-t", image, "-f", str(DOCKERFILE), str(REPO_ROOT)],
        check=False,
    )
    if result.returncode != 0:
        raise Exit(f"docker build failed for image {image}", code=result.returncode)


def _ensure_image(docker: str, image: str, rebuild: bool) -> None:
    if rebuild:
        _build_image(docker, image)
        return

    inspect_result = subprocess.run(
        [docker, "image", "inspect", image],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if inspect_result.returncode != 0:
        _build_image(docker, image)


@task
def halcompile(
    _context,
    component: str = "spindle_supervisor_2026-05-17.comp",
    image: str = "cvt-halcompile:latest",
    rebuild: bool = False,
    extra_args: str = "",
) -> None:
    if not DOCKERFILE.is_file():
        _fail(f"Dockerfile is missing: {DOCKERFILE}")

    component_path = _resolve_component(component)
    docker = _require_docker()
    _ensure_image(docker, image, rebuild)

    compile_path, remove_staged_source = _prepare_compile_source(component_path)

    docker_command = [
        docker,
        "run",
        "--rm",
        "-v",
        f"{REPO_ROOT}:/workspace",
        "-w",
        "/workspace",
    ]

    getuid = getattr(os, "getuid", None)
    getgid = getattr(os, "getgid", None)
    if callable(getuid) and callable(getgid):
        docker_command.extend(["--user", f"{getuid()}:{getgid()}"])

    workspace_component = _workspace_component(compile_path)
    shell_command = f"halcompile --compile {shlex.quote(workspace_component)}"
    if extra_args:
        shell_command = f"{shell_command} {extra_args}"

    docker_command.extend([image, "sh", "-lc", shell_command])
    try:
        result = subprocess.run(docker_command, check=False)
    finally:
        if remove_staged_source:
            compile_path.unlink(missing_ok=True)

    if result.returncode != 0:
        raise Exit(f"halcompile failed for {component_path.name}", code=result.returncode)