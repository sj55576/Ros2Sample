#!/usr/bin/env python3
"""Consistency checker between documentation and implementation.

Compares the actual ROS 2 packages under ``src/`` against the root
README and the specification docs, and exits non-zero when they have
drifted apart. Intended to run in CI as
``python3 scripts/check_docs_consistency.py``.

Checks:

1. Every package under ``src/`` (a directory with package.xml) is
   listed in the root README.md; simulation packages must also appear
   in docs/simulation_spec.md.
2. Every ``config/*.yaml`` in a package is referenced from a launch
   file or the package README (detects unwired "orphan" configs).
3. Every executable registered in setup.py ``console_scripts`` is
   mentioned in the package README.
"""

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = REPO_ROOT / 'src'

# チュートリアル用パッケージなど、シミュレーション仕様書
# (docs/simulation_spec.md) への記載を必須としないパッケージ。
SPEC_EXEMPT_PACKAGES = {'ros2_learning'}

ENTRY_POINT_RE = re.compile(r"'(?P<name>[\w-]+)\s*=\s*[\w.]+:[\w.]+'")


def find_packages():
    """Return the directories under src/ that contain a package.xml."""
    return sorted(
        p.parent for p in SRC_DIR.glob('*/package.xml')
    )


def check_package_listed(pkg_dirs, errors):
    """Check that every package is listed in the README and the spec."""
    readme = (REPO_ROOT / 'README.md').read_text(encoding='utf-8')
    spec = (REPO_ROOT / 'docs' / 'simulation_spec.md').read_text(
        encoding='utf-8')
    for pkg_dir in pkg_dirs:
        name = pkg_dir.name
        if f'`{name}`' not in readme:
            errors.append(
                f'README.md: パッケージ `{name}` が記載されていません')
        if name in SPEC_EXEMPT_PACKAGES:
            continue
        if f'`{name}`' not in spec:
            errors.append(
                f'docs/simulation_spec.md: パッケージ `{name}` が'
                '収録パッケージとして記載されていません')


def check_config_referenced(pkg_dirs, errors):
    """Check that every config/*.yaml is referenced by launch or README."""
    for pkg_dir in pkg_dirs:
        config_dir = pkg_dir / 'config'
        if not config_dir.is_dir():
            continue
        references = ''
        for launch_file in pkg_dir.glob('launch/*.py'):
            references += launch_file.read_text(encoding='utf-8')
        pkg_readme = pkg_dir / 'README.md'
        if pkg_readme.is_file():
            references += pkg_readme.read_text(encoding='utf-8')
        for yaml_file in sorted(config_dir.glob('*.yaml')):
            if yaml_file.name not in references:
                errors.append(
                    f'{pkg_dir.name}: config/{yaml_file.name} が launch '
                    'ファイルからもパッケージ README からも参照されていません')


def check_executables_documented(pkg_dirs, errors):
    """Check that every console_scripts executable is documented."""
    for pkg_dir in pkg_dirs:
        setup_py = pkg_dir / 'setup.py'
        pkg_readme = pkg_dir / 'README.md'
        if not setup_py.is_file() or not pkg_readme.is_file():
            continue
        readme_text = pkg_readme.read_text(encoding='utf-8')
        for match in ENTRY_POINT_RE.finditer(
                setup_py.read_text(encoding='utf-8')):
            executable = match.group('name')
            if executable not in readme_text:
                errors.append(
                    f'{pkg_dir.name}: 実行ファイル `{executable}` が'
                    'パッケージ README で言及されていません')


def main():
    """Run all checks and return 1 when any drift is found."""
    pkg_dirs = find_packages()
    if not pkg_dirs:
        print('src/ 配下に ROS 2 パッケージが見つかりません', file=sys.stderr)
        return 1
    errors = []
    check_package_listed(pkg_dirs, errors)
    check_config_referenced(pkg_dirs, errors)
    check_executables_documented(pkg_dirs, errors)
    if errors:
        print('ドキュメントと実装の乖離が見つかりました:', file=sys.stderr)
        for error in errors:
            print(f'  - {error}', file=sys.stderr)
        return 1
    print(f'OK: {len(pkg_dirs)} パッケージのドキュメント整合性を確認しました')
    return 0


if __name__ == '__main__':
    sys.exit(main())
