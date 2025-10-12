#!/usr/bin/env python3
"""
Get Conda Package Version
==========================

Get the installed version of a conda package and output it for GitHub Actions.

Usage:
    python get_package_version.py <package_name>

Example:
    python get_package_version.py ros-humble-unilabos-msgs
"""

import json
import os
import subprocess
import sys


def get_package_version(package_name: str) -> str:
    """
    Get the installed version of a conda package.

    Args:
        package_name: Name of the package to check

    Returns:
        str: Version string or 'not-found'
    """
    try:
        # Use conda list with JSON output for reliable parsing
        result = subprocess.run(["conda", "list", package_name, "--json"], capture_output=True, text=True, check=True)

        packages = json.loads(result.stdout)

        if packages:
            version = packages[0]["version"]
            return version
        else:
            return "not-found"

    except (subprocess.CalledProcessError, json.JSONDecodeError, KeyError, IndexError) as e:
        print(f"Error getting package version: {e}", file=sys.stderr)
        return "error"


def main():
    """Main entry point."""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <package_name>", file=sys.stderr)
        sys.exit(1)

    package_name = sys.argv[1]
    version = get_package_version(package_name)

    # Output for GitHub Actions
    github_output = os.getenv("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a") as f:
            f.write(f"installed_version={version}\n")

    # Also print for logs
    print(f"Installed {package_name} version: {version}")


if __name__ == "__main__":
    main()
