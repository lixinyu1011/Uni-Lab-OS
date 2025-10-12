#!/usr/bin/env python3
"""
UniLabOS Installation Verification Script
=========================================

This script verifies that UniLabOS and its dependencies are correctly installed.
Run this script after installing the conda-pack environment to ensure everything works.

Usage:
    python verify_installation.py

    Or in the conda environment:
    conda activate unilab
    python verify_installation.py
"""

import sys
import importlib


def check_package(package_name: str, display_name: str = None) -> bool:
    """
    Check if a package can be imported.

    Args:
        package_name: Name of the package to import
        display_name: Display name (defaults to package_name)

    Returns:
        bool: True if package is available
    """
    if display_name is None:
        display_name = package_name

    try:
        importlib.import_module(package_name)
        print(f"  ✓ {display_name}")
        return True
    except ImportError:
        print(f"  ✗ {display_name}")
        return False


def check_python_version() -> bool:
    """Check Python version."""
    version = sys.version_info
    version_str = f"{version.major}.{version.minor}.{version.micro}"

    if version.major == 3 and version.minor >= 11:
        print(f"  ✓ Python {version_str}")
        return True
    else:
        print(f"  ✗ Python {version_str} (requires Python 3.8+)")
        return False


def main():
    """Run all verification checks."""
    print("=" * 60)
    print("UniLabOS Installation Verification")
    print("=" * 60)
    print()

    all_passed = True

    # Check Python version
    print("Checking Python version...")
    if not check_python_version():
        all_passed = False
    print()

    # Check ROS2 rclpy
    print("Checking ROS2 rclpy...")
    if not check_package("rclpy", "ROS2 rclpy"):
        all_passed = False
    print()

    # Run environment checker from unilabos
    print("Checking UniLabOS and dependencies...")
    try:
        from unilabos.utils.environment_check import EnvironmentChecker

        print("  ✓ UniLabOS installed")

        checker = EnvironmentChecker()
        env_check_passed = checker.check_all_packages()

        if env_check_passed:
            print("  ✓ All required packages available")
        else:
            print(f"  ✗ Missing {len(checker.missing_packages)} package(s):")
            for import_name, _ in checker.missing_packages:
                print(f"    - {import_name}")
            all_passed = False
    except ImportError:
        print("  ✗ UniLabOS not installed")
        all_passed = False
    except Exception as e:
        print(f"  ✗ Environment check failed: {str(e)}")
        all_passed = False
    print()

    # Summary
    print("=" * 60)
    print("Verification Summary")
    print("=" * 60)

    if all_passed:
        print("\n✓ All checks passed! Your UniLabOS installation is ready.")
        print("\nNext steps:")
        print("  1. Review the documentation: docs/user_guide/launch.md")
        print("  2. Try the examples: docs/boot_examples/")
        print("  3. Configure your devices: unilabos_data/startup_config.json")
        return 0
    else:
        print("\n✗ Some checks failed. Please review the errors above.")
        print("\nTroubleshooting:")
        print("  1. Ensure you're in the correct conda environment: conda activate unilab")
        print("  2. Check the installation documentation: docs/user_guide/installation.md")
        print("  3. Try reinstalling: pip install -e .")
        return 1


if __name__ == "__main__":
    sys.exit(main())
