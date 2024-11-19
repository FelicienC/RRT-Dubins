"""This file runs the examples in the examples directory as integration tests."""

import os
import subprocess

import pytest

EXAMPLES_DIR = "examples"
example_files = [
    os.path.join(root, file)
    for root, _, files in os.walk(EXAMPLES_DIR)
    for file in files
    if file.endswith(".py")
]


@pytest.mark.parametrize("file", example_files)
def test_example(file):
    result = subprocess.run(
        ["python", file],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"Example {file} failed with error:\n{result.stderr}"
