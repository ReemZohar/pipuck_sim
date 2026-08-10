#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
build_dir="${script_dir}/build"

cmake -S "${project_dir}" -B "${build_dir}" -DBUILD_TESTING=ON
cmake --build "${build_dir}"

# Read the generated list and execute each test sequentially
while IFS= read -r test_executable; do
    "${test_executable}"
done < "${build_dir}/tests/test_executables.txt"
