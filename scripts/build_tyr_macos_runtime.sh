#!/usr/bin/env bash
# Build the two non-system dylibs needed by the Tyr Apple Silicon release.
# Requires macOS, Xcode command-line tools, CMake, Ninja, curl and shasum.
# Usage: scripts/build_tyr_macos_runtime.sh [--output DIR] [--jobs N]
# Default output: <repository>/build/tyr-runtime. All downloads, extracted
# sources, dependency build caches, libraries and receipts stay in that folder.
# No app, main build cache, Homebrew installation or system directory is changed.
#
# The source archives are pinned below. Compiler/SDK versions are recorded in
# build-environment.txt; this is a repeatable recipe, not a bit-identical build
# guarantee across different Xcode releases. Outputs target arm64/macOS 14.0.
#
# Packaging inputs:
#   <output>/lib/libzstd.1.dylib
#   <output>/lib/libImath-3_2.30.dylib
#   <output>/licenses/zstd/{LICENSE,COPYING,build-LICENSE}
#   <output>/licenses/Imath/LICENSE.md
# Retain downloads/, source-manifest.txt and this script with release sources.

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
runtime_dir="$repo_dir/build/tyr-runtime"
jobs=4

usage() {
    awk 'NR == 1 {next} /^#/ {sub(/^# */, ""); print; next} {exit}' "${BASH_SOURCE[0]}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output)
            [[ $# -ge 2 && -n "$2" ]] || { echo "--output requires a directory" >&2; exit 2; }
            runtime_dir="$2"
            shift 2
            ;;
        --jobs)
            [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || { echo "--jobs requires a positive integer" >&2; exit 2; }
            jobs="$2"
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ "$(uname -s)" == Darwin ]] || { echo "This build requires macOS." >&2; exit 1; }
for tool in cmake ninja curl shasum xcrun tar lipo otool; do
    command -v "$tool" >/dev/null || { echo "Missing required tool: $tool" >&2; exit 1; }
done

mkdir -p "$runtime_dir"
runtime_dir="$(cd "$runtime_dir" && pwd -P)"
mkdir -p "$runtime_dir/downloads" "$runtime_dir/sources" "$runtime_dir/lib" \
    "$runtime_dir/licenses/zstd" "$runtime_dir/licenses/Imath"

zstd_url='https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz'
zstd_sha256='eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3'
imath_url='https://codeload.github.com/AcademySoftwareFoundation/Imath/tar.gz/refs/tags/v3.2.3'
imath_sha256='e10c12b3f21f45bf08e09d4215d9c7691368d747beebd840de0b6fefed2df9f8'

fetch_verified() {
    local url="$1" expected="$2" archive="$3" actual
    if [[ ! -f "$archive" ]]; then
        curl --fail --location --retry 3 --output "$archive.download" "$url"
        actual="$(shasum -a 256 "$archive.download" | awk '{print $1}')"
        [[ "$actual" == "$expected" ]] || { echo "Checksum mismatch: $url" >&2; exit 1; }
        mv "$archive.download" "$archive"
    fi
    actual="$(shasum -a 256 "$archive" | awk '{print $1}')"
    [[ "$actual" == "$expected" ]] || { echo "Checksum mismatch: $archive" >&2; exit 1; }
}

fetch_verified "$zstd_url" "$zstd_sha256" "$runtime_dir/downloads/zstd-1.5.7.tar.gz"
fetch_verified "$imath_url" "$imath_sha256" "$runtime_dir/downloads/Imath-v3.2.3.tar.gz"

cat > "$runtime_dir/source-manifest.txt" <<EOF
zstd 1.5.7
URL: $zstd_url
SHA256: $zstd_sha256
Archive: downloads/zstd-1.5.7.tar.gz
License: licenses/zstd/LICENSE (BSD), licenses/zstd/COPYING (GPL alternative)

Imath 3.2.3
URL: $imath_url
SHA256: $imath_sha256
Archive: downloads/Imath-v3.2.3.tar.gz
License: licenses/Imath/LICENSE.md (BSD-3-Clause)
EOF

# Extract only verified archives. Re-extraction restores upstream source files
# on repeated invocations while retaining these dependency-specific build caches.
tar -xzf "$runtime_dir/downloads/zstd-1.5.7.tar.gz" -C "$runtime_dir/sources"
tar -xzf "$runtime_dir/downloads/Imath-v3.2.3.tar.gz" -C "$runtime_dir/sources"

sdk="$(xcrun --sdk macosx --show-sdk-path)"
cc="$(xcrun --sdk macosx --find clang)"
cxx="$(xcrun --sdk macosx --find clang++)"
{
    date -u '+Build UTC: %Y-%m-%dT%H:%M:%SZ'
    sw_vers
    cmake --version
    ninja --version
    "$cc" --version
    printf 'SDK: %s\nArchitecture: arm64\nDeployment target: 14.0\n' "$sdk"
} > "$runtime_dir/build-environment.txt"

common_args=(
    -G Ninja
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_C_COMPILER="$cc"
    -DCMAKE_CXX_COMPILER="$cxx"
    -DCMAKE_C_FLAGS=
    -DCMAKE_CXX_FLAGS=
    -DCMAKE_OSX_ARCHITECTURES=arm64
    -DCMAKE_OSX_DEPLOYMENT_TARGET=14.0
    -DCMAKE_OSX_SYSROOT="$sdk"
    -DCMAKE_INSTALL_PREFIX="$runtime_dir/install"
    -DCMAKE_INSTALL_NAME_DIR=@rpath
    -DCMAKE_BUILD_WITH_INSTALL_NAME_DIR=ON
    -DCMAKE_INSTALL_RPATH=@loader_path
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    -DBUILD_SHARED_LIBS=ON
)

cmake -S "$runtime_dir/sources/zstd-1.5.7/build/cmake" -B "$runtime_dir/build/zstd" \
    "${common_args[@]}" \
    -DZSTD_BUILD_SHARED=ON -DZSTD_BUILD_STATIC=OFF -DZSTD_BUILD_PROGRAMS=OFF \
    -DZSTD_BUILD_TESTS=OFF -DZSTD_BUILD_CONTRIB=OFF \
    2>&1 | tee "$runtime_dir/zstd-configure.log"
cmake --build "$runtime_dir/build/zstd" --target libzstd_shared --parallel "$jobs" \
    2>&1 | tee "$runtime_dir/zstd-build.log"

cmake -S "$runtime_dir/sources/Imath-3.2.3" -B "$runtime_dir/build/imath" \
    "${common_args[@]}" \
    -DBUILD_TESTING=OFF -DPYTHON=OFF -DPYBIND11=OFF -DBUILD_WEBSITE=OFF \
    -DIMATH_BUILD_APPLE_FRAMEWORKS=OFF \
    2>&1 | tee "$runtime_dir/imath-configure.log"
cmake --build "$runtime_dir/build/imath" --target Imath --parallel "$jobs" \
    2>&1 | tee "$runtime_dir/imath-build.log"

# Copy real files under the ABI names used by the app; no system install runs.
cp -L "$runtime_dir/build/zstd/lib/libzstd.1.dylib" "$runtime_dir/lib/libzstd.1.dylib"
cp -L "$runtime_dir/build/imath/src/Imath/libImath-3_2.30.dylib" "$runtime_dir/lib/libImath-3_2.30.dylib"
cp "$runtime_dir/sources/zstd-1.5.7/LICENSE" "$runtime_dir/licenses/zstd/LICENSE"
cp "$runtime_dir/sources/zstd-1.5.7/COPYING" "$runtime_dir/licenses/zstd/COPYING"
cp "$runtime_dir/sources/zstd-1.5.7/build/LICENSE" "$runtime_dir/licenses/zstd/build-LICENSE"
cp "$runtime_dir/sources/Imath-3.2.3/LICENSE.md" "$runtime_dir/licenses/Imath/LICENSE.md"

for library in "$runtime_dir/lib/libzstd.1.dylib" "$runtime_dir/lib/libImath-3_2.30.dylib"; do
    [[ "$(lipo -archs "$library")" == arm64 ]] || { echo "Unexpected architecture: $library" >&2; exit 1; }
    [[ "$(otool -l "$library" | awk '$1 == "minos" {print $2}')" == 14.0 ]] || { echo "Unexpected deployment target: $library" >&2; exit 1; }
    [[ "$(otool -D "$library" | tail -n 1)" == "@rpath/$(basename "$library")" ]] || { echo "Unexpected install name: $library" >&2; exit 1; }
    while IFS= read -r dependency; do
        case "$dependency" in
            /usr/lib/*|/System/Library/*) ;;
            *) echo "Unexpected external dependency: $dependency" >&2; exit 1 ;;
        esac
    done < <(otool -L "$library" | awk 'NR > 2 {print $1}')
    {
        file "$library"
        otool -L "$library"
        otool -l "$library"
    } > "$library.audit.txt"
done

(cd "$runtime_dir" && shasum -a 256 lib/*.dylib > SHA256SUMS)
printf 'Runtime libraries and license notices are ready in %s\n' "$runtime_dir"
