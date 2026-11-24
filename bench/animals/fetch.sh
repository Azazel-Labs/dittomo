#!/bin/sh
# Downloads four-legged rigs into bench/animals/data (or the directory given), each from a pinned commit and checked
# against its SHA-256. They're kept apart from bench/rigs, whose bench scores human joints.
#
# They're fetched, not committed: each keeps its own licence, as its source says.
#   Khronos glTF-Sample-Assets: Fox (mesh CC0 by PixelMannen; rig and animation CC-BY 4.0 by @tomkranis; glTF by
#     @AsoboStudio and @scurest)
#   Quaternius's Ultimate Animated Animals, CC0, from a copy on GitHub: Husky
set -eu

dir=${1:-"$(dirname "$0")/data"}
mkdir -p "$dir"

khronos=https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea/Models
quaternius="https://raw.githubusercontent.com/laoniutoushx/TD-demo-2024-04-03/704c3b82c4aea39da90f524961df8dd33811a07b/Asserts/Models/ulitimate%20animals/glTF"

sum() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | cut -d' ' -f1
    else
        shasum -a 256 "$1" | cut -d' ' -f1
    fi
}

fetch() { # file url sha256
    out="$dir/$1"
    if [ -f "$out" ] && [ "$(sum "$out")" = "$3" ]; then
        return
    fi
    echo "fetching $1"
    curl -fsSL --retry 3 -o "$out.part" "$2"
    got=$(sum "$out.part")
    if [ "$got" != "$3" ]; then
        rm -f "$out.part"
        echo "$1: SHA-256 $got, expected $3" >&2
        exit 1
    fi
    mv "$out.part" "$out"
}

fetch fox.glb "$khronos/Fox/glTF-Binary/Fox.glb" d97044e701822bac5a62696459b27d7b375aada5de8574ed4362edbba94771f7
fetch husky.gltf "$quaternius/Husky.gltf" 0cfd85ddfcc5c07caf1bdeb296b21017b0da4c8558a92086daf4a172e0bf8d97
echo "2 animals in $dir"
