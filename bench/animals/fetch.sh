#!/bin/sh
# Downloads four-legged rigs into bench/animals/data (or the directory given), each from a pinned commit and checked
# against its SHA-256. They're kept apart from bench/rigs, whose bench scores human joints.
#
# They're fetched, not committed: each keeps its own licence, as its source says.
#   Khronos glTF-Sample-Assets: Fox (mesh CC0 by PixelMannen; rig and animation CC-BY 4.0 by @tomkranis; glTF by
#     @AsoboStudio and @scurest)
#   Quaternius's Ultimate Animated Animals, CC0, from a copy on GitHub: Husky, Alpaca, Cow, Deer, Horse, Shiba Inu,
#     Stag, Wolf
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
fetch alpaca.gltf "$quaternius/Alpaca.gltf" 5009bfa756c7b7af5e02263d47a6d9088cadaada2240b41b4e87f31327b0e063
fetch cow.gltf "$quaternius/Cow.gltf" b04af067688d9fadf1395b4456dbf3117c2f760fb49e704ee5c8b6391fc8adb5
fetch deer.gltf "$quaternius/Deer.gltf" b8afd0647e7a74332ce802c8326dfefe1a2b76da67ab925a631aaddb5ae9bdd4
fetch horse.gltf "$quaternius/Horse.gltf" 3deb61550dff1d2786d04b6e8559d63ad3907d6ab606ba28ce0af074ed96341b
fetch shiba.gltf "$quaternius/ShibaInu.gltf" 3236979b94a840eb72d9ce091b1361da327a96f5ce068e666680297480f30f6b
fetch stag.gltf "$quaternius/Stag.gltf" 170b964909d16d1ab4d428b1d714f25016913d32481acd9c3d62923192583be6
fetch wolf.gltf "$quaternius/Wolf.gltf" cc02e9d128b5715f352ee8bea086f97a35f1d875d240de99b0f9f2775c37d415
echo "9 animals in $dir"
