#!/bin/sh
# Downloads the rigs `cargo bench --features fs --bench rigs` measures mapping on, into bench/rigs/data (or the
# directory given), each from a pinned commit and checked against its SHA-256.
#
# They're fetched, not committed: each keeps its own licence, as its source says.
#   three.js examples (MIT repository; Xbot, Michelle, Soldier and Samba Dancing come from Mixamo; RobotExpressive
#     from Tomás Laulhé, CC0; readyplayer.me.glb from Ready Player Me)
#   Khronos glTF-Sample-Assets: CesiumMan, RiggedFigure, BrainStem (see each model's README for its licence)
#   VRM Consortium's Seed-san sample and pixiv three-vrm's constraint sample (the licence is in each file's VRM meta)
#   Babylon.js Assets: HVGirl
set -eu

dir=${1:-"$(dirname "$0")/data"}
mkdir -p "$dir"

three=https://raw.githubusercontent.com/mrdoob/three.js/b17ad2558855dd789ab09f1dd07d6ae4d7fff086/examples/models
khronos=https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea/Models
vrmc=https://raw.githubusercontent.com/vrm-c/vrm-specification/94e82dd346fa6cf0337c4421728640e5252dd38e/samples
pixiv=https://raw.githubusercontent.com/pixiv/three-vrm/3caf7a21ee68f0d5dfa363ab3d577450bbf60136/packages/three-vrm/examples/models
babylon=https://raw.githubusercontent.com/BabylonJS/Assets/6946e9be89e0d471d5caba0d2338b4c1ada250d0/meshes

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

fetch xbot.glb "$three/gltf/Xbot.glb" 002f8d269de68e5dce3d25195caf390d1aa359bbfaae3fcf4c8dc78ec36c3ba5
fetch michelle.glb "$three/gltf/Michelle.glb" 7a87e15a99ccbc5e5877be66e1e4ecae0a581adcafa0cce1a5569f49909e968e
fetch soldier.glb "$three/gltf/Soldier.glb" dfb230fc1f942f259dd00281a1186953ad602fc5d69067ce63e24b2aa439736b
fetch robot-expressive.glb "$three/gltf/RobotExpressive/RobotExpressive.glb" 047f5e5fb3bb6d378bd1df16ca6137f2a596c99b3a1b5690b4020c05aaf6f319
fetch readyplayerme.glb "$three/gltf/readyplayer.me.glb" b69cec7a5cc7dc7e1c1f19c324024d090ae06e1315441a7bce4bf236112addc1
fetch samba-dancing.fbx "$three/fbx/Samba%20Dancing.fbx" b9003ee562c87bf03051c3a502411b0808d3513f1d74a2011f7530d9f067069f
fetch cesium-man.glb "$khronos/CesiumMan/glTF-Binary/CesiumMan.glb" b7001eaeea8254bd44773bcd247e78696d94169388fbb2a1800fc69434e777d9
fetch rigged-figure.glb "$khronos/RiggedFigure/glTF-Binary/RiggedFigure.glb" d6be85417d3e256861ee733eea6916093a7af7c79c16366181fd8abcaeb38cf5
fetch brainstem.glb "$khronos/BrainStem/glTF-Binary/BrainStem.glb" ddda3a53b5c9771f992463034a430340ba2bac571e5869c723f7cba4976e9174
fetch seed-san.vrm "$vrmc/Seed-san/vrm/Seed-san.vrm" 624d0d554bc205bbdc33e22a68a2c3c20edebb3e573011ead8878a65e5329b23
fetch vrm-twist.vrm "$pixiv/VRM1_Constraint_Twist_Sample.vrm" 12c2b97e95e700783a6a550dc0eee2d7880aeedccef9ae67bc4c5a2f0f2631a2
fetch hvgirl.glb "$babylon/HVGirl.glb" ca7840ac8726d771ea75ad49df05e3d404a0654c5a8bf96edf2018802c37f5d7
echo "12 rigs in $dir"
