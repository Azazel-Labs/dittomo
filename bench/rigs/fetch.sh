#!/bin/sh
# Downloads the rigs `cargo bench --features fs --bench rigs` measures mapping on, into bench/rigs/data (or the
# directory given), each from a pinned commit and checked against its SHA-256.
#
# They're fetched, not committed: each keeps its own license, as its source says.
#   three.js examples (MIT repository; Xbot, Michelle, Soldier and Samba Dancing come from Mixamo; RobotExpressive
#     from Tomás Laulhé, CC0; readyplayer.me.glb from Ready Player Me; the archer and warrior are 3ds Max
#     Bipeds)
#   Khronos glTF-Sample-Assets: CesiumMan, RiggedFigure, BrainStem (see each model's README for its license)
#   VRM Consortium's Seed-san sample and pixiv three-vrm's constraint sample (the license is in each file's VRM meta)
#   Babylon.js Assets: HVGirl
#   Mesh2Motion: its human rig (CC0)
#   BoQsc's Godot male base mesh, a Rigify metarig (CC0)
#   Godot demo projects: the platformer's robot (MIT), and GDQuest's Mannequiny (CC-BY 4.0, GDQuest and
#     contributors, https://www.gdquest.com/)
#   CMU Graphics Lab Motion Capture Database (http://mocap.cs.cmu.edu/, free for research and commercial use), as
#     B. Hahne's MotionBuilder-friendly BVH conversion, from una-dinosauria's copy
set -eu

dir=${1:-"$(dirname "$0")/data"}
mkdir -p "$dir"

three=https://raw.githubusercontent.com/mrdoob/three.js/b17ad2558855dd789ab09f1dd07d6ae4d7fff086/examples/models
khronos=https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea/Models
vrmc=https://raw.githubusercontent.com/vrm-c/vrm-specification/94e82dd346fa6cf0337c4421728640e5252dd38e/samples
pixiv=https://raw.githubusercontent.com/pixiv/three-vrm/3caf7a21ee68f0d5dfa363ab3d577450bbf60136/packages/three-vrm/examples/models
babylon=https://raw.githubusercontent.com/BabylonJS/Assets/6946e9be89e0d471d5caba0d2338b4c1ada250d0/meshes
m2m=https://raw.githubusercontent.com/Mesh2Motion/mesh2motion-app/2609e0345371763c4c390f751d79021285da04b1/static/assets
boqsc=https://raw.githubusercontent.com/BoQsc/Godot-Male-Base-Mesh/e7807327a8ebcc637831995a231dae701b6d4f56
godot=https://raw.githubusercontent.com/godotengine/godot-demo-projects/281a99f02178db944bcddef39aa694e3bfbe354a/3d
cmu=https://raw.githubusercontent.com/una-dinosauria/cmu-mocap/09a07f54f3bbb58797325f009282d0b2048a2871/data

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
fetch archer.fbx "$three/fbx/archer/ArcherRi01.fbx" f926e7439bcd6e733b05e6f7fa18aae011cfeed424d24ae5f4ca2fe434ab7115
fetch warrior.fbx "$three/fbx/warrior/Warrior.fbx" f00e5b65fface7d30beeb96b7469f129809810020b694af53c7e25affcb6762c
fetch samba-dancing.fbx "$three/fbx/Samba%20Dancing.fbx" b9003ee562c87bf03051c3a502411b0808d3513f1d74a2011f7530d9f067069f
fetch cesium-man.glb "$khronos/CesiumMan/glTF-Binary/CesiumMan.glb" b7001eaeea8254bd44773bcd247e78696d94169388fbb2a1800fc69434e777d9
fetch rigged-figure.glb "$khronos/RiggedFigure/glTF-Binary/RiggedFigure.glb" d6be85417d3e256861ee733eea6916093a7af7c79c16366181fd8abcaeb38cf5
fetch brainstem.glb "$khronos/BrainStem/glTF-Binary/BrainStem.glb" ddda3a53b5c9771f992463034a430340ba2bac571e5869c723f7cba4976e9174
fetch seed-san.vrm "$vrmc/Seed-san/vrm/Seed-san.vrm" 624d0d554bc205bbdc33e22a68a2c3c20edebb3e573011ead8878a65e5329b23
fetch vrm-twist.vrm "$pixiv/VRM1_Constraint_Twist_Sample.vrm" 12c2b97e95e700783a6a550dc0eee2d7880aeedccef9ae67bc4c5a2f0f2631a2
fetch hvgirl.glb "$babylon/HVGirl.glb" ca7840ac8726d771ea75ad49df05e3d404a0654c5a8bf96edf2018802c37f5d7
fetch mesh2motion-human.glb "$m2m/human/model.glb" bce15f56f0ca4daf4e085337b2412116ab18d3165cb51ea672d1cb9f1194b37f
fetch godot-robot.glb "$godot/platformer/player/player.glb" 2c7e11f0d7e84bac8a45d724eab05d664a0a4ba41360ff2641e73635e2b5acf6
fetch mannequiny.glb "$godot/ragdoll_physics/characters/mannequiny.glb" 6b0273d9e8eef159aee9cbc0e75752bbe6abbea56fbcb1a8534e6173f80dda3c
fetch male-base-mesh.glb "$boqsc/Original/male_base_mesh.glb" 206cb651484baf9aaa29a030a38feecbe29575c3e1cbd820a58fa343c2ae14c4
fetch cmu-mocap.bvh "$cmu/002/02_03.bvh" 6f4d38b92b041221030de2e1058903e0f7a8fa08f9c6c9efabf7df9a128011ac
echo "19 rigs in $dir"
