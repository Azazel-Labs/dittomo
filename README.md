<p align="center"><img src="assets/brand/hero.png" alt="dittomo: one motion, any body" width="100%"></p>

# dittomo

**One motion. Any body.** dittomo retargets an animation from one skeleton onto any other. The more alike the two rigs,
the closer the copy, and a rig onto itself plays its clip unchanged. It keeps what makes a performance read as the same:
where the character looks, where its hands rest on its body, and where its feet land.

<p align="center"><img src="assets/brand/demo.gif" alt="A samba danced by Michelle, and the same samba played by dittomo on a Ready Player Me avatar" width="640"></p>

Pure Rust, no `unsafe`, deterministic, MIT-0. Its only dependency is [glam](https://crates.io/crates/glam); reading
model files adds [polysmelt](https://github.com/Azazel-Labs/polysmelt), behind a feature.

```rust
use dittomo::{Clip, Fixes, Mapping, Skeleton};

fn play_on(source: &Skeleton, target: &Skeleton, clip: &Clip) -> Result<Clip, dittomo::Error> {
    let map = Mapping::new(source, target);
    println!("{:.0}% alike", map.quality() * 100.0);
    map.apply_with(source, target, clip, Fixes::ALL)
}
```

`apply` is the plain retarget; `apply_with` adds the contact fixes below, and `apply_touching` also keeps hands on the
body, given both rigs' skins. For a game loop, `Player` poses one frame at a time without allocating, and `Crowd` poses
many characters through one player at once, eight side by side, in about half the time each.

For poses that arrive one at a time, with nothing ahead to look at (motion capture streaming in, a networked player,
another animation system's output), `Live` plays each and makes the same fixes from the poses so far, also without
allocating:

```rust
use dittomo::{Fixes, Live, Mapping, Skeleton, Transform};

fn stream(source: &Skeleton, target: &Skeleton, poses: impl Iterator<Item = (f32, Vec<Transform>)>) {
    let map = Mapping::new(source, target);
    let mut live = Live::new(&map, source, target, Fixes::ALL).unwrap();
    let mut out = target.rest();
    for (dt, pose) in poses {
        live.pose(dt, &pose, &mut out).unwrap();
        // draw `out`
    }
}
```

Arms kept out of the body, hands together and hands on the head come out just as the whole clip's do. Feet plant as
soon as the source's are down and nearly still, held where they were shown the pose before so they don't slide as they
settle, and a clip walked in place moves on as far as the planted foot slides back. `Live::touching`, given both rigs' skins, keeps hands at the skin's distance too, just as the whole clip does.
`Live::delayed(6)` plays each pose six poses late, a fifth of a second at 30 a second, and plants feet with those six
in sight as the whole clip does: eased in before they land, held where they stand on average.

The videos are rendered by `examples/video.rs`: the source, dittomo's copy, and the same clip with each bone's rotation
copied by name, side by side, skinned and drawn on the CPU and piped to `ffmpeg`.

```sh
cargo run --release --features fs --example video -- source.glb target.vrm out.mp4 [--clip NAME] [--seconds S]
```

## Results

### Finding the right bones on real rigs

Twelve real characters from four ecosystems, fetched at pinned commits by `sh bench/rigs/fetch.sh`: Mixamo's Xbot,
Michelle, Soldier and Samba Dancing (one as FBX), Ready Player Me, three.js's RobotExpressive, Khronos's CesiumMan,
RiggedFigure and BrainStem, two VRM avatars, and Babylon's HVGirl. Each rig's body joints are labelled by hand
(`bench/rigs/labels.txt`); every one of the 132 ordered pairs is mapped, and a joint counts as found when the target's
joint plays the source's. `cargo bench --features fs --bench rigs`:

| Matcher | Joints found (2558) |
| --- | ---: |
| dittomo | 96.4% |
| Exact name, ignoring case and namespace (three.js `SkeletonUtils.retarget`, Babylon) | 30.3% |

| Joint | dittomo | By name | | Joint | dittomo | By name |
| --- | ---: | ---: | --- | --- | ---: | ---: |
| hips | 100% | 38% | | thighs | 97% | 24% |
| spine | 95% | 38% | | shins | 97% | 24% |
| chest | 98% | 24% | | feet | 100% | 29% |
| neck | 100% | 51% | | toes | 100% | 29% |
| head | 94% | 32% | | clavicles | 100% | 44% |
| upper arms | 92% / 88% | 23% | | forearms | 92% / 88% | 23% |
| hands | 100% | 33% | | index fingers | 94% | 42% |

What it gets wrong:

- **RobotExpressive**, a cartoon whose legs start a fifth of its height out from the middle and whose feet hang from the
  root for IK: a third of its joints are found against CesiumMan and RiggedFigure, whose names say nothing either.
- **BrainStem**, named `node0`…`node21`, with one arm stopping short of the hand and upper arms starting far out from
  the middle: 67% to 100% against the Mixamo rigs. With no collarbone, its upper arms start where a collarbone ends
  and are taken for one.
- With names to go by, the misses are mostly one bone along a chain (a chest taken for the bone below it).

CI runs this on every push and fails below 95% of joints found or above 3° of limb error. The rigs are fetched rather than committed; each keeps its own license.
Engines' own humanoid matchers (Unity's Avatar, Godot's `BoneMap`, Unreal's IK Retargeter) aren't in the table: they need
the engine running.

### Playing real clips

The same rigs' own 35 clips (Mixamo dances, walks, runs and gestures, RobotExpressive's 14, CesiumMan's and
RiggedFigure's walks, BrainStem's 35-second take, HVGirl's four) played on every other rig: 385 clip and rig pairs.
Each labelled limb segment, shoulder to elbow and so on, is compared with the source's direction frame by frame, each
in its own body's frame. The other rows are what a clip gets without a retargeter, even given the right bones: an
engine's animation mixer plays a clip on another rig by copying each bone's local rotation, and a hand-written script
commonly copies each bone's turn from rest in the world.

| Retarget | Mean | Median | 90th percentile | Within 15° |
| --- | ---: | ---: | ---: | ---: |
| dittomo | 2.4° | 0.2° | 6.1° | 97% |
| dittomo, with the joints it misses set by hand (`Mapping::set`) | 1.3° | 0.2° | 4.0° | 99% |
| World turns from rest copied onto the right bones | 22.0° | 11.2° | 64.6° | 56% |
| Local rotations copied onto the right bones | 74.7° | 83.8° | 151.4° | 22% |

What's left is mostly proportions: a two-bone stretch (hips to chest, hand to index finger) can't point the same way on
both rigs when the bones between are in other ratios. The upper arms are the biggest gap between dittomo and the hand
labels, from BrainStem's and RobotExpressive's misses above. CI fails if the mean passes 4°.

Where the character looks is judged apart: the way the head faces, against the source's, each in its own body's frame.
It's off by 1.3° on average (local rotations copied: 54.6°). It was 11.8° when the neck and head took their source's
turns as they are, which tips the face of a rig whose head bone isn't built facing forward. They now keep the target's
own rest look and take only the turn away from it; that costs chest → head 1.6° of direction, 4.6° to 6.2°.

The same pairs again, with each contact fix alone and all together, over the 23,420 frames where a source foot is
planted (within 5% of its height of the lowest it gets, moving under a tenth of its height a second). Slide is how far
the target's foot moves each such frame; height off is how far it is from the source foot's height over the floor,
scaled; both are over the target's height. Clips walked in place are judged on the travel their feet imply.

| Fixes | Foot slide | Foot height off | Limb directions, mean |
| --- | ---: | ---: | ---: |
| none (`apply`) | 0.00389 | 0.0093 | 2.3° |
| travel | 0.00099 | 0.0093 | 2.3° |
| planted feet | 0.00313 | 0.0019 | 3.3° |
| push-out | 0.00389 | 0.0093 | 2.3° |
| all (`apply_with`) | 0.00003 | 0.0011 | 3.5° |
| all, live (`Live`) | 0.00004 | 0.0012 | 3.7° |
| all, live, 6 poses late (`Live::delayed`) | 0.00005 | 0.0011 | 3.5° |

Together they cut foot slide 119 times and the height error 8.4 times. The degree they cost is the legs bending to keep
feet down where the target's stride and leg proportions differ from the source's. Played live, with nothing ahead,
slide is cut 93 times: a foot is held from the pose it lands, where the target's was the pose before, so it doesn't
slide as the legs settle unlike the source's, though the knee bends a little more to hold it. Six poses late it is cut
74 times, with limbs as close as the whole clip's. A foot is planted from the first pose it stands on, judged by how
fast it comes or goes, whichever is slower, so clips keyed in steps, whose feet jump between held poses, keep their
feet still too. Push-out finds nothing to do here, as none of these clips put a hand through the body.

Hands on the body, over the 63 pairs where both rigs have a skinned mesh: 8,802 hand frames, 4,671 of them with the
source's hand within 3% of its height of its own skin. The gap is how far the hand is out from the skin nearest it,
over each body's height; the target's should match the source's.

| Fixes | Hand gap off, source hand near: mean | 90th percentile | Target hand inside its body, source's not |
| --- | ---: | ---: | ---: |
| all but touch (`apply_with`) | 0.0290 | 0.0651 | 8.0% |
| all (`apply_touching`) | 0.0106 | 0.0212 | 1.4% |
| all, live (`Live::touching`) | 0.0106 | 0.0212 | 1.4% |

So a hand resting on Michelle's hip rests on the coat of a stockier body instead of sinking into it: the gap is matched
2.7 times closer and hands end up inside the body a sixth as often.

### Against the learned retargeters

[SAN](https://arxiv.org/abs/2005.05732) ([code](https://github.com/DeepMotionEditing/deep-motion-editing), SIGGRAPH
2020) set the Mixamo test most learned retargeters report on: 106 clips played from each of four characters onto the
others with the same skeleton (intra-structural), and from BigVegas, whose skeleton adds a pelvis hub, hair, a cape and
a facial joint, onto all four (cross-structural). The error is the mean squared distance of each joint from the clip
Mixamo itself made for the target, over the target's height squared, ×10³. `sh bench/san/fetch.sh` downloads SAN's test
set; `cargo bench --bench san` scores it with SAN's own formula.

| Method | Trained | Intra | Cross |
| --- | :---: | ---: | ---: |
| NKN (CVPR 2018), from SAN's table | yes | 6.24 | 243 |
| PMnet (BMVC 2019), from SAN's table | yes | 5.72 | – |
| CycleGAN-style, from SAN's table | yes | 7.66 | 8.97 |
| SAN, from SAN's table | yes | 2.76 | 2.25 |
| **dittomo** (`apply`, no fixes) | no | **1.60** | **0.85** |
| Local rotations copied by bone name, root moved as the source's | no | 43.0 | 24.9 |
| Local rotations copied by bone name, root scaled by hips height | no | 1.37 | 0.70 |

dittomo finds the bones itself, needs no training, and beats SAN on both. Three caveats:

- Copying local rotations by name and scaling the root does better still, because Mixamo makes these clips by copying
  rotations onto each character and then adjusting its legs; per joint, the copy is exact above the hips.
  [R2ET](https://arxiv.org/abs/2303.08658) ([code](https://github.com/Kebii/R2ET), CVPR 2023) says the same of its own
  table, where copying beats every learned method on position error. This test rewards doing what Mixamo does more than
  looking right. The copy works only because all five skeletons share Mixamo's names, rest axes and T-pose; on the real
  rigs above it is off by 74.7°.
- SAN's paper gives 8.86 for its copy baseline; unscaled, these runs get 43.0, mostly from Mousey being about half
  BigVegas's height. The scaled copy is the fair baseline.
- The travel fix, which adds the travel an in-place clip's feet imply, does harm here (8.2 and 14.3): Mixamo's in-place
  dances, like "Dancing Running Man", are meant to stay put.

### Speed

On one core of an Intel Core Ultra 7 155H, Rust 1.100, `cargo bench --bench retarget`, best of several runs:

| Case | Time |
| --- | ---: |
| Map a 65-bone Mixamo-layout rig onto itself | 0.5 ms |
| Map it onto a 76-bone game-engine mannequin (five-bone spine, twist bones, A-pose, other bone axes) | 0.9 ms |
| Play a clip, mixamo → mannequin, setup included | 2.5 µs per frame |
| Pose one frame with a reused `Player` | 1.3 µs per frame |
| A walk with every fix on: travel, planted feet, push-out | 3.3 µs per frame |
| The same walk done in place, travel worked out from the feet | 3.3 µs per frame |
| Set up a `Player` for a pair of rigs | 40 µs |

Mapping's times are scaled from runs back to back with the earlier, slower mapping, which took 1.1 and 2.3 ms here.
On a busier run, where the walk with every fix took 4.5 µs a frame, `Live`
played the same walk with every fix but touch in 3.1 µs a frame, the walk done in place in 3.0 µs (the whole clip:
4.9 µs), and the walk six poses late in 3.4 µs. Where a reused `Player` posed a frame in 1.8 µs, a `Crowd` of 64 characters
posed each in 0.89 µs. Keeping hands at the skin's distance as well, with skins of 5,570 and
6,697 points, takes about 4.5 times as long as the live walk without it (30 µs against 6.5 µs on a run twice as busy).

three.js plays the same clip on the same rigs in `bench/three`, given dittomo's bone pairs as its names so only playing
is timed. Measured back to back with dittomo on one machine (three.js r187 on Node 26):

| Play a 60-frame walk, mixamo 65 → mannequin 76 | dittomo | three.js `SkeletonUtils` | |
| --- | ---: | ---: | ---: |
| Whole clip, setup included (`apply`, `retargetClip`) | 3.6 µs per frame | 221 µs per frame | 61× |
| One frame, rigs reused (`Player::pose`, `retarget`) | 2.5 µs per frame | 186 µs per frame | 74× |

three.js does less for that time: it copies each bone's world rotation as it is, with no correction for rests that
differ unless offsets are given by hand, and finds each bone's pair by name on every frame.

Babylon.js's `AnimatorAvatar.retargetAnimationGroup` (9.34, in `bench/babylon`) is closer. It corrects for rests that
differ, carrying each key's turn from the source bone's rest into the target's, and bakes a whole clip, with no call
to pose one frame. Measured back to back on a busy machine, best of five runs each: dittomo played a frame in 7.0 µs,
Babylon in 50 µs (7×), and in 143 µs with the hips' path scaled to the target (20×), which it does by playing both
clips through again key by key. three.js took 327 µs on the same runs. CI runs all three on every push.

Foot skate per planted frame, over the character's height, on a 90-frame walk:

| Target | Plain | Feet planted |
| --- | ---: | ---: |
| Same rig with 30% longer shins and feet | 0.00048 | 0.00002 |
| Mannequin | 0.00053 | 0.00001 |

Timings and foot skate are measured on made-up rigs that copy the real ones' layouts (see Tested on). CI benchmarks
every pull request against its base on the same runner and fails it if anything gets more than 25% slower.

## Reading models

With the `load` feature, `dittomo::load::Model` reads a skeleton and its clips from glTF, GLB (VRM avatars too), FBX or
BVH bytes (`Model::from_bytes(bytes, "fbx")`), FBX read directly with no Blender round trip and BVH motion capture (the
CMU database's, say) with no conversion. `fs` adds
`Model::load(path)`, and `usd` adds USD. Each skin's joints, and the nodes above them, become the skeleton; each
animation is sampled at its keys into a `Clip`. `load` builds for wasm. The twelve benchmark rigs and their 35 clips
read in about 0.14 s together on one core of a laptop. The eleven glTF and VRM rigs take under 50 ms between them (the
biggest, hvgirl's 1,295 frames over 147 channels, about 12 ms); the rest is the 3.7 MB Mixamo FBX with its 1,223-frame
dance, most of it inflating its compressed arrays.

## Contact fixes

`Fixes` turns each on or off; all are on by default. None of them need bone names: legs and arms are found from the rest
pose alone (`contact::legs`, `contact::arms`). The spine runs from the biggest root to the highest bone. A leg ends in a
bone near the floor on a chain hanging from the spine; an arm ends at the first bone whose children split the chain
evenly, as fingers do. The knee or elbow is the joint along the chain standing farthest from both the spine and the
end, and the hip or shoulder the one above it farthest from both, so twist bones, sleeves and helpers along the way
don't count. The arms are the chains off the one spine bone whose chains end in the most bones, so hair, capes and
tails are left out, and fingers hanging low aren't taken for feet.

- **Travel.** A clip walked in place gets the travel its planted feet imply. A clip that already travels is left alone.
- **Planted feet.** Where the source's foot is planted, the target's is held where it lands, at the height the source's
  foot is over the floor, scaled to the target. Two-bone IK (`contact::reach`) bends the knee and swings the hip to put
  it there, keeping the foot's turn, easing in and out over about a tenth of a second.
- **Push-out.** Spheres along the spine and one round the head stand for the body; elbows, hands and forearms inside
  them are turned out, the upper arm first, then the forearm. Short arms raised overhead go round the head, not
  through it.
- **Hands together.** Where the source's wrists meet (a clap overhead in jumping jacks, a clasp, hands pressed in
  prayer), the target's are brought together at a point both its arms reach, outside the body and the head, keeping
  the hands' turn and the gap between them the source has, scaled. Arms too short to meet where the source's did meet
  lower, arms too long meet without crossing; the meeting point is searched for round each sphere in the way, and kept
  near the last frame's so the hands don't jump. They let go smoothly as the source's hands part.
- **Hands on head.** Where the source's hand is on its head (covering the ears, clasped on top, a palm to the forehead),
  the target's goes to the same place on its own head: the way from the middle of the head to the wrist is taken in
  the head's frame and scaled from one head to the other, so a cartoon's big head gets hands spread wider and the hands
  follow as the head turns. The head's size comes from its skin with `apply_touching`, else from the bone at its top,
  where there is one. A hand that can't get there (short arms on a big head) goes as near as it can over the head's
  surface, not into it, and the elbow swings round the line from shoulder to hand till the arm is no deeper in the head
  than the source's.
- **Touch** (`Mapping::apply_touching`, given each rig's `Surface`; `load::Model` reads it as `skin` from the skinned
  mesh). Each skin is thinned to points about 1/80 of the height apart, each bound to the bone that moves it most.
  Every frame, the gap from each hand to the skin of the rest of the body (arms left out) is measured on the source
  and on the target, along the skin's smoothed outward facing. Where the source's hand is near its body, two-bone IK
  moves the target's hand to the same gap over its own height, keeping its turn, and lets go smoothly as the source's
  hand moves away.

## How it maps

Nothing is assumed about what the skeletons are: humans, quadrupeds, birds and tails all go through the same matcher.
Every source bone is compared with every target bone on:

- **Names.** Spellings from Mixamo, Unreal, Unity, Rigify, Character Creator, Daz, 3ds Max Biped and Blender's numbered
  copies are brought together: `mixamorig:LeftUpLeg`, `thigh_l`, `DEF-thigh.L` and `LeftUpperLeg` all read as the left
  thigh. Sides (`L`, `_r`, `Left`) and numbers (`Spine2`, `spine_03`) count. Bones named `Bone.004` simply say nothing.
- **Shape at rest**, measured in the body's own frame and over its height, so size, units and facing don't matter:
  - which way the bone points from its parent and to its biggest child
  - how high it stands
  - how long it is and how much body lies beyond it
  - which side it's on
  - and, counting for less as proportions differ, where it is and how far along the body.

The body's frame is found from the plane its bones are most nearly mirrored in, turned towards the bones named left or,
failing names, the way its feet point.

The two trees are then matched as wholes for the most total likeness, keeping the family. A bone below another plays a
bone below the other's, and separate limbs play separate limbs. Any bone on either side can be passed over, so a root
above the hips, twist helpers, a five-bone spine against a three-bone one, fingers or a tail don't throw off the rest.
This is the constrained tree mapping of Zhang (1996), with subtrees paired by the Hungarian method.

`Mapping::quality` says how much of the target is played and how alike, from 0 to 1. `Mapping::set` corrects a bone
the match got wrong.

## How it plays

Each mapped bone takes its source bone's turn away from rest, in the world. That turn is first carried into the
target's frame and corrected for the two rest poses: at rest, the source bone and the next mapped bones below it point
one way, and the target's point another, and the turn that best lines up all of them at once is undone. All of them,
not the one with most below it: one rig's collarbones start above its chest and another's beside it, and lining up
either alone would tip the chest. So the target's bones point where the source's
do, frame by frame, whatever each rig's rest pose (A-pose or T-pose) or bone axes.

A target bone between two mapped ones takes its share of their turns by how far along it is, so a short spine bends a
long one smoothly. Other unmapped bones keep their rest pose under their parent. The topmost mapped bones, and any whose
source moves rather than only turns, travel as their source does, scaled by how high each stands.

## Tested on

The twelve real rigs above. Their layouts are copied into the tests as made-up rigs where a fix came from one: a palm
bone holding two fingers, a COLLADA export stopping at the base of the head with numbered limbs in an A-pose, a
nameless rig with one hand missing, and a chest whose collarbones start beside it rather than above it.

Made-up humanoids that differ the ways real rigs do: names from different tools or none, extra roots and twist bones,
two to five spine bones, T-pose against A-pose, three conventions for bone axes, facing different ways, centimetres
against metres, and different leg lengths. A sweep of 300 such pairs all map correctly, with every limb within a
degree of the source's direction. A four-legged animal maps onto an unnamed one with a longer body and tail. A snake
mapped from a human is found to share little.

The contact tests walk a procedural gait (planted feet, swing, bobbing hips) on these rigs and on a longer-legged copy,
and push a hand placed inside the chest back out. Their limb finding is tested on an avatar laid out as VRM exports
are (twist bones between forearm and hand, helper bones off the collarbones, a ponytail, the chest bone well below the
shoulders) and on a rig whose fingers hang below its knees.

## Browser demo

[demo/](demo) runs dittomo in the browser as plain wasm, with no bindings generator and no JavaScript dependencies. Drop
in two glTF, GLB or FBX files, or use the built-in sample. It shows both skeletons playing side by side, the quality, how
long the match and the clip took, every target bone's match and score, and toggles for each fix. The whole module is
about 1 MB, file readers included.

```sh
sh demo/build.sh
cd demo/www && python -m http.server   # then open http://localhost:8000
```

## Next

The full plan, including the learned runtime this feeds and the open-source landscape, is in
[docs/plan.md](docs/plan.md).

- **The rest of the comparison benchmark**: R2ET's own test (its pairs aren't published as data). Bone-finding, limb
  directions, foot contact, playback against three.js and Babylon, and SAN's Mixamo test are done (above).
- **Engine humanoid profiles** in the rig benchmark, by exporting what Unity's Avatar and Godot's `BoneMap` choose.
- **Many characters per call**, laid out as structure of arrays.
