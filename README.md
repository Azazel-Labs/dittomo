# mummer

**The same performance, any body.** mummer retargets an animation from one skeleton onto any other. The more alike the
two rigs, the closer the copy, and a rig onto itself plays its clip unchanged.

Pure Rust, no `unsafe`, deterministic, MIT-0. Its only dependency is [glam](https://crates.io/crates/glam).

```rust
use mummer::{Clip, Mapping, Skeleton};

fn play_on(source: &Skeleton, target: &Skeleton, clip: &Clip) -> Result<Clip, mummer::Error> {
    let map = Mapping::new(source, target);
    println!("{:.0}% alike", map.quality() * 100.0);
    map.apply(source, target, clip)
}
```

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
one way, and the target's point another, and that difference is undone. So the target's bones point where the source's
do, frame by frame, whatever each rig's rest pose (A-pose or T-pose) or bone axes.

A target bone between two mapped ones takes its share of their turns by how far along it is, so a short spine bends a
long one smoothly. Other unmapped bones keep their rest pose under their parent. The topmost mapped bones, and any whose
source moves rather than only turns, travel as their source does, scaled by how high each stands.

## Tested on

Made-up humanoids that differ the ways real rigs do: names from different tools or none, extra roots and twist bones,
two to five spine bones, T-pose against A-pose, three conventions for bone axes, facing different ways, centimetres
against metres, and different leg lengths. A sweep of 300 such pairs all map correctly, with every limb within a
degree of the source's direction. A four-legged animal maps onto an unnamed one with a longer body and tail. A snake
mapped from a human is found to share little.

## Next

- **Root motion.** For a clip that runs in place, work out the travel from its planted feet.
- **Feet held planted** with leg IK, so feet don't slide when leg lengths differ.
- **A benchmark** on real rigs (Mixamo's same clip on different characters as the reference), and a browser demo.
