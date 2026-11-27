# A network per clip, and what might do better (future research, noted 2026-11-26)

Not started, on purpose. Ideas to prototype later, with what a survey found and what each would have to beat.

## The idea

For one animation, a small network that takes a time and a body (skeleton and skin) and gives that body's pose, as a
human animator would have redone it: `pose = f_clip(time, body)`.

dittomo is already a function of that shape, in about 4 µs a frame for any skeleton, so a network can't win on speed
for one character on a CPU. Where it could:

- **What rules miss**: an animator's choices nobody wrote down (which hand leads, when a contact is given up).
- **Many bodies on the GPU**: a crowd of different proportions, or a character creator's sliders, answered in a shader
  with no bake per body.
- **Size**: one function in place of a clip baked for each of a thousand bodies.

## The hard part: what teaches it

A clip's own frames show the motion on one body. Nothing says what it should be on another.

| Teacher | The network becomes | Catch |
| --- | --- | --- |
| dittomo on thousands of made-up bodies | a small, GPU-ready copy of the rules over bodies | no better than the rules |
| A slow whole-clip optimizer | a fast copy of something too slow to run live | what it optimizes is the rules again, as costs |
| An animator's fixes on three or four very different bodies | animator intent spread over bodies | animator time per clip; four examples over a whole space |

The third is the one that matches the idea. dittomo gives the base for any body; the network learns only how the
animator's answer differs from it, and falls back to the rules where it is unsure.

## How to shape it

- A small network per clip (thousands of weights, runs in a shader), not a transformer. A transformer with a token a
  bone belongs one level up: one model over all clips.
- The body described after mapping: the same slots for any skeleton (hips, chest, upper arm), their lengths, and each
  part's radii from the body charts. No topology for the network to learn.
- It gives contacts, not angles: where a hand is against a part (along, round, gap), solved by the IK there is. Bone
  lengths then always hold.
- Rules last: a learned part sits between dittomo's plain result and its contact fixes, so contacts stay exact.

## The smallest test

One clip with clear contacts (cmu-comb-hair or cmu-wash-self). A family of bodies by sliders: limb lengths, shoulder
width, head size, girth. On bodies held out, against a slow whole-clip optimizer taken as truth:

1. dittomo as it is;
2. a small network taught by the optimizer;
3. dittomo and a small learned correction;
4. a blend of a few baked clips at the same size, with nothing learned.

Worth going on if (3) clearly beats (1) on the contact atlas's measures, and (2) or (3) beats (4) by half or more. If
(4) ties, ship the blend: it needs no training.

## What is published (survey, 2026-11-26)

Read from search results and abstracts, not the papers; check one before leaning on it. Costs are estimates.

No paper found does one network per clip with the body as input. Nearest:

- **Many clips, body as input**: Skeleton-Aware Networks (Aberman et al., 2020, arxiv 2005.05732), SAME (Lee et al.,
  2023), HuMoT (Mourot et al., 2023, arxiv 2305.18897). Skeletons only.
- **Skin and contacts**: Contact-Aware Retargeting of Skinned Motion (Villegas et al., 2021, arxiv 2109.07431), which
  says a network alone can't hold contacts and optimizes each sequence after it; R2ET (Zhang et al., 2023, arxiv
  2303.08658), a learned correction on copied rotations; MeshRet (Ye et al., 2024, arxiv 2410.20986).
- **One clip as a function of time**: NeMF (He et al., 2022, arxiv 2206.03287). One skeleton.
- **Rules, no learning**: ReConForM (Cheynel et al., 2025, arxiv 2502.21207), interaction meshes (Ho, Komura, Tai,
  2010). dittomo's own family.

What it says about the idea:

- There is no data of animators redoing clips on other bodies. The pairs the field trains on are Mixamo's, made by a
  retargeter.
- Networks hold contacts as soft costs; rules hold them exactly.
- A network of 50 to 200 thousand weights: about 10 to 50 µs a frame on a core. A token a bone: 1 to 3 ms.
- For one body a network's answer is a baked clip, and dittomo bakes 300 frames in about 1.5 ms.

## Ideas that may do better than the idea as put

- **Functional maps** (Ovsjanikov et al., 2012) carry points from one skin to another of a different shape. In place
  of along, round and gap by part: a true skin-to-skin match for where a hand goes, with nothing trained per clip.
- **One network over all clips** (as DeepONet, Lu et al., 2021, arxiv 1910.03193), if size over bodies is the aim.
- **Blending baked clips over sliders**, nothing learned: the baseline above, and maybe the answer for character
  creators.
