# Keeping a hand's place by the body: research digest (2026-11-26)

Three surveys (academic contact-preserving retargeting, robotics and production tools, cross-domain math) were run to
find what would make dittomo pose a target the way an animator would: hands over the head, touching, on a hip, on a
knee, crossed, flat on the floor. This is the digest; costs given for things dittomo doesn't have yet are estimates.

## What everyone converges on

The general form of dittomo's separate hand fixes (hands on the head, touch, hands together) is one idea: **where an
end is, expressed against the body parts near it, weighted by nearness.** Published as egocentric coordinates (Molla,
Debarba, Boulic, TVCG 2018), relationship descriptors (Al-Asqhar, Komura, Choi, SCA 2013), a one-vertex interaction
mesh (Ho, Komura, Tai 2010; OmniRetarget 2025), MeshRet's (bone, fraction along, angle about) sensors (NeurIPS 2024),
and shipped as Unreal 5.7's experimental Relative IK op, which needs a physics asset per rig. Molla's runs on analytic
IK; nothing published runs near microseconds.

## The plan it suggests ("body charts")

1. **Parts.** About 17 per body, one per mapped bone pair (pelvis, spine segments, chest, neck, head, each limb's
   upper, lower and end), each a segment with a radius: from the skeleton alone, or a small table of radii by
   (fraction along, angle about) sampled once from the skin. The floor is one more part.
2. **Coordinates.** A source point against part k: fraction along the bone `t`, direction about it `n` in the part's
   frame, and gap `g` off its surface. On the target: the same `t` and `n` on its part, gap scaled by size.
3. **Blend.** Each part's answer weighted by nearness (ReConForM: full at 5% of height, none at 15%; or 1/(g+ε)^γ),
   with the plain retarget as one more candidate, so nothing near means today's result exactly, and a rig onto itself
   is the identity.
4. **Pair term.** Two hands mapped apart will part; hands near each other map their midpoint and keep their scaled
   separation and relative turn (cooperative dual-arm task space; DexPilot switches to the exact gap when close).
5. **The palm, not the wrist.** The point of the hand nearest the body is what is mapped (sign-language phonology
   treats the contacting part of the hand as part of the sign); the wrist follows.
6. **Feasibility after.** Push-out, reach clamps and the meeting search stay as they are, run after the goal is set;
   an unreachable goal slides over the part toward the shoulder (what hands-on-head does now, for any part).

Then, in order of worth:

- **Planted hands, knees, elbows** on the floor (push-ups, crawling, cartwheels), with the body pitched so the arms
  keep the source's bend: the arm's version of lowering the body for planted feet. HumanIK and iClone have hand floor
  contact; nothing live does.
- **Latching** a contact in the touched bone's frame while the source's holds still there, so a hand resting on a hip
  doesn't skate as the nearest part changes (the known weak point of egocentric methods: Delahaye 2023 reports
  jitter).
- **Crossing order**: for forearm pairs and shins, keep which is in front (sign of the triple product of the two
  segments and the line between them) and their gap, by swinging elbows about the shoulder-to-wrist line.
- **Elbow as a second effector**: pick the swivel putting the elbow nearest where its own chart sends it.
- **Palm facing**: keep the angle between palm and surface.
- **Fingertip contact** (pinch, prayer, interlaced), **held props** (a constant hand-to-hand transform means a
  two-handed prop: keep it rigid), soft knee and elbow extension, hinge limits, feet spacing by hip width.

Rejected: cages and generalized barycentric coordinates (global, costly, negative weights), whole-mesh interaction
solves, per-frame transport or registration, distances alone (they lose direction: MeshRet's ablation, 2.57 against
0.77 contact error).

## Failure modes to design against

Jitter where the nearest part switches; stalls at the reach limit; over-stiff push-out at armpits and crotch (never
push shallower than the source's own depth); false contacts (gate by how far the plain result is from the goal:
Villegas uses 15 cm); creases where two parts disagree (bound the weight, then project onto the nearest part's gap).

## Measures worth adopting

Contact precision and recall per pair of parts; where on the body the contact lands, not only its gap; penetration
share; grounded and sliding F1 (ReConForM: within 1% of height, under 0.1% of height a second); jerk against the
source's; agreement of plain relations (hand above head, hands close, hand left of the midline). Position error
against Mixamo's own retarget is not a contact measure: copying rotations wins it in every paper.

## Speeds published by others

GMR 14 to 29 ms a frame; teleoperation stacks about 10 ms; ReConForM 67 FPS on a GPU; a 2026 quadratic-program
self-penetration repair 0.75 ms on one core; Unreal, Unity, HumanIK publish nothing.

## Features found nowhere else

Mapping any topology with no name table; contact fixes with no per-rig setup; live planting with no look-ahead;
hand-on-head scaled by head size; the floor fix for quadrupeds; microseconds a frame.

## First prototype (`Fixes::place`, off by default, whole clips only)

`src/contact/charts.rs`: a trunk (from between the hips' joints to the head's, the shoulders counted as three fifths
along), a head ball, thighs, shins, upper arms and forearms, each with its surface along and round (6 by 8), from the
skin where known, else guessed from hip and shoulder spacing. The hand's middle is mapped, nearer parts counting for
more; the two bodies' axes are tied by the mapping's turn, as a rig's own forward can face the other way.

The contact atlas (`cargo bench --features fs --bench rigs -- --atlas`; `ATLAS_PAIRS=1` prints each pair,
`ATLAS_ONLY=a.glb,b.glb` runs one, `ATLAS_SELF=1` plays each rig onto itself, which must come back unchanged) on 210
pairs, each cell the same labeled bone, along-bone error, hand gap off (mean), in the body:

| Fixes | trunk | head and neck | thighs |
| --- | ---: | ---: | ---: |
| all, bones only | 59%, 0.46, 0.0307, 18.7% | 93%, 1.13, 0.0426, 33.8% | 88%, 0.18, 0.0294, 29.0% |
| all, with skins | 59%, 0.47, 0.0091, 3.7% | 94%, 1.18, 0.0177, 10.6% | 88%, 0.18, 0.0083, 3.8% |
| all and place, bones only | 68%, 0.32, 0.0263, 25.3% | 93%, 1.03, 0.0409, 34.8% | 91%, 0.09, 0.0226, 29.0% |
| all and place, with skins | 68%, 0.33, 0.0081, 6.1% | 93%, 1.19, 0.0185, 13.5% | 89%, 0.11, 0.0060, 4.5% |

Better where a hand is along the trunk and thighs, and its gap there; not yet on the head, and more hands end in the
body. Lessons: skin binding near the hips differs too much between rigs to say which part a hand is by, so labeled
bones do; guessed thicknesses were half the real ones until set against skins. Next: keep a hand no deeper than the
source's after placing; the head; hands meeting; `Live`; hands on the floor.

### After two fixes (2026-11-26, later)

- A placed hand comes no nearer a part than the source's is to its own.
- **Only what is nearly there is brought there** (all the way from within 3% of the height, not at all from past
  10%). Without this a broad-shouldered target's hands, hanging far from its thighs, were pulled a quarter of its
  height inward because a slight source's hang by its own: on real clips arm directions went from 2.9° to 4.2° off
  and pops from 0.30% to 0.75%. A hand hanging by a thigh is by it by chance, not by intent; the gate is how the two
  are told apart for now.
- The head is left to the fix made for it, which eases a hand on and off.
- An arm's last tenth of reach is given up softly.

| Fixes | trunk | head and neck | thighs | real clips: limbs, pops |
| --- | ---: | ---: | ---: | ---: |
| all, bones only | 59%, 0.46, 0.0307, 18.7% | 93%, 1.13, 0.0426, 33.8% | 88%, 0.18, 0.0294, 29.0% | 2.9°, 0.30% |
| all, with skins | 59%, 0.47, 0.0091, 3.7% | 94%, 1.18, 0.0177, 10.6% | 88%, 0.18, 0.0083, 3.8% | |
| all and place, bones only | 65%, 0.39, 0.0277, 18.4% | 93%, 1.09, 0.0415, 33.8% | 90%, 0.15, 0.0240, 22.6% | 3.2°, 0.33% |
| all and place, with skins | 63%, 0.40, 0.0077, 3.2% | 94%, 1.17, 0.0178, 10.6% | 88%, 0.16, 0.0068, 2.5% | |

Nothing on the atlas is worse with it; a wider gate (6% to 14%) places better still (68%, 0.0072) but costs 3.5° and
0.42% pops. Open: telling intent from chance better than by distance (stillness against the part; an arm hanging
straight down), so the gate can widen; `Live`; hands meeting; the floor.

### Telling intent from chance (tried, 2026-11-26)

- **Arm hanging straight down** (shoulder over wrist): widening the gate for arms that aren't cost as many pops as
  widening it for all (0.42% at 6% to 14%), so hanging arms weren't where the pops came from.
- **Stillness against the part** (hand's speed in the part's frame under 0.3 to 1 heights a second): limbs 3.1° from
  3.2°, but pops up to 0.41% from 0.33%. Speed flickers from pose to pose, and so did the weight.

So how much a hand is placed has to be eased over time, as the head fix eases a hand on over a quarter of a second
(ahead of the source's over a whole clip), not switched by what one pose shows. That is the next thing to build:
charts give where; an eased weight gives how much.

### Eased (2026-11-26, later still)

Charts say where; how much is eased. Over a whole clip a hand is placed no less than a pose a quarter of a second
before or after has it, less that time; live, it rises and falls no faster than that. Pops with placing on: 0.31%
whole clip (0.30% without), 0.33% live (0.33%); limbs 3.2° (2.9°) and 3.7° (3.4°). The atlas is as before. A wider
gate now costs few pops (0.34% at 6% to 14%) but more of the source's arm directions (3.6°), mostly arms hanging by
thighs: that, not pops, is what keeps the gate narrow now.

A hanging arm brought from less far (1% to 4%, others from 6% to 14%) holds limbs to 3.2° and pops to 0.34%, and
with the narrow gate for the rest gets 3.0°; but the atlas's hands on thighs are on arms hanging straight down, and
lose what placing gave them (in the body 29.2% again, from 22.7%), and the wider gate round the head does harm with
skins (in the body 13.5% from 10.6%). Not kept: one narrow gate for all stays.

## The floor, and hands on it (2026-11-26)

- Seven CMU clips picked for what the hands do are played on every rig (`-- --moves`). They showed first that a clip
  whose floor isn't its rest pose's left the target hanging over a meter above its own: fixed (see the README).
- A hand down on the floor and still is now held there as a foot is, over a whole clip (an arm takes part only if
  its hand comes down somewhere in the clip). Crawling: a low, still wrist is 0.051 of the height off the source's
  height over the floor, from 0.060, and slides 0.0022 a frame, from 0.0026. Feet get to 0.0000 and 0.00001: a
  crawl has knees down too, and a body with other arms and legs has to pitch its trunk to bring both down, which
  reaching with the arms alone can't. That (knees and elbows as contacts, the trunk pitched between hands and
  knees) is the next step for the floor; `Live` doesn't hold hands yet.
