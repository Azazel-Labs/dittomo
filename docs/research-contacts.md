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

### The head's size without a skin (tried, 2026-11-26)

Heads guessed from bones are about 30% smaller than skins show (0.050 of the height against 0.062 to 0.088). Guessing
them that much bigger did harm: hands in the head on the atlas 37.2% from 33.8%, pops 0.36% from 0.30%. A hand is put
by the ratio of the two heads, which a bigger guess for both leaves as it was, while a bigger head is "near" from
farther off and takes hold of more hands. Not kept. What a hand in the head without a skin needs is the hand's own
size (its fingers reach into the head from a wrist put right), not a bigger head.

## The head as a part, and hands meeting (2026-11-26, last)

### The head (kept)

With placing on, the head is one more part: a ball where the body takes its head to be, as far out each way (6 up
by 8 round) as the skin on the head's bone shows, its sides tied between the rigs as the trunks' are. It is blended
with the other parts by nearness, as any part is, with three rules of its own:

- **A hand held on a head is there on purpose**, and is brought to its place from however far, not only from nearly
  there. Held: the hand's middle within 1% of the height of the head's surface (not at all past 4%), as much as the
  head counts against the part counting most.
- **A hand held on a head is kept off nothing else.** On a smaller head a hand is nearer the shoulders than the
  source's, and the trunk's top would push it off again.
- **A hand sweeping past a head isn't by it**: under 0.8 heights a second against the head it may be, over 1.6 not at
  all (what the old fix asks, in heights).

Where both bodies have a head, the fix made for the head isn't run: charts stand in for it.

The atlas (now 182 pairs), head and neck; real clips; the picked clips' wrist by the head (comb-hair, jumping-jacks,
thinker, wash-self):

| Fixes | head and neck | real clips: limbs, pops | wrist by the head |
| --- | ---: | ---: | ---: |
| none | 92%, 1.16, 0.0435, 25.2% | 1.9°, 0.20% | 0.0525, 0.0456, 0.0502, 0.0491 |
| all, bones only | 93%, 1.13, 0.0426, 33.8% | 2.9°, 0.30% | 0.0520, 0.0447, 0.0473, 0.0465 |
| all, with skins | 94%, 1.18, 0.0177, 10.6% | | |
| all and place, bones only, before | 93%, 1.10, 0.0416, 33.9% | 3.2°, 0.31% (live 3.7°, 0.33%) | 0.0482, 0.0445, 0.0414, 0.0422 |
| all and place, with skins, before | 93%, 1.20, 0.0184, 11.0% | | |
| all and place, bones only, now | 92%, 1.07, 0.0402, 27.7% | 3.2°, 0.30% (live 3.7°, 0.32%) | 0.0486, 0.0441, 0.0397, 0.0420 |
| all and place, with skins, now | 92%, 1.14, 0.0164, 6.4% | | |

Trunk and thighs are as they were (with skins: in the body 2.8% from 3.0% and 2.4% from 2.3%); every row without
placing is the same to the last digit. On rigs made to differ (`tests/contact.rs`: a head 1.6 and 0.7 times as tall,
arms half as long again), a hand's middle laid by the ear and on top lands within 0.26 rad round the head and 1.5%
of the height off its height over it, from 0.35 to 1.19 rad and 3.6% to 9.1% unfixed; a hand swept past moves
0.0003 of the height.

What was tried on the way, each on the atlas's head and neck (bones only, then with skins):

- **The old fix aimed by the chart** (the hand's middle, the fitted ball; the old fix still saying how much): in the
  head 32.8% and 10.9%, gap 0.0411 and 0.0177. Level: where a hand is aimed wasn't what put it in the head.
- **The old fix off, nothing in its place**: 22.9% and 6.5%. The old fix is what puts hands in heads here: it takes
  hold of every hand within one and a half head radii of the head's middle, those nearer the neck or a shoulder
  too, and sends them by the head alone. Blending by nearness is what the head needed.
- **The head as a part with no rule of its own**: 23.2% and 6.1%, pops 0.29%, the best the atlas gets. But the
  atlas's heads are much alike; on a head 1.6 times as tall the hand stays 1.8 rad round the head from its place
  (0.26 with the rules), as nothing is brought from past 10% of the height. Not kept as it was: the three rules
  above cost 4.5 points without skins and 0.3 with.
- **Held from farther off** (2% to 6% of the height): 30.1% and 6.9%. **From nearer** (0.5% to 2.5%): 26.1% and
  6.3%, the gap 0.0406. 1% to 4% kept.
- **The trunk stopping flat at the head's joint** (its top no ball about it): 29.0% and 6.8% from 28.6% and 6.5%
  then. Not kept.
- **Each part keeping a hand off by its share of the say** (and by four times its share): hands on thighs in the
  body 24.4% (24.1%) from 22.7%, with skins 2.8% (2.4%) from 2.4%. Not kept, though it is what lets a hand laid on the
  other hand stay there on arms three quarters as long (the trunk pushes both off it: 0.018 of the height off,
  0.003 with it, 0.009 with placing off).
- **Gaps scaled by the hands' sizes**, not the heights: trunk 0.0297 from 0.0277 and thighs 24.5% from 22.7%. Hand
  sizes read from skins and from bones don't agree well enough. Not kept.

Gone with the old fix when placing is on: the elbow swung round to keep the forearm out of the head, and a hand
that can't reach sliding over the head, not stopping short. Neither is measured by the atlas.

### Hands meeting (nothing kept but the measure)

The picked clips can't measure it: in none do the wrists come within 10% of the height (least apart: wash-self
14.8%, thinker 17.5%, jumping-jacks 18.5%), so that column is over no frames. A second atlas does
(`-- --atlas` prints it after the first, `-- --meeting` alone): one hand held before the chest, the other put by IK
all over the skin of that forearm and hand; 210 pairs, 15036 poses. Each cell: along the held forearm off, the
hand's gap to skin not its own arm's off (mean), in that skin, the wrists' distance off:

| Fixes | hand on the other arm |
| --- | ---: |
| none | 0.25, 0.0231, 25.1%, 0.0306 |
| all, bones only | 0.16, 0.0220, 27.6%, 0.0163 |
| all, with skins | 0.17, 0.0194, 22.8%, 0.0219 |
| all and place, bones only | 0.15, 0.0213, 26.5%, 0.0185 |
| all and place, with skins | 0.18, 0.0181, 21.6%, 0.0257 |

A hand ends in the other arm in over a fifth of the poses whatever is on; nothing tried moved that without harm
elsewhere:

- **A pair term** (the way from one hand's middle to the other's kept as the source's in the chests' frames, each
  hand going half the way, what one arm can't reach made up by the other, eased): scaled by the hands' sizes, in
  the arm 29.9% and 23.6%, wrists 0.0226 and 0.0359; scaled by height, 28.6% and 27.1%, wrists 0.0141 and 0.0219;
  only for hands within one and a half hand lengths, 26.7% and 22.8%. It keeps the wrists' distance and loses the
  touch: an offset copied says nothing of how thick the other arm is, which the forearm's chart already knows. On
  rigs made to differ it is exact (0.000 of the height off), which is what the test measured, not what the eye
  does. Not kept.
- **The other hand as a part** (a line from the wrist to twice the hand's middle, fitted to the skin of the hand and
  fingers): 27.3% and 21.3%. The forearm's end already stands for it. Not kept.
- **A hand by the other forearm there on purpose**, as on a head: along the forearm 0.13 and 0.16, from 0.15 and
  0.18, but in the arm 27.1% and 22.4%. Not kept.
- **The touch fix seeing the other arms' skin**: in the arm 8.2% from 21.6%, gap 0.0110 from 0.0181. But in the
  first atlas the other arm hangs by the hip, and a hand put on that hip is "touching" it: trunk in the body 3.7%
  from 2.8%, thighs on the same bone 84% from 88%; with the body's skin having the last word, 11.0% in the arm,
  trunk still 4.7%. Not kept, but the one thing that moved the number: fingers reach into the other arm from a
  middle put right, and only the skin knows.

Next for hands meeting: the touch fix against the other arm, told apart from an arm hanging by the body (the two
contacts solved together, or an arm by the body counted as the body); palm facing. Crossing order wasn't tried.
