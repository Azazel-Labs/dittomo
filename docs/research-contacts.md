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
