//! Clips written as BVH (Biovision Hierarchy) motion capture, for tools that read no glTF.
//!
//! [`write()`] turns a skeleton and a clip on it (a retargeted one, say) into BVH text. The hierarchy is the skeleton's
//! rest pose with every joint's axes lined up with the world's, as BVH tools expect: each `OFFSET` is where the joint
//! sits from its parent at rest, in the world, so the file's rest pose is the skeleton's own (its T- or A-pose)
//! whatever axes its bones had. Each frame turns each joint from there, as `Zrotation Xrotation Yrotation` in degrees
//! (Rz·Rx·Ry), kept near the last frame's angles so nothing flips by a full turn between frames. A root also has
//! `Xposition Yposition Zposition`, its place in the world, as does any other joint whose distance from its parent
//! changes; positions replace the `OFFSET`, as most readers take them. Scale is folded into the offsets and positions,
//! since BVH has none. A leaf gets an `End Site` that carries its bone on as far again.
//!
//! BVH frames are evenly spaced: a clip whose times are is written pose for pose, and any other is sampled at the
//! frame time asked for (rotations slerped, translations and scales lerped). Units are the skeleton's own, BVH being
//! Y up like glTF and silent about units. A name's whitespace becomes `_`, since most readers split on it.

use crate::{Clip, Error, Quat, Skeleton, Transform, Vec3};
use glam::EulerRot;
use std::fmt::Write as _;

/// The frame time [`write()`] picks for a clip when asked for none: the clip's own spacing if even, else its middle
/// spacing, but no shorter than 1/120 s. A clip of one pose gets 1/30 s.
#[must_use]
pub fn frame_time(clip: &Clip) -> f32 {
    if clip.times.len() < 2 {
        return 1.0 / 30.0;
    }
    if let Some(step) = even(&clip.times) {
        return step;
    }
    let mut steps: Vec<f32> = clip.times.windows(2).map(|w| w[1] - w[0]).collect();
    steps.sort_by(f32::total_cmp);
    steps[steps.len() / 2].max(1.0 / 120.0)
}

/// The spacing of evenly spaced times, to within a thousandth of it.
fn even(times: &[f32]) -> Option<f32> {
    let step = (times[times.len() - 1] - times[0]) / (times.len() - 1) as f32;
    let even =
        step > 0.0 && (times.iter().enumerate()).all(|(i, &t)| (t - (times[0] + i as f32 * step)).abs() <= step * 1e-3);
    even.then_some(step)
}

/// `clip` on `skeleton` as BVH text, one frame every `frame_time` seconds ([`frame_time`]'s pick if `None`).
///
/// # Errors
/// [`Error::ClipTimes`] if the clip has no times, times that don't rise, or `frame_time` isn't a positive number;
/// [`Error::ClipSize`] or [`Error::PoseSize`] if its poses don't fit its times or the skeleton.
pub fn write(skeleton: &Skeleton, clip: &Clip, frame_time: Option<f32>) -> Result<String, Error> {
    clip.check(skeleton.len())?;
    let times = &clip.times;
    if times.is_empty() || times.windows(2).any(|w| w[1] <= w[0]) || !times.iter().all(|t| t.is_finite()) {
        return Err(Error::ClipTimes);
    }
    let step = frame_time.unwrap_or_else(|| self::frame_time(clip));
    if !(step.is_finite() && step > 0.0) {
        return Err(Error::ClipTimes);
    }

    // The poses to write: the clip's own when they're evenly spaced at this step, else samples of it.
    let own = times.len() == 1 || even(times).is_some_and(|s| (s - step).abs() <= s * 1e-3);
    let sampled: Vec<Vec<Transform>>;
    let poses: &[Vec<Transform>] = if own {
        &clip.poses
    } else {
        let span = times[times.len() - 1] - times[0];
        let count = (span / step + 1e-3).floor() as usize + 1;
        sampled = (0..count).map(|k| sample(clip, times[0] + k as f32 * step)).collect();
        &sampled
    };

    let bones = skeleton.bones();
    let rest = skeleton.world(&skeleton.rest());
    let rest_turn: Vec<Quat> = rest.iter().map(|p| p.rotation.inverse()).collect();
    let offset = |b: usize| -> Vec3 {
        let at = rest[b].position();
        bones[b].parent.map_or(at, |p| at - rest[p].position())
    };

    // Each frame's world-aligned turns and offsets, joint by joint.
    let extent = rest.iter().map(|p| p.position().length()).fold(0.0, f32::max).max(1e-6);
    let mut moves: Vec<bool> = bones.iter().map(|b| b.parent.is_none()).collect();
    let frames: Vec<Vec<(Quat, Vec3)>> = poses
        .iter()
        .map(|pose| {
            let placed = skeleton.world(pose);
            let turn: Vec<Quat> = (placed.iter().zip(&rest_turn))
                .map(|(p, r)| (p.rotation * *r).normalize())
                .collect();
            (0..bones.len())
                .map(|b| match bones[b].parent {
                    None => (turn[b], placed[b].position()),
                    Some(p) => {
                        let up = turn[p].inverse();
                        let at = up * (placed[b].position() - placed[p].position());
                        if (at - offset(b)).length() > extent * 1e-4 {
                            moves[b] = true;
                        }
                        ((up * turn[b]).normalize(), at)
                    }
                })
                .collect()
        })
        .collect();

    let mut out = String::from("HIERARCHY\n");
    let children = skeleton.children();
    for (b, bone) in bones.iter().enumerate() {
        if bone.parent.is_none() {
            hierarchy(&mut out, skeleton, &children, &moves, &offset, b, 0);
        }
    }
    let _ = write!(
        out,
        "MOTION\nFrames: {}\nFrame Time: {}\n",
        frames.len(),
        number(step, 7)
    );

    let mut last: Vec<Option<Vec3>> = vec![None; bones.len()];
    let per_row = 30 * bones.len() + 24 * moves.iter().filter(|&&m| m).count();
    out.reserve(frames.len() * per_row);
    for frame in &frames {
        for (b, &(turn, at)) in frame.iter().enumerate() {
            if moves[b] {
                for v in [at.x, at.y, at.z] {
                    push_number(&mut out, v, 5);
                    out.push(' ');
                }
            }
            let angles = euler(turn, last[b]);
            last[b] = Some(angles);
            for v in [angles.x, angles.y, angles.z] {
                push_number(&mut out, v, 4);
                out.push(' ');
            }
        }
        out.pop();
        out.push('\n');
    }
    Ok(out)
}

/// Bone `b`'s block and its children's, indented by `depth`.
fn hierarchy(
    out: &mut String,
    skeleton: &Skeleton,
    children: &[Vec<usize>],
    moves: &[bool],
    offset: &dyn Fn(usize) -> Vec3,
    b: usize,
    depth: usize,
) {
    let pad = "  ".repeat(depth);
    let bone = &skeleton.bones()[b];
    let kind = if bone.parent.is_none() { "ROOT" } else { "JOINT" };
    let name: String = bone
        .name
        .chars()
        .map(|c| if c.is_whitespace() { '_' } else { c })
        .collect();
    let name = if name.is_empty() { format!("bone{b}") } else { name };
    let o = offset(b);
    let _ = writeln!(out, "{pad}{kind} {name}\n{pad}{{\n{pad}  OFFSET {}", triple(o));
    let channels = if moves[b] {
        "CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation"
    } else {
        "CHANNELS 3 Zrotation Xrotation Yrotation"
    };
    let _ = writeln!(out, "{pad}  {channels}");
    if children[b].is_empty() {
        let end = if bone.parent.is_some() { o } else { Vec3::ZERO };
        let _ = writeln!(
            out,
            "{pad}  End Site\n{pad}  {{\n{pad}    OFFSET {}\n{pad}  }}",
            triple(end)
        );
    }
    for &c in &children[b] {
        hierarchy(out, skeleton, children, moves, offset, c, depth + 1);
    }
    let _ = writeln!(out, "{pad}}}");
}

fn triple(v: Vec3) -> String {
    [v.x, v.y, v.z].map(|c| number(c, 5)).join(" ")
}

fn number(v: f32, places: u32) -> String {
    let mut s = String::new();
    push_number(&mut s, v, places);
    s
}

/// `v` to `places` decimals (up to 9), without trailing zeros or a negative zero: `0.5`, `-12`, `0`.
fn push_number(out: &mut String, v: f32, places: u32) {
    let scale = 10u64.pow(places);
    let n = (f64::from(v) * scale as f64).round();
    if !n.is_finite() || n.abs() >= 9e18 {
        let _ = write!(out, "{v}");
        return;
    }
    let n = n as i64;
    if n < 0 {
        out.push('-');
    }
    let n = n.unsigned_abs();
    let (whole, mut part) = (n / scale, n % scale);
    let _ = write!(out, "{whole}");
    if part != 0 {
        let mut digits = places as usize;
        while part % 10 == 0 {
            part /= 10;
            digits -= 1;
        }
        let _ = write!(out, ".{part:0digits$}");
    }
}

/// `q` as Z, X and Y angles in degrees (q = Rz·Rx·Ry), of the two such triples the one nearer `near`, each angle
/// within half a turn of its counterpart there.
fn euler(q: Quat, near: Option<Vec3>) -> Vec3 {
    let (z, x, y) = q.to_euler(EulerRot::ZXY);
    let a = Vec3::new(z, x, y) * (180.0 / std::f32::consts::PI);
    let Some(near) = near else { return a };
    let b = Vec3::new(a.x + 180.0, 180.0 - a.y, a.z + 180.0);
    let wrap = |v: Vec3| -> Vec3 {
        let d = v - near;
        v - (d / 360.0).round() * 360.0
    };
    let (a, b) = (wrap(a), wrap(b));
    if (a - near).length_squared() <= (b - near).length_squared() {
        a
    } else {
        b
    }
}

/// The pose at time `t`, between the clip's keys either side of it.
fn sample(clip: &Clip, t: f32) -> Vec<Transform> {
    let times = &clip.times;
    let i = times.partition_point(|&k| k <= t);
    if i == 0 {
        return clip.poses[0].clone();
    }
    if i == times.len() {
        return clip.poses[i - 1].clone();
    }
    let f = (t - times[i - 1]) / (times[i] - times[i - 1]);
    (clip.poses[i - 1].iter().zip(&clip.poses[i]))
        .map(|(a, b)| Transform {
            translation: a.translation.lerp(b.translation, f),
            rotation: a.rotation.slerp(b.rotation, f).normalize(),
            scale: a.scale.lerp(b.scale, f),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bone;

    #[test]
    fn angles_compose_as_z_x_y() {
        for q in [
            Quat::from_euler(EulerRot::XYZ, 0.3, -1.1, 2.0),
            Quat::from_rotation_x(1.5),
            Quat::from_axis_angle(Vec3::new(1.0, 2.0, -0.5).normalize(), 2.7),
        ] {
            let a = euler(q, None) * (std::f32::consts::PI / 180.0);
            let back = Quat::from_rotation_z(a.x) * Quat::from_rotation_x(a.y) * Quat::from_rotation_y(a.z);
            assert!(back.dot(q).abs() > 0.99999, "{q} -> {a} -> {back}");
        }
    }

    #[test]
    fn angles_stay_near_the_last() {
        let mut last = None;
        for k in 0..200 {
            let q = Quat::from_rotation_y(k as f32 * 0.1) * Quat::from_rotation_x(0.2);
            let a = euler(q, last);
            if let Some(l) = last {
                assert!((a - l).abs().max_element() < 10.0, "{l} -> {a}");
            }
            last = Some(a);
        }
        assert!(last.unwrap().z > 1000.0, "a turn keeps counting up");
    }

    fn arm() -> Skeleton {
        let bone = |name: &str, parent, rest| Bone {
            name: name.into(),
            parent,
            rest,
        };
        // Axes that aren't the world's at rest, and a scaled root.
        let turned = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        Skeleton::new(vec![
            bone(
                "my root",
                None,
                Transform {
                    translation: Vec3::new(0.0, 100.0, 0.0),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::splat(0.01),
                },
            ),
            bone("upper", Some(0), Transform::new(Vec3::new(0.0, 50.0, 0.0), turned)),
            bone(
                "lower",
                Some(1),
                Transform::new(Vec3::new(40.0, 0.0, 0.0), Quat::IDENTITY),
            ),
        ])
        .unwrap()
    }

    #[test]
    fn rest_is_the_skeletons_own_with_world_axes() {
        let s = arm();
        let clip = Clip {
            times: vec![0.0],
            poses: vec![s.rest()],
        };
        let text = write(&s, &clip, None).unwrap();
        assert!(
            text.contains("ROOT my_root\n{\n  OFFSET 0 100 0\n  CHANNELS 6"),
            "{text}"
        );
        assert!(
            text.contains("JOINT upper\n  {\n    OFFSET 0 0.5 0\n    CHANNELS 3"),
            "{text}"
        );
        // The lower arm sits 0.4 up the turned upper arm: straight up in the world.
        assert!(text.contains("OFFSET 0 0.4 0\n      CHANNELS 3"), "{text}");
        assert!(
            text.ends_with("Frame Time: 0.0333333\n0 100 0 0 0 0 0 0 0 0 0 0\n"),
            "{text}"
        );
    }

    #[test]
    fn uneven_times_are_sampled() {
        let s = arm();
        let mut bent = s.rest();
        bent[2].rotation = Quat::from_rotation_z(1.0);
        let clip = Clip {
            times: vec![0.0, 0.1, 1.0],
            poses: vec![s.rest(), s.rest(), bent],
        };
        assert_eq!(frame_time(&clip), 0.9);
        let text = write(&s, &clip, Some(0.25)).unwrap();
        assert!(text.contains("Frames: 5\nFrame Time: 0.25\n"), "{text}");
        let last: Vec<&str> = text.lines().last().unwrap().split(' ').collect();
        assert_eq!(last[9], number(1f32.to_degrees(), 4));
        assert_eq!(write(&s, &clip, Some(0.0)), Err(Error::ClipTimes));
    }
}
