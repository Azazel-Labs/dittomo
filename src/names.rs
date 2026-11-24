//! Bone names: their words, side and number, with the many spellings rigs use for the same part brought to one.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Side {
    Left,
    Right,
    Middle,
}

/// What a bone's name says about it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Name {
    pub side: Side,
    /// Its words, each in one spelling for its part, without the ones that say nothing (`mixamorig`, `DEF`, `Bone`).
    pub words: Vec<String>,
    /// The last number in it (`Spine2`, `spine_03`, `Bone.004`).
    pub number: Option<u32>,
}

/// Words that say nothing about which part a bone is.
const NOISE: &[&str] = &[
    "mixamorig",
    "bip",
    "def",
    "org",
    "mch",
    "jnt",
    "joint",
    "bone",
    "bn",
    "j",
    "b",
    "cc",
    "base",
    "rig",
    "armature",
    "sk",
    "skel",
    "bend",
    "drv",
    "jj",
    "valvebiped",
    "npc",
    "ctrl",
    "x",
    // The middle of the body, as VRoid marks it (`J_Bip_C_Spine`).
    "c",
    "center",
    "centre",
];

/// Two words that make one.
const PAIRS: &[(&str, &str, &str)] = &[
    ("upper", "arm", "upperarm"),
    ("up", "arm", "upperarm"),
    ("lower", "arm", "forearm"),
    ("lo", "arm", "forearm"),
    ("fore", "arm", "forearm"),
    ("up", "leg", "thigh"),
    ("upper", "leg", "thigh"),
    ("lower", "leg", "shin"),
    ("lo", "leg", "shin"),
    ("toe", "base", "toe"),
    ("upper", "chest", "upperchest"),
    ("chest", "upper", "upperchest"),
    ("chest", "lower", "chest"),
    ("abdomen", "upper", "chest"),
    ("abdomen", "lower", "spine"),
    ("head", "top", "headtop"),
    ("head", "end", "headtop"),
    ("collar", "bone", "clavicle"),
];

/// One spelling for each part.
const SAME: &[(&str, &str)] = &[
    ("pelvis", "hips"),
    ("pelvic", "hips"),
    ("hip", "hips"),
    ("abdomen", "spine"),
    ("torso", "spine"),
    ("back", "spine"),
    ("waist", "spine"),
    ("chestupper", "upperchest"),
    ("collar", "clavicle"),
    ("collarbone", "clavicle"),
    ("clav", "clavicle"),
    ("shoulder", "clavicle"),
    ("uparm", "upperarm"),
    ("humerus", "upperarm"),
    ("shldr", "upperarm"),
    ("lowerarm", "forearm"),
    ("loarm", "forearm"),
    ("elbow", "forearm"),
    ("radius", "forearm"),
    ("ulna", "forearm"),
    ("wrist", "hand"),
    ("upleg", "thigh"),
    ("upperleg", "thigh"),
    ("femur", "thigh"),
    ("calf", "shin"),
    ("lowerleg", "shin"),
    ("loleg", "shin"),
    ("knee", "shin"),
    ("tibia", "shin"),
    ("ankle", "foot"),
    ("toes", "toe"),
    ("ball", "toe"),
    ("toebase", "toe"),
    ("pointer", "index"),
    ("mid", "middle"),
    ("pinkie", "pinky"),
    ("little", "pinky"),
    ("small", "pinky"),
    ("headtopend", "headtop"),
];

const FINGERS: &[&str] = &["thumb", "index", "middle", "ring", "pinky"];

/// Parts that are near each other, for partial likeness.
const NEAR: &[(&str, &str, f32)] = &[
    ("spine", "chest", 0.6),
    ("chest", "upperchest", 0.7),
    ("spine", "upperchest", 0.5),
    ("hips", "spine", 0.3),
    ("clavicle", "upperarm", 0.4),
    ("neck", "head", 0.3),
    ("head", "headtop", 0.5),
    ("foot", "toe", 0.3),
    // A limb's whole chain, numbered (`arm_joint_L_2`).
    ("arm", "upperarm", 0.6),
    ("arm", "forearm", 0.6),
    ("arm", "hand", 0.3),
    ("arm", "clavicle", 0.3),
    ("leg", "thigh", 0.6),
    ("leg", "shin", 0.6),
    ("leg", "foot", 0.3),
    ("hand", "finger", 0.3),
];

/// What a bone's name says.
pub(crate) fn parse(name: &str) -> Name {
    let tail = name.rsplit([':', '|', '/']).next().unwrap_or(name);
    let raw = split(tail);
    let mut side = Side::Middle;
    let mut words: Vec<String> = Vec::new();
    let mut number = None;
    let mut skip_number = false;
    for w in raw {
        if w.bytes().all(|c| c.is_ascii_digit()) {
            if !std::mem::take(&mut skip_number) {
                number = w.parse().ok();
            }
            continue;
        }
        skip_number = false;
        match w.as_str() {
            "l" | "left" | "lf" | "lft" => side = Side::Left,
            "r" | "right" | "rt" | "rgt" => side = Side::Right,
            "bip" | "bip0" => skip_number = true,
            w if NOISE.contains(&w) => {}
            _ => words.push(w),
        }
    }
    let mut joined = Vec::with_capacity(words.len());
    let mut i = 0;
    while i < words.len() {
        if let Some(&(_, _, one)) = words
            .get(i + 1)
            .and_then(|next| PAIRS.iter().find(|(a, b, _)| *a == words[i] && *b == next.as_str()))
        {
            joined.push(one.to_owned());
            i += 2;
        } else {
            joined.push(words[i].clone());
            i += 1;
        }
    }
    let mut words: Vec<String> = joined
        .into_iter()
        .map(|w| {
            SAME.iter()
                .find(|(a, _)| *a == w)
                .map_or(w, |(_, one)| (*one).to_owned())
        })
        .map(|w| match (w.as_str(), number) {
            // Alone, an arm is the upper arm and a leg the shin (`LeftArm`, `LeftLeg`); numbered, the whole limb's
            // chain.
            ("arm", None) => "upperarm".to_owned(),
            ("leg", None) => "shin".to_owned(),
            _ => w,
        })
        .collect();
    if words.iter().any(|w| FINGERS.contains(&w.as_str())) {
        words.retain(|w| w != "hand" && w != "finger" && w != "fingers");
    }
    words.dedup();
    Name { side, words, number }
}

/// Lowercase words, split at anything not a letter or digit, where lowercase turns upper, and between letters and
/// digits (`LeftUpLeg` → left up leg, `spine_01` → spine 01, `HTMLBone2` → html bone 2).
fn split(s: &str) -> Vec<String> {
    let chars: Vec<char> = s.chars().collect();
    let mut out = Vec::new();
    let mut cur = String::new();
    for (i, &c) in chars.iter().enumerate() {
        if !c.is_alphanumeric() {
            if !cur.is_empty() {
                out.push(std::mem::take(&mut cur));
            }
            continue;
        }
        if !cur.is_empty() {
            // What's in `cur` came from the characters just before this one.
            let prev = chars[i - 1];
            let next_lower = chars.get(i + 1).is_some_and(|n| n.is_lowercase());
            let cut = (prev.is_lowercase() && c.is_uppercase())
                || (prev.is_ascii_digit() != c.is_ascii_digit())
                || (prev.is_uppercase() && c.is_uppercase() && next_lower);
            if cut {
                out.push(std::mem::take(&mut cur));
            }
        }
        cur.extend(c.to_lowercase());
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

/// Whether the names put the bones on opposite sides.
pub(crate) fn opposed(a: &Name, b: &Name) -> bool {
    matches!((a.side, b.side), (Side::Left, Side::Right) | (Side::Right, Side::Left))
}

/// How alike two names are, from 0 to 1; none when either says nothing.
pub(crate) fn likeness(a: &Name, b: &Name) -> Option<f32> {
    score(a, b, |i, j| word(&a.words[i], &b.words[j]))
}

/// How alike two words are, from 0 to 1.
fn word(x: &str, y: &str) -> f32 {
    if x == y {
        return 1.0;
    }
    let fx = if FINGERS.contains(&x) { "finger" } else { x };
    let fy = if FINGERS.contains(&y) { "finger" } else { y };
    NEAR.iter()
        .find(|(p, q, _)| (*p == fx && *q == fy) || (*p == fy && *q == fx))
        .map_or(0.0, |n| n.2)
}

/// [`likeness`], given how alike `a`'s word `i` and `b`'s word `j` are.
fn score(a: &Name, b: &Name, word: impl Fn(usize, usize) -> f32) -> Option<f32> {
    let (n, m) = (a.words.len(), b.words.len());
    if n == 0 || m == 0 {
        return None;
    }
    // How well each of one name's words is met in the other's, on the whole.
    let half = |n: usize, m: usize, word: &dyn Fn(usize, usize) -> f32| -> f32 {
        (0..n)
            .map(|i| (0..m).map(|j| word(i, j)).fold(0.0, f32::max))
            .sum::<f32>()
            / n as f32
    };
    let mut s = half(n, m, &word).min(half(m, n, &|i, j| word(j, i)));
    if (a.side == Side::Middle) != (b.side == Side::Middle) {
        s *= 0.7;
    }
    if a.number.is_some() && b.number.is_some() && a.number != b.number {
        s *= 0.85;
    }
    Some(s)
}

/// Two rigs' names with their words numbered, and how alike each two words are, so that every pair of names can be
/// weighed without comparing letters.
pub(crate) struct Lexicon {
    /// Each rig's names' words by number, one name after another, and where each name's are.
    words: [Vec<usize>; 2],
    spans: [Vec<std::ops::Range<usize>>; 2],
    /// How alike words `i` and `j` are, at `i * count + j`.
    alike: Vec<f32>,
    count: usize,
}

impl Lexicon {
    pub(crate) fn new(first: &[Name], second: &[Name]) -> Self {
        let mut numbers: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
        let mut all: Vec<&str> = Vec::new();
        let (mut words, mut spans) = ([Vec::new(), Vec::new()], [Vec::new(), Vec::new()]);
        for (k, names) in [first, second].into_iter().enumerate() {
            for name in names {
                let start = words[k].len();
                for w in &name.words {
                    let number = *numbers.entry(w.as_str()).or_insert_with(|| {
                        all.push(w);
                        all.len() - 1
                    });
                    words[k].push(number);
                }
                spans[k].push(start..words[k].len());
            }
        }
        Self {
            words,
            spans,
            alike: all.iter().flat_map(|x| all.iter().map(|y| word(x, y))).collect(),
            count: all.len(),
        }
    }

    /// As [`likeness`]: the first rig's name `a`, which is `na`, and the second's `b`, `nb`.
    pub(crate) fn likeness(&self, a: usize, na: &Name, b: usize, nb: &Name) -> Option<f32> {
        let (wa, wb) = (
            &self.words[0][self.spans[0][a].clone()],
            &self.words[1][self.spans[1][b].clone()],
        );
        score(na, nb, |i, j| self.alike[wa[i] * self.count + wb[j]])
    }
}

#[cfg(test)]
mod tests {
    use super::{Side, likeness, parse};

    fn words(name: &str) -> (Side, Vec<String>, Option<u32>) {
        let n = parse(name);
        (n.side, n.words, n.number)
    }

    #[test]
    fn spellings_from_different_tools_come_to_one() {
        let w = |s: &[&str]| s.iter().map(|&x| x.to_owned()).collect::<Vec<_>>();
        assert_eq!(words("mixamorig:LeftUpLeg"), (Side::Left, w(&["thigh"]), None));
        assert_eq!(words("thigh_l"), (Side::Left, w(&["thigh"]), None));
        assert_eq!(words("DEF-thigh.L"), (Side::Left, w(&["thigh"]), None));
        assert_eq!(words("LeftUpperLeg"), (Side::Left, w(&["thigh"]), None));
        assert_eq!(words("mixamorig:LeftForeArm"), (Side::Left, w(&["forearm"]), None));
        assert_eq!(words("lowerarm_r"), (Side::Right, w(&["forearm"]), None));
        assert_eq!(words("CC_Base_R_Forearm"), (Side::Right, w(&["forearm"]), None));
        assert_eq!(words("mixamorig:Spine2"), (Side::Middle, w(&["spine"]), Some(2)));
        assert_eq!(words("spine_03"), (Side::Middle, w(&["spine"]), Some(3)));
        assert_eq!(words("Bip01 Spine1"), (Side::Middle, w(&["spine"]), Some(1)));
        assert_eq!(words("mixamorig:LeftHandIndex1"), (Side::Left, w(&["index"]), Some(1)));
        assert_eq!(words("Bone.004"), (Side::Middle, w(&[]), Some(4)));
        assert_eq!(
            words("upperarm_twist_01_l"),
            (Side::Left, w(&["upperarm", "twist"]), Some(1))
        );
        assert_eq!(words("pelvis"), (Side::Middle, w(&["hips"]), None));
        assert_eq!(words("J_Bip_C_UpperChest"), (Side::Middle, w(&["upperchest"]), None));
        assert_eq!(words("J_Bip_L_UpperArm"), (Side::Left, w(&["upperarm"]), None));
        // Numbered, an arm or a leg is a joint somewhere along the limb.
        assert_eq!(words("arm_joint_L_2"), (Side::Left, w(&["arm"]), Some(2)));
        assert_eq!(words("leg_joint_R_1"), (Side::Right, w(&["leg"]), Some(1)));
        assert_eq!(words("mixamorig:LeftLeg"), (Side::Left, w(&["shin"]), None));
    }

    #[test]
    fn like_names_are_alike_and_others_less() {
        let l = |a: &str, b: &str| likeness(&parse(a), &parse(b));
        assert!(l("mixamorig:LeftArm", "upperarm_l").unwrap() > 0.99);
        assert!(l("upperarm_twist_01_l", "upperarm_l").unwrap() < 0.6);
        assert!(l("chest", "spine_03").unwrap() > 0.4);
        assert!(l("hand_l", "foot_l").unwrap() < 0.01);
        assert_eq!(l("Bone.001", "hand_l"), None);
        assert!(l("leg_joint_L_1", "thigh_l").unwrap() > 0.5);
        assert!(l("arm_joint_L_2", "lowerarm_l").unwrap() > 0.5);
    }
}
