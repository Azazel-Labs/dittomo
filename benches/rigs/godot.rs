//! Godot's humanoid auto bone mapping, as an engine baseline to score against.
//!
//! Ported from Godot's `editor/scene/3d/bone_map_editor_plugin.cpp` (`BoneMapper::auto_mapping_process` and its
//! helpers) at commit 25d0522345fb8b58cc469b6760dd73c04b0f0186, with the profile bone names of `SkeletonProfileHumanoid`
//! from `scene/resources/skeleton_profile.cpp`, the string helpers it leans on from `core/string/ustring.cpp`, and the
//! bone name cleanup from `modules/gltf/gltf_document.cpp`.
//!
//! Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md).
//! Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.
//! Godot is under the MIT license: permission is granted, free of charge, to deal in the software without
//! restriction, provided the copyright notice and permission notice are kept; it comes without warranty. The full
//! text is at <https://github.com/godotengine/godot/blob/master/LICENSE.txt>.
//!
//! It's a faithful port, quirks included: the aim is Godot's own numbers, not a better matcher. Each regex Godot uses
//! is matched by hand, with the original pattern in a comment. Where it can't be exact:
//!
//! - Godot runs on the skeleton its glTF importer builds; this runs on ours, the skins' joints and every node above
//!   them, in the file's order, parents first. Godot leaves the non-joint nodes above the joints (an armature node,
//!   say) out, and names unnamed joints `bone`, `bone_2` and so on where we say `node3`. Neither kind of name matches
//!   any of Godot's words, so the nodes above only show where a walk runs to the top: Root, and the spine when the
//!   bone taken for Hips isn't below the chest (cesium-man's Spine lands on its `Z_UP` node, where Godot's would be
//!   its topmost joint). That spine bone scores the same either way here.
//! - Godot makes clashing bone names unique (`_gen_unique_bone_name`, adding `_2` and up); clashes are rare and
//!   aren't redone here, so a name found by Godot's `find_bone` goes to the first bone with it.
//! - Case and word breaks use Rust's Unicode tables where Godot has its own; for ASCII names they agree.
//! - The rest pose is used only to guess Root (a bone at the origin). Godot is +Y up with its profile facing +Z, as
//!   glTF is, but nothing here depends on the axes: the mapper tells left from right by name alone, never by side.

use std::collections::VecDeque;

use dittomo::Skeleton;

/// `SkeletonProfileHumanoid`'s bones, in the profile's order.
const PROFILE: [&str; 56] = [
    "Root",
    "Hips",
    "Spine",
    "Chest",
    "UpperChest",
    "Neck",
    "Head",
    "LeftEye",
    "RightEye",
    "Jaw",
    "LeftShoulder",
    "LeftUpperArm",
    "LeftLowerArm",
    "LeftHand",
    "LeftThumbMetacarpal",
    "LeftThumbProximal",
    "LeftThumbDistal",
    "LeftIndexProximal",
    "LeftIndexIntermediate",
    "LeftIndexDistal",
    "LeftMiddleProximal",
    "LeftMiddleIntermediate",
    "LeftMiddleDistal",
    "LeftRingProximal",
    "LeftRingIntermediate",
    "LeftRingDistal",
    "LeftLittleProximal",
    "LeftLittleIntermediate",
    "LeftLittleDistal",
    "RightShoulder",
    "RightUpperArm",
    "RightLowerArm",
    "RightHand",
    "RightThumbMetacarpal",
    "RightThumbProximal",
    "RightThumbDistal",
    "RightIndexProximal",
    "RightIndexIntermediate",
    "RightIndexDistal",
    "RightMiddleProximal",
    "RightMiddleIntermediate",
    "RightMiddleDistal",
    "RightRingProximal",
    "RightRingIntermediate",
    "RightRingDistal",
    "RightLittleProximal",
    "RightLittleIntermediate",
    "RightLittleDistal",
    "LeftUpperLeg",
    "LeftLowerLeg",
    "LeftFoot",
    "LeftToes",
    "RightUpperLeg",
    "RightLowerLeg",
    "RightFoot",
    "RightToes",
];

/// The finger bones of each hand, thumb to little finger, as `auto_mapping_process` lists them.
const LEFT_FINGERS: [[&str; 3]; 5] = [
    ["LeftThumbMetacarpal", "LeftThumbProximal", "LeftThumbDistal"],
    ["LeftIndexProximal", "LeftIndexIntermediate", "LeftIndexDistal"],
    ["LeftMiddleProximal", "LeftMiddleIntermediate", "LeftMiddleDistal"],
    ["LeftRingProximal", "LeftRingIntermediate", "LeftRingDistal"],
    ["LeftLittleProximal", "LeftLittleIntermediate", "LeftLittleDistal"],
];
const RIGHT_FINGERS: [[&str; 3]; 5] = [
    ["RightThumbMetacarpal", "RightThumbProximal", "RightThumbDistal"],
    ["RightIndexProximal", "RightIndexIntermediate", "RightIndexDistal"],
    ["RightMiddleProximal", "RightMiddleIntermediate", "RightMiddleDistal"],
    ["RightRingProximal", "RightRingIntermediate", "RightRingDistal"],
    ["RightLittleProximal", "RightLittleIntermediate", "RightLittleDistal"],
];

/// Each bone's Godot humanoid profile name, or `None`. A bone Godot gives more than one name keeps the first in the
/// profile's order, as Godot's bone renamer does when it retargets on import (`BoneMap::find_profile_bone_name`).
pub fn auto_map(skeleton: &Skeleton) -> Vec<Option<&'static str>> {
    let mut mapper = Mapper::new(skeleton);
    mapper.auto_mapping_process();
    let mut out = vec![None; skeleton.len()];
    for (name, bone) in PROFILE.iter().zip(&mapper.map) {
        if let Some(b) = *bone
            && out[b].is_none()
        {
            out[b] = Some(*name);
        }
    }
    out
}

/// `BoneMapper::BoneSegregation`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    None,
    Left,
    Right,
}

/// A regex Godot matches against a lowercased bone name, anywhere in it.
#[derive(Clone, Copy)]
enum Pat {
    /// A literal word, such as `"hip"`.
    Word(&'static str),
    /// Alternatives, such as `"thumb|pollex"`.
    Any(&'static [&'static str]),
    /// `"(a|b).*c"`: one of the first words, then the last later on.
    Then(&'static [&'static str], &'static str),
    /// `"eye(?!.*(brow|lash|lid))"`.
    Eye,
}

impl Pat {
    fn search(self, s: &str) -> bool {
        match self {
            Pat::Word(w) => s.contains(w),
            Pat::Any(ws) => ws.iter().any(|w| s.contains(w)),
            Pat::Then(firsts, last) => firsts.iter().any(|f| at(s, f).any(|i| s[i + f.len()..].contains(last))),
            Pat::Eye => at(s, "eye").any(|i| {
                let rest = &s[i + 3..];
                !["brow", "lash", "lid"].iter().any(|w| rest.contains(w))
            }),
        }
    }
}

/// Every place `word` starts in `s`, overlapping ones too, as a regex would try them.
fn at<'s>(s: &'s str, word: &'s str) -> impl Iterator<Item = usize> + 's {
    (0..s.len()).filter(move |&i| s.as_bytes()[i..].starts_with(word.as_bytes()))
}

use Pat::{Any, Then, Word};

/// `String::to_lower`: each character to its lowercase one.
fn to_lower(s: &str) -> String {
    s.chars().map(lower_char).collect()
}

fn lower_char(c: char) -> char {
    let mut l = c.to_lowercase();
    match (l.next(), l.next()) {
        (Some(l), None) => l,
        _ => c,
    }
}

/// `String::to_snake_case`, through `String::_separate_compound_words`.
fn to_snake_case(s: &str) -> String {
    let c: Vec<char> = s.chars().collect();
    if c.is_empty() {
        return String::new();
    }
    let digit = |c: char| c.is_ascii_digit();
    let mut words: Vec<char> = Vec::with_capacity(c.len() + 8);
    let mut start = 0;
    let (mut prev_upper, mut prev_lower, mut prev_digit) = (c[0].is_uppercase(), c[0].is_lowercase(), digit(c[0]));
    for i in 1..c.len() {
        let (upper, lower, dig) = (c[i].is_uppercase(), c[i].is_lowercase(), digit(c[i]));
        let next_lower = c.get(i + 1).is_some_and(|n| n.is_lowercase());
        let a = prev_lower && upper; // aA
        let b = (prev_upper || prev_digit) && upper && next_lower; // AAa, 2Aa
        let cc = prev_digit && lower && next_lower; // 2aa
        let d = (prev_upper || prev_lower) && dig; // A2, a2
        if a || b || cc || d {
            words.extend(&c[start..i]);
            words.push(' ');
            start = i;
        }
        (prev_upper, prev_lower, prev_digit) = (upper, lower, dig);
    }
    words.extend(&c[start..]);
    // Whitespace, underscores and hyphens become spaces, then everything is lowercased and spaces become underscores.
    words
        .into_iter()
        .map(|ch| {
            if ch.is_whitespace() || ch == '_' || matches!(ch, '-' | '\u{2010}' | '\u{2011}') {
                '_'
            } else {
                lower_char(ch)
            }
        })
        .collect()
}

/// `(?<![a-zA-Z])left` (or `right`): the word, not right after a letter.
fn side_word(s: &str, word: &str) -> bool {
    let b = s.as_bytes();
    at(s, word).any(|i| i == 0 || !b[i - 1].is_ascii_alphabetic())
}

/// `(?<![a-zA-Z0-9])l(?![a-zA-Z0-9])` (or `r`): the letter alone, with no letter or digit on either side.
fn side_letter(s: &str, letter: u8) -> bool {
    let b = s.as_bytes();
    (0..b.len()).any(|i| {
        b[i] == letter
            && (i == 0 || !b[i - 1].is_ascii_alphanumeric())
            && b.get(i + 1).is_none_or(|n| !n.is_ascii_alphanumeric())
    })
}

/// `BoneMapper::guess_bone_segregation`.
fn guess_bone_segregation(name: &str) -> Side {
    let fixed = to_snake_case(name);
    if side_word(&fixed, "left") {
        return Side::Left;
    }
    if side_word(&fixed, "right") {
        return Side::Right;
    }
    if side_letter(&fixed, b'l') {
        return Side::Left;
    }
    if side_letter(&fixed, b'r') {
        return Side::Right;
    }
    Side::None
}

/// `GLTFDocument::_sanitize_bone_name`: the importer turns `:` and `/` into `_` before anything else sees a name.
fn sanitize_bone_name(name: &str) -> String {
    name.replace([':', '/'], "_")
}

struct Mapper<'a> {
    skeleton: &'a Skeleton,
    /// Each bone's name as Godot's importer leaves it.
    names: Vec<String>,
    /// And lowercased, as `is_match_with_bone_name` matches it.
    lower: Vec<String>,
    children: Vec<Vec<usize>>,
    /// The skeleton bone set for each profile bone, by `_set_skeleton_bone_name`.
    map: Vec<Option<usize>>,
}

impl<'a> Mapper<'a> {
    fn new(skeleton: &'a Skeleton) -> Self {
        let names: Vec<String> = skeleton.bones().iter().map(|b| sanitize_bone_name(&b.name)).collect();
        let lower = names.iter().map(|n| to_lower(n)).collect();
        // Godot keeps each bone's children in bone order.
        let mut children = vec![Vec::new(); skeleton.len()];
        for (i, b) in skeleton.bones().iter().enumerate() {
            if let Some(p) = b.parent {
                children[p].push(i);
            }
        }
        Self {
            skeleton,
            names,
            lower,
            children,
            map: vec![None; PROFILE.len()],
        }
    }

    fn parent(&self, b: Option<usize>) -> Option<usize> {
        b.and_then(|b| self.skeleton.bones()[b].parent)
    }

    fn children_of(&self, b: Option<usize>) -> Vec<usize> {
        b.map_or_else(Vec::new, |b| self.children[b].clone())
    }

    /// `Skeleton3D::find_bone`: the first bone of that name.
    fn find_bone(&self, name: &str) -> Option<usize> {
        self.names.iter().position(|n| n == name)
    }

    /// `BoneMap::_set_skeleton_bone_name`; `None` clears it, as Godot's empty name does.
    fn set(&mut self, profile: &str, bone: Option<usize>) {
        let i = PROFILE.iter().position(|p| *p == profile).expect("a profile bone");
        self.map[i] = bone;
    }

    /// `is_match_with_bone_name` and the side test together.
    fn hit(&self, b: usize, pat: Pat, side: Side) -> bool {
        pat.search(&self.lower[b]) && guess_bone_segregation(&self.names[b]) == side
    }

    /// `BoneMapper::search_bone_by_name`. With no `child`, the bones under `parent` (or the whole skeleton),
    /// breadth first; with one, the bones from the child's parent up to, not including, `parent`. Each word is tried
    /// in turn, and of the bones it hits the shortest name wins: the first such below, the topmost above. A
    /// `children_count` of `Some(0)` takes only bones without children, `Some(n)` bones with at least n.
    fn search(
        &self,
        picklist: &[Pat],
        side: Side,
        parent: Option<usize>,
        child: Option<usize>,
        children_count: Option<usize>,
    ) -> Option<usize> {
        let len = |b: usize| self.names[b].chars().count();
        for &word in picklist {
            let mut shortest: Option<usize> = None;
            if child.is_none() {
                let mut queue: VecDeque<usize> = match parent {
                    None => (0..self.skeleton.len())
                        .filter(|&b| self.skeleton.bones()[b].parent.is_none())
                        .collect(),
                    Some(p) => self.children[p].iter().copied().collect(),
                };
                while let Some(idx) = queue.pop_front() {
                    let children = &self.children[idx];
                    queue.extend(children);
                    match children_count {
                        Some(0) if !children.is_empty() => continue,
                        Some(n) if n > 0 && children.len() < n => continue,
                        _ => {}
                    }
                    if self.hit(idx, word, side) && shortest.is_none_or(|s| len(idx) < len(s)) {
                        shortest = Some(idx);
                    }
                }
            } else {
                // Godot never passes a children count with a child: its loop would not move on if one ruled a bone out.
                let mut idx = self.parent(child);
                while let Some(i) = idx
                    && Some(i) != parent
                {
                    if self.hit(i, word, side) && shortest.is_none_or(|s| len(i) <= len(s)) {
                        shortest = Some(i);
                    }
                    idx = self.parent(idx);
                }
            }
            if let Some(s) = shortest
                && !self.names[s].is_empty()
            {
                return self.find_bone(&self.names[s]);
            }
        }
        None
    }

    /// `BoneMapper::auto_mapping_process`.
    fn auto_mapping_process(&mut self) {
        use Side::{Left, Right};

        // 1. Guess Hips
        let picklist = [Word("hip"), Word("pelvis"), Word("waist"), Word("torso"), Word("spine")];
        let Some(hips) = self.search(&picklist, Side::None, None, None, None) else {
            return; // If there is no Hips, we cannot guess bone after then.
        };
        self.set("Hips", Some(hips));

        // 2. Guess Root
        let mut search_path = Vec::new();
        let mut bone_idx = self.parent(Some(hips));
        while let Some(b) = bone_idx {
            search_path.push(b);
            bone_idx = self.parent(Some(b));
        }
        let root = match search_path.len() {
            0 => None,
            1 => Some(search_path[0]), // It is only one bone which can be root.
            _ => {
                let rest = self.skeleton.world(&self.skeleton.rest());
                // "root"
                search_path
                    .iter()
                    .find(|&&b| Word("root").search(&self.lower[b])) // Name match is preferred.
                    .or_else(|| {
                        // The bone existing at the origin is appropriate as a root (`Vector3::is_zero_approx`).
                        search_path
                            .iter()
                            .find(|&&b| rest[b].position().abs().max_element() < 0.00001)
                    })
                    .or(search_path.last()) // Ambiguous, but most parental bone selected.
                    .copied()
            }
        };
        self.set("Root", root);

        // 3. Guess Foots
        let picklist = [Word("foot"), Word("ankle")];
        let left_foot = self.search(&picklist, Left, Some(hips), None, None);
        self.set("LeftFoot", left_foot);
        let right_foot = self.search(&picklist, Right, Some(hips), None, None);
        self.set("RightFoot", right_foot);

        // 3-1. Guess LowerLegs
        // "(low|under).*leg", "knee", "shin", "calf", "leg"
        let picklist = [
            Then(&["low", "under"], "leg"),
            Word("knee"),
            Word("shin"),
            Word("calf"),
            Word("leg"),
        ];
        let left_lower_leg = left_foot.and_then(|f| self.search(&picklist, Left, Some(hips), Some(f), None));
        self.set("LeftLowerLeg", left_lower_leg);
        let right_lower_leg = right_foot.and_then(|f| self.search(&picklist, Right, Some(hips), Some(f), None));
        self.set("RightLowerLeg", right_lower_leg);

        // 3-2. Guess UpperLegs
        // "up.*leg", "thigh", "leg"
        let picklist = [Then(&["up"], "leg"), Word("thigh"), Word("leg")];
        let b = left_lower_leg.and_then(|l| self.search(&picklist, Left, Some(hips), Some(l), None));
        self.set("LeftUpperLeg", b);
        let b = right_lower_leg.and_then(|l| self.search(&picklist, Right, Some(hips), Some(l), None));
        self.set("RightUpperLeg", b);

        // 3-3. Guess Toes
        let picklist = [Word("toe"), Word("ball")];
        for (foot, side, name) in [(left_foot, Left, "LeftToes"), (right_foot, Right, "RightToes")] {
            let b = foot.and_then(|f| {
                self.search(&picklist, side, Some(f), None, None).or_else(|| {
                    // Maybe only one child of the Foot is Toes.
                    let c = &self.children[f];
                    (c.len() == 1).then(|| c[0])
                })
            });
            self.set(name, b);
        }

        // 4. Guess Hands
        let picklist = [Word("hand"), Word("wrist"), Word("palm"), Word("fingers")];
        let mut hand_or_palm = [None, None];
        for (k, (side, name)) in [(Left, "LeftHand"), (Right, "RightHand")].into_iter().enumerate() {
            let palm = self
                .search(&picklist, side, Some(hips), None, Some(5))
                // Ambiguous, but try again for fewer finger models.
                .or_else(|| self.search(&picklist, side, Some(hips), None, None));
            // Check for the presence of a wrist, since bones with five children may be palmar.
            let mut hand = palm;
            while let Some(h) = hand {
                match self.search(&picklist, side, Some(hips), Some(h), None) {
                    Some(b) => hand = Some(b),
                    None => break,
                }
            }
            self.set(name, hand);
            hand_or_palm[k] = palm;
        }
        let [left_hand_or_palm, right_hand_or_palm] = hand_or_palm;

        // 4-1. Guess Finger
        if let Some(h) = left_hand_or_palm {
            self.fingers(h, Left, &LEFT_FINGERS);
        }
        if let Some(h) = right_hand_or_palm {
            self.fingers(h, Right, &RIGHT_FINGERS);
        }

        // 5. Guess Arms
        let picklist = [Word("shoulder"), Word("clavicle"), Word("collar")];
        let left_shoulder = self.search(&picklist, Left, Some(hips), None, None);
        self.set("LeftShoulder", left_shoulder);
        let right_shoulder = self.search(&picklist, Right, Some(hips), None, None);
        self.set("RightShoulder", right_shoulder);

        // 5-1. Guess LowerArms
        // "(low|fore).*arm", "elbow", "arm"
        let picklist = [Then(&["low", "fore"], "arm"), Word("elbow"), Word("arm")];
        let lower_arm = |me: &Self, shoulder: Option<usize>, palm: Option<usize>, side| match (shoulder, palm) {
            (Some(s), Some(p)) => me.search(&picklist, side, Some(s), Some(p), None),
            _ => None,
        };
        let left_lower_arm = lower_arm(self, left_shoulder, left_hand_or_palm, Left);
        self.set("LeftLowerArm", left_lower_arm);
        let right_lower_arm = lower_arm(self, right_shoulder, right_hand_or_palm, Right);
        self.set("RightLowerArm", right_lower_arm);

        // 5-2. Guess UpperArms
        // "up.*arm", "arm"
        let picklist = [Then(&["up"], "arm"), Word("arm")];
        for (shoulder, lower, side, name) in [
            (left_shoulder, left_lower_arm, Left, "LeftUpperArm"),
            (right_shoulder, right_lower_arm, Right, "RightUpperArm"),
        ] {
            let b = match (shoulder, lower) {
                (Some(s), Some(l)) => self.search(&picklist, side, Some(s), Some(l), None),
                _ => None,
            };
            self.set(name, b);
        }

        // 6. Guess Neck
        // "neck", then "head" and "face" for no neck model.
        let picklist = [Word("neck"), Word("head"), Word("face")];
        let mut neck = self.search(&picklist, Side::None, Some(hips), None, None);
        if neck.is_none() {
            // If it can't expect by name, search child spine of where the right and left shoulders (or hands) cross.
            let mut ls = left_shoulder.or(left_hand_or_palm);
            let mut rs = right_shoulder.or(right_hand_or_palm);
            if ls.is_some() && rs.is_some() {
                let mut detect = false;
                while ls.is_some() && ls != Some(hips) && rs.is_some() && rs != Some(hips) {
                    ls = self.parent(ls);
                    rs = self.parent(rs);
                    if ls == rs {
                        detect = true;
                        break;
                    }
                }
                if detect {
                    // Both may have run off the top together; Godot then finds no children.
                    // "spine": it would be better to limit the search with "spine" because it could be mistaken
                    // with breast, wing and etc.
                    neck = self
                        .children_of(ls)
                        .into_iter()
                        .filter(|&c| Some(c) != ls && Some(c) != rs)
                        .find(|&c| Word("spine").search(&self.lower[c]));
                }
            }
        }

        // 7. Guess Head
        let picklist = [Word("head"), Word("face")];
        let mut head = self.search(&picklist, Side::None, neck, None, None);
        if head.is_none()
            && let Some(n) = neck
            && self.children[n].len() == 1
        {
            head = Some(self.children[n][0]); // Maybe only one child of the Neck is Head.
        }
        if head.is_none() {
            if neck.is_some() {
                head = neck; // The head animation should have more movement.
                neck = None;
                self.set("Head", head);
            }
        } else {
            self.set("Neck", neck);
            self.set("Head", head);
        }

        let neck_or_head = neck.or(head);
        if let Some(nh) = neck_or_head {
            // 7-1. Guess Eyes
            // "eye(?!.*(brow|lash|lid))"
            let picklist = [Pat::Eye];
            let b = self.search(&picklist, Left, Some(nh), None, None);
            self.set("LeftEye", b);
            let b = self.search(&picklist, Right, Some(nh), None, None);
            self.set("RightEye", b);

            // 7-2. Guess Jaw
            let b = self.search(&[Word("jaw")], Side::None, Some(nh), None, None);
            self.set("Jaw", b);
        }

        // 8. Guess UpperChest or Chest
        if neck_or_head.is_none() {
            return; // Abort.
        }
        let mut chest_or_upper_chest = self.parent(neck_or_head);
        let mut is_appropriate = true;
        for shoulder in [left_shoulder, right_shoulder] {
            if shoulder.is_some() {
                let mut b = self.parent(shoulder);
                let mut detect = false;
                while let Some(i) = b
                    && i != hips
                {
                    if Some(i) == chest_or_upper_chest {
                        detect = true;
                        break;
                    }
                    b = self.parent(b);
                }
                if !detect {
                    is_appropriate = false;
                }
            }
        }
        if !is_appropriate {
            // A missing shoulder has no parent here, as Godot's `get_bone_parent(-1)` gives -1.
            let (lp, rp) = (self.parent(left_shoulder), self.parent(right_shoulder));
            chest_or_upper_chest = if lp == rp { lp } else { None };
        }
        let Some(chest) = chest_or_upper_chest else {
            return; // Will be not able to guess Spines.
        };

        // 9. Guess Spines
        let mut search_path = Vec::new();
        let mut b = self.parent(Some(chest));
        while let Some(i) = b
            && i != hips
        {
            search_path.push(i);
            b = self.parent(b);
        }
        search_path.reverse();
        match search_path.len() {
            0 => self.set("Spine", Some(chest)), // Maybe chibi model...?
            1 => {
                self.set("Spine", Some(search_path[0]));
                self.set("Chest", Some(chest));
            }
            n => {
                self.set("Spine", Some(search_path[0]));
                self.set("Chest", Some(search_path[n - 1])); // Probably UpperChest's parent is appropriate.
                self.set("UpperChest", Some(chest));
            }
        }
    }

    /// Step 4-1 for one hand, from its hand or palm bone. Like Godot, `tips_index` carries over from one finger to the
    /// next, and the thumb's tip count is kept in a bool (`thumb_tips_size`), so it is at most one.
    fn fingers(&mut self, hand: usize, side: Side, map: &[[&'static str; 3]; 5]) {
        // "thumb|pollex", "index|fore", "middle", "ring", "little|pinkie|pinky"
        let fingers = [
            Any(&["thumb", "pollex"]),
            Any(&["index", "fore"]),
            Word("middle"),
            Word("ring"),
            Any(&["little", "pinkie", "pinky"]),
        ];
        let mut tips_index: i64 = -1;
        let mut thumb_tips_size = false;
        let mut named_finger_is_found = false;
        for i in 0..5 {
            let Some(mut finger) = self.search(&[fingers[i]], side, Some(hand), None, Some(0)) else {
                continue;
            };
            // From the tip up to, not including, the hand.
            let mut search_path = Vec::new();
            loop {
                if finger == hand {
                    break;
                }
                search_path.push(finger);
                match self.skeleton.bones()[finger].parent {
                    Some(p) => finger = p,
                    None => break,
                }
            }
            // Tips detection by name matching with "distal" from root.
            for j in (0..search_path.len()).rev() {
                if Word("distal").search(&self.lower[search_path[j]]) {
                    tips_index = j as i64 - 1;
                    break;
                }
            }
            // Tips detection by name matching with "tip|leaf" from end.
            if tips_index < 0
                && let Some(j) = search_path
                    .iter()
                    .position(|&b| Any(&["tip", "leaf"]).search(&self.lower[b]))
            {
                tips_index = j as i64;
            }
            // Tips detection by thumb children size.
            if tips_index < 0 {
                if i == 0 {
                    thumb_tips_size = search_path.len() > 3;
                }
                tips_index = i64::from(thumb_tips_size) - 1;
            }
            if self.assign_finger(search_path, tips_index, &map[i]) {
                named_finger_is_found = true;
            }
        }

        // It is a bit corner case, but possibly the finger names are sequentially numbered...
        if !named_finger_is_found {
            let finger = Word("finger");
            let mut finger_names: Vec<String> = self.children[hand]
                .iter()
                .filter(|&&c| finger.search(&self.lower[c]))
                .map(|&c| self.names[c].clone())
                .collect();
            finger_names.sort(); // Order by lexicographic, normal use cases never have more than 10 fingers in one hand.
            for (i, name) in finger_names.iter().take(5).enumerate() {
                let finger_root = self.find_bone(name).expect("a bone of that name");
                let mut search_path = Vec::new();
                if let Some(mut f) = self.search(&[finger], side, Some(finger_root), None, Some(0)) {
                    loop {
                        if f == finger_root {
                            break;
                        }
                        search_path.push(f);
                        match self.skeleton.bones()[f].parent {
                            Some(p) => f = p,
                            None => break,
                        }
                    }
                }
                search_path.push(finger_root);
                // Tips detection by thumb children size.
                if i == 0 {
                    thumb_tips_size = search_path.len() > 3;
                }
                tips_index = i64::from(thumb_tips_size) - 1;
                self.assign_finger(search_path, tips_index, &map[i]);
            }
        }
    }

    /// Drops the tips (the first `tips_index + 1` bones, from the tip) and sets the finger's bones from the hand out,
    /// the last three at most. Whether it set any.
    fn assign_finger(&mut self, mut search_path: Vec<usize>, tips_index: i64, names: &[&'static str; 3]) -> bool {
        let drop = usize::try_from(tips_index + 1).unwrap_or(0).min(search_path.len());
        search_path.drain(..drop);
        search_path.reverse();
        // Eliminate the possibility of carpal bone.
        let keep = &search_path[search_path.len().saturating_sub(3)..];
        for (name, &b) in names.iter().zip(keep) {
            self.set(name, Some(b));
        }
        !keep.is_empty()
    }
}
