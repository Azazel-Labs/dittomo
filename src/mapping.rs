//! Which target bone plays which source bone.
//!
//! Every pair of bones gets a likeness from their names and shapes, and the two trees are matched as wholes for the
//! most total likeness, keeping the family: a bone below another plays a bone below the other's, and bones in separate
//! limbs play bones in separate limbs. A bone on either side can be passed over, its children taking its place, so
//! extra bones don't throw the match off. (This is the constrained tree mapping of Zhang, 1996, which pairs up subtrees
//! with the Hungarian method and so stays fast whatever the order of children.)

use glam::Quat;

use crate::assign::assign;
use crate::names;
use crate::shape::Shape;
use crate::{Clip, Error, Skeleton, transfer};

/// A target bone's source bone, and how alike the two were found, from 0 to 1 and over for a likeness better than the
/// target bone's with itself.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pair {
    pub source: usize,
    pub score: f32,
}

/// Which target bone plays which source bone, and how the two bodies are turned to each other.
#[derive(Clone, Debug, PartialEq)]
pub struct Mapping {
    pairs: Vec<Option<Pair>>,
    sources: usize,
    quality: f32,
    turn: Quat,
}

impl Mapping {
    /// The mapping from `source` to `target` that pairs the most alike bones while keeping the trees' families.
    #[must_use]
    pub fn new(source: &Skeleton, target: &Skeleton) -> Self {
        let s = Shape::of(source);
        let t = Shape::of(target);
        let found = matched(&s, &t);
        let mut pairs = vec![None; target.len()];
        for (a, b, like) in found {
            let own = likeness(&t, b, &t, b).max(1e-6);
            pairs[b] = Some(Pair {
                source: a,
                score: like / own,
            });
        }
        let mut mapping = Self {
            pairs,
            sources: source.len(),
            quality: 0.0,
            turn: t.frame * s.frame.inverse(),
        };
        mapping.quality = mapping.weigh(&t);
        mapping
    }

    /// The source bone this target bone plays, if any.
    #[must_use]
    pub fn source_of(&self, target: usize) -> Option<usize> {
        self.pairs.get(target).copied().flatten().map(|p| p.source)
    }

    /// Each target bone's pair, in the target's order.
    #[must_use]
    pub fn pairs(&self) -> &[Option<Pair>] {
        &self.pairs
    }

    /// How much of the target is played and how alike, from 0 (nothing) to 1 (as by itself), its bones weighed by
    /// their length.
    #[must_use]
    pub fn quality(&self) -> f32 {
        self.quality
    }

    /// The turn from the source body's frame to the target's (about the up axis, for bodies facing different ways).
    #[must_use]
    pub fn turn(&self) -> Quat {
        self.turn
    }

    /// Make a target bone play another source bone, or none. For a bone the match got wrong; a pair that breaks the
    /// family (a bone playing one above what its parent plays) plays, but oddly.
    ///
    /// # Panics
    /// If either bone isn't in its skeleton.
    pub fn set(&mut self, target: usize, source: Option<usize>) {
        assert!(target < self.pairs.len(), "no target bone {target}");
        assert!(source.is_none_or(|s| s < self.sources), "no source bone");
        self.pairs[target] = source.map(|source| Pair { source, score: 1.0 });
    }

    /// The clip played by the target.
    ///
    /// # Errors
    /// If the clip doesn't pose the source, or the mapping was made for skeletons of other sizes.
    pub fn apply(&self, source: &Skeleton, target: &Skeleton, clip: &Clip) -> Result<Clip, Error> {
        if source.len() != self.sources || target.len() != self.pairs.len() {
            return Err(Error::NotTheirMapping);
        }
        clip.check(source.len())?;
        Ok(transfer::play(self, source, target, clip))
    }

    fn weigh(&self, t: &Shape) -> f32 {
        let (mut got, mut all) = (0.0f32, 0.0f32);
        for (b, pair) in self.pairs.iter().enumerate() {
            let len = t.children[b].iter().map(|&c| t.at[c].distance(t.at[b])).sum::<f32>();
            let w = 0.02 + len;
            all += w;
            if let Some(p) = pair {
                got += w * p.score.clamp(0.0, 1.0);
            }
        }
        if all > 0.0 { got / all } else { 0.0 }
    }
}

/// How alike source bone `a` and target bone `b` are: above zero when more alike than not.
pub(crate) fn likeness(s: &Shape, a: usize, t: &Shape, b: usize) -> f32 {
    let mut v = 0.0;
    let (na, nb) = (&s.names[a], &t.names[b]);
    if let Some(n) = names::likeness(na, nb) {
        v += 2.0 * n - 0.7;
    }
    if names::opposed(na, nb) {
        v -= 3.0;
    }
    // On opposite sides of the body.
    let (la, lb) = (s.at[a].x, t.at[b].x);
    if la * lb < 0.0 && la.abs() > 0.03 && lb.abs() > 0.03 {
        v -= 2.0;
    }
    // What holds whatever the proportions: which way it points, from its parent and to its children, how high it
    // stands and how much body lies beyond it. Then, worth less as proportions differ, where it is and how far along
    // the body.
    // A root has no way in and no length: that says little, so it counts half.
    let way = |x: glam::Vec3, y: glam::Vec3| {
        if x == glam::Vec3::ZERO || y == glam::Vec3::ZERO {
            0.5
        } else {
            x.dot(y)
        }
    };
    let bell = |d: f32, w: f32| (-(d / w) * (d / w)).exp();
    v += 0.4 * way(s.dir[a], t.dir[b]);
    v += 0.4 * way(s.inward[a], t.inward[b]);
    v += 0.4 * bell(s.at[a].y - t.at[b].y, 0.1);
    v += 0.4 * bell(s.beyond[a] - t.beyond[b], 0.1);
    let (la, lb) = (s.length[a], t.length[b]);
    v += 0.4
        * match (la > 1e-4, lb > 1e-4) {
            (true, true) => bell((la / lb).ln(), 0.4),
            _ => 0.5,
        };
    let near = 0.08 + 0.35 * s.reach[a].min(t.reach[b]);
    v += 0.8 * bell(s.at[a].distance(t.at[b]), near);
    v += 0.4 * bell(s.reach[a] - t.reach[b], 0.1);
    v -= 1.2;
    v
}

#[derive(Clone, Copy)]
enum Tree {
    None,
    /// The two roots play each other.
    Pair,
    /// The source root is passed over for one of its children.
    SourceChild(usize),
    /// The target root is passed over for one of its children.
    TargetChild(usize),
}

#[derive(Clone)]
enum Forest {
    None,
    /// Children's trees paired up.
    Paired(Vec<(usize, usize)>),
    /// A source child is passed over, its children standing in for the source's.
    SourceChild(usize),
    TargetChild(usize),
}

/// The pairs (source bone, target bone, likeness) of the best match.
fn matched(s: &Shape, t: &Shape) -> Vec<(usize, usize, f32)> {
    // Node 0 on each side stands above the roots, so a skeleton of several trees is one tree.
    let kids = |shape: &Shape| -> Vec<Vec<usize>> {
        let n = shape.children.len();
        let mut k = vec![Vec::new(); n + 1];
        for i in 0..n {
            k[i + 1] = shape.children[i].iter().map(|c| c + 1).collect();
        }
        let is_child: Vec<bool> = {
            let mut v = vec![false; n];
            for c in shape.children.iter().flatten() {
                v[*c] = true;
            }
            v
        };
        k[0] = (0..n).filter(|&i| !is_child[i]).map(|i| i + 1).collect();
        k
    };
    let (ka, kb) = (kids(s), kids(t));
    let (na, nb) = (ka.len(), kb.len());
    let at = |a: usize, b: usize| a * nb + b;
    let mut like = vec![0.0f32; na * nb];
    for a in 1..na {
        for b in 1..nb {
            like[at(a, b)] = likeness(s, a - 1, t, b - 1);
        }
    }
    let mut tree = vec![0.0f32; na * nb];
    let mut tree_how = vec![Tree::None; na * nb];
    let mut forest = vec![0.0f32; na * nb];
    let mut forest_how = vec![Forest::None; na * nb];
    // Children come after their parents (the stand-in root is 0), so going backwards meets them first.
    for a in (0..na).rev() {
        for b in (0..nb).rev() {
            // Forests: the children of a against the children of b.
            let mut best = (0.0f32, Forest::None);
            if !ka[a].is_empty() && !kb[b].is_empty() {
                let w: Vec<Vec<f32>> = ka[a]
                    .iter()
                    .map(|&i| kb[b].iter().map(|&j| tree[at(i, j)]).collect())
                    .collect();
                let (total, pairs) = assign(&w);
                if total > best.0 {
                    best = (
                        total,
                        Forest::Paired(pairs.into_iter().map(|(i, j)| (ka[a][i], kb[b][j])).collect()),
                    );
                }
            }
            for &j in &kb[b] {
                if forest[at(a, j)] > best.0 {
                    best = (forest[at(a, j)], Forest::TargetChild(j));
                }
            }
            for &i in &ka[a] {
                if forest[at(i, b)] > best.0 {
                    best = (forest[at(i, b)], Forest::SourceChild(i));
                }
            }
            forest[at(a, b)] = best.0;
            forest_how[at(a, b)] = best.1;

            // Trees: a's tree against b's.
            let pair = if a == 0 && b == 0 {
                forest[at(a, b)]
            } else if a == 0 || b == 0 {
                f32::NEG_INFINITY
            } else {
                like[at(a, b)] + forest[at(a, b)]
            };
            let mut best = (0.0f32, Tree::None);
            if pair > best.0 || (a == 0 && b == 0) {
                best = (pair.max(0.0), Tree::Pair);
            }
            for &j in &kb[b] {
                if tree[at(a, j)] > best.0 {
                    best = (tree[at(a, j)], Tree::TargetChild(j));
                }
            }
            for &i in &ka[a] {
                if tree[at(i, b)] > best.0 {
                    best = (tree[at(i, b)], Tree::SourceChild(i));
                }
            }
            tree[at(a, b)] = best.0;
            tree_how[at(a, b)] = best.1;
        }
    }

    let mut out = Vec::new();
    let mut todo = vec![(true, 0usize, 0usize)];
    // (is a tree, a, b)
    while let Some((is_tree, a, b)) = todo.pop() {
        if is_tree {
            match tree_how[at(a, b)] {
                Tree::None => {}
                Tree::Pair => {
                    if a != 0 && b != 0 {
                        out.push((a - 1, b - 1, like[at(a, b)]));
                    }
                    todo.push((false, a, b));
                }
                Tree::SourceChild(i) => todo.push((true, i, b)),
                Tree::TargetChild(j) => todo.push((true, a, j)),
            }
        } else {
            match &forest_how[at(a, b)] {
                Forest::None => {}
                Forest::Paired(pairs) => todo.extend(pairs.iter().map(|&(i, j)| (true, i, j))),
                Forest::SourceChild(i) => todo.push((false, *i, b)),
                Forest::TargetChild(j) => todo.push((false, a, *j)),
            }
        }
    }
    out.sort_unstable_by_key(|p| p.1);
    out
}
