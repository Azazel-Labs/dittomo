//! Which target bone plays which source bone.
//!
//! Every pair of bones gets a likeness from their names and shapes, and the two trees are matched as wholes for the
//! most total likeness, keeping the family: a bone below another plays a bone below the other's, and bones in separate
//! limbs play bones in separate limbs. A bone on either side can be passed over, its children taking its place, so
//! extra bones don't throw the match off; passed over alone among its siblings, its children stand beside them, as a
//! pelvis holding both legs does against hips that hold the legs and the spine together. (This is the constrained tree
//! mapping of Zhang, 1996, which pairs up subtrees with the Hungarian method and so stays fast whatever the order of
//! children.)

use glam::Quat;

use crate::assign::assign;
use crate::contact::{self, Fixes};
use crate::names::{self, Lexicon};
use crate::shape::Shape;
use crate::surface::Surface;
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
            let own = likeness(&t, b, &t, b, names::likeness(&t.names[b], &t.names[b])).max(1e-6);
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
        transfer::play(self, source, target, clip)
    }

    /// The clip played by the target, then fixed up: travel given to a clip walked in place, feet kept still while
    /// planted, arms kept out of the body and head, hands brought together where the source's meet and kept on the
    /// head where the source's are on theirs. [`Fixes::ALL`] makes every fix; [`Fixes::NONE`] is [`Mapping::apply`].
    ///
    /// # Errors
    /// If the clip doesn't pose the source, or the mapping was made for skeletons of other sizes.
    pub fn apply_with(&self, source: &Skeleton, target: &Skeleton, clip: &Clip, fixes: Fixes) -> Result<Clip, Error> {
        self.apply_touching(source, target, [&Surface::default(), &Surface::default()], clip, fixes)
    }

    /// [`Mapping::apply_with`], knowing each rig's skin (source's first): a hand as near its skin as to touch it keeps
    /// the same gap to the target's, scaled by height, so it rests on a hip or a coat rather than floating off it or
    /// sinking in. An empty [`Surface`] leaves that fix out.
    ///
    /// # Errors
    /// If the clip doesn't pose the source, or the mapping was made for skeletons of other sizes.
    pub fn apply_touching(
        &self,
        source: &Skeleton,
        target: &Skeleton,
        skins: [&Surface; 2],
        clip: &Clip,
        fixes: Fixes,
    ) -> Result<Clip, Error> {
        if source.len() != self.sources || target.len() != self.pairs.len() {
            return Err(Error::NotTheirMapping);
        }
        clip.check(source.len())?;
        let (s, t) = (Shape::light(source), Shape::light(target));
        let traveled = if fixes.travel {
            contact::traveled(source, &s, clip)
        } else {
            None
        };
        let clip = traveled.as_ref().unwrap_or(clip);
        let mut player = transfer::Player::of_shapes(self, source, target, &s, &t);
        let mut out = transfer::play_on(&mut player, target, clip)?;
        if fixes.feet {
            contact::plant(self, [(source, &s), (target, &t)], clip, &mut out);
        }
        // Each body (its arms, spine and head), found once for every fix that needs it.
        let both = fixes.touch || fixes.hands || fixes.head;
        let mut t_body = (fixes.push_out || both).then(|| contact::body(target, &t)).flatten();
        let mut s_body = both.then(|| contact::body(source, &s)).flatten();
        // With skins, each head is as big as its skin shows, not as its bones suggest.
        for (body, shape, skin) in [(&mut s_body, &s, skins[0]), (&mut t_body, &t, skins[1])] {
            if let Some(b) = body {
                b.fit_head(shape, skin);
            }
        }
        if let (true, Some(tb)) = (fixes.push_out, &t_body) {
            contact::push_out(target, tb, &mut out);
        }
        if let (Some(sb), Some(tb)) = (&s_body, &t_body) {
            let (rigs, bodies) = ([(source, &s), (target, &t)], [sb, tb]);
            if fixes.hands {
                contact::hands(self, rigs, bodies, clip, &mut out);
            }
            if fixes.head {
                contact::on_head(self, rigs, bodies, clip, &mut out);
            }
            // Last, as it sees the skin: the hands the others placed are brought to the gap the source's have.
            if fixes.touch {
                contact::touch(
                    self,
                    [(source, &s, skins[0]), (target, &t, skins[1])],
                    bodies,
                    clip,
                    &mut out,
                );
            }
        }
        Ok(out)
    }

    /// How many bones the source has.
    pub(crate) fn sources(&self) -> usize {
        self.sources
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

/// How alike source bone `a` and target bone `b` are, given how alike their names are: above zero when more alike than
/// not.
fn likeness(s: &Shape, a: usize, t: &Shape, b: usize, named: Option<f32>) -> f32 {
    let mut v = 0.0;
    let (na, nb) = (&s.names[a], &t.names[b]);
    if let Some(n) = named {
        v += 2.0 * n - 0.7;
    }
    if names::opposed(na, nb) {
        v -= 3.0;
    }
    // A twist bone sits along a limb near a joint, so where it stands says it could be one: only its name says not.
    if na.helper() != nb.helper() {
        v -= 0.6;
    }
    // A bone marking where a chain ends lies past the last joint, where a rig without one has its last joint.
    if na.end() != nb.end() {
        v -= 0.3;
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
    // Rigs stop at different depths, at the wrist or the fingertips, the ankle or the toes' ends: a bone with nothing
    // beyond it says nothing of how much body would be.
    v += 0.4
        * if s.children[a].is_empty() || t.children[b].is_empty() {
            0.5
        } else {
            bell(s.beyond[a] - t.beyond[b], 0.1)
        };
    let (la, lb) = (s.length[a], t.length[b]);
    v += 0.4
        * match (la > 1e-4, lb > 1e-4) {
            (true, true) => bell((la / lb).ln(), 0.4),
            _ => 0.5,
        };
    let near = 0.08 + 0.35 * s.reach[a].min(t.reach[b]);
    v += 0.8 * bell(s.at[a].distance(t.at[b]), near);
    v += 0.4 * bell(s.reach[a] - t.reach[b], 0.1);
    // Whatever the proportions, the arms hang from the chest and the legs from the hips.
    let role = |sh: &Shape, x: usize| sh.girdles.iter().position(|&g| g == Some(x));
    match (role(s, a), role(t, b)) {
        (Some(x), Some(y)) => v += if x == y { 0.6 } else { -0.6 },
        (Some(_), None) | (None, Some(_)) if s.girdles[0].is_some() && t.girdles[0].is_some() => v -= 0.6,
        _ => {}
    }
    // Limbs branch where they branch, however high: a bone two limbs hang from plays one, not a bone along a chain.
    // A bone with nothing beyond it might be either, on a rig that stops there.
    if s.forks[a] && t.forks[b] {
        v += 0.3;
    } else if s.forks[a] != t.forks[b] && !s.children[a].is_empty() && !t.children[b].is_empty() {
        v -= 0.5;
    }
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
    /// Children's trees paired up: these pairs of all the match's.
    Paired(std::ops::Range<usize>),
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
    let words = Lexicon::new(&s.names, &t.names);
    for a in 1..na {
        for b in 1..nb {
            let named = words.likeness(a - 1, &s.names[a - 1], b - 1, &t.names[b - 1]);
            like[at(a, b)] = likeness(s, a - 1, t, b - 1, named);
        }
    }
    let mut tree = vec![0.0f32; na * nb];
    let mut tree_how = vec![Tree::None; na * nb];
    let mut forest = vec![0.0f32; na * nb];
    let mut forest_how = vec![Forest::None; na * nb];
    // The pairs of every forest's best pairing, one after another, and the pairings being weighed.
    let (mut paired, mut won, mut tried) = (Vec::new(), Vec::new(), Vec::new());
    let mut spliced = Vec::new();
    // Children come after their parents (the stand-in root is 0), so going backwards meets them first.
    for a in (0..na).rev() {
        for b in (0..nb).rev() {
            // Forests: the children of a against the children of b.
            let mut best = (0.0f32, Forest::None);
            let worth = |i: usize, j: usize| tree[at(i, j)];
            if !ka[a].is_empty() && !kb[b].is_empty() {
                let total = pairing(&ka[a], &kb[b], worth, &mut tried);
                if total > best.0 {
                    best = (total, Forest::Paired(0..0));
                    std::mem::swap(&mut won, &mut tried);
                }
                // A child passed over with its siblings kept, its own children standing beside them: a pelvis
                // between the hips and both legs, where the other rig hangs the legs and the spine off the hips
                // alike. Passing the pelvis over alone would keep one leg at most. A child with one child of its own
                // needs no splice: passing it over within its pair does as much.
                if ka[a].len() > 1 {
                    for &i in ka[a].iter().filter(|&&i| ka[i].len() > 1) {
                        spliced.clear();
                        spliced.extend(ka[a].iter().copied().filter(|&x| x != i).chain(ka[i].iter().copied()));
                        let total = pairing(&spliced, &kb[b], worth, &mut tried);
                        if total > best.0 {
                            best = (total, Forest::Paired(0..0));
                            std::mem::swap(&mut won, &mut tried);
                        }
                    }
                }
                if kb[b].len() > 1 {
                    for &j in kb[b].iter().filter(|&&j| kb[j].len() > 1) {
                        spliced.clear();
                        spliced.extend(kb[b].iter().copied().filter(|&x| x != j).chain(kb[j].iter().copied()));
                        let total = pairing(&ka[a], &spliced, worth, &mut tried);
                        if total > best.0 {
                            best = (total, Forest::Paired(0..0));
                            std::mem::swap(&mut won, &mut tried);
                        }
                    }
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
            if let Forest::Paired(range) = &mut best.1 {
                let start = paired.len();
                paired.extend_from_slice(&won);
                *range = start..paired.len();
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
                Forest::Paired(range) => todo.extend(paired[range.clone()].iter().map(|&(i, j)| (true, i, j))),
                Forest::SourceChild(i) => todo.push((false, *i, b)),
                Forest::TargetChild(j) => todo.push((false, a, *j)),
            }
        }
    }
    fill(&ka, &kb, &like, &mut out);
    out.sort_unstable_by_key(|p| p.1);
    out
}

/// The pairing of the trees of `rows` with those of `cols` worth most, as pairs of them, into `out`, and its worth.
fn pairing(rows: &[usize], cols: &[usize], worth: impl Fn(usize, usize) -> f32, out: &mut Vec<(usize, usize)>) -> f32 {
    out.clear();
    if rows.len() == 1 || cols.len() == 1 {
        // One tree on a side pairs with the worthiest on the other, the first of the worthiest, as [`assign`] would.
        let mut best = (0.0, None);
        for &i in rows {
            for &j in cols {
                let w = worth(i, j);
                if w > best.0 {
                    best = (w, Some((i, j)));
                }
            }
        }
        out.extend(best.1);
        return best.0;
    }
    let w: Vec<Vec<f32>> = rows
        .iter()
        .map(|&i| cols.iter().map(|&j| worth(i, j)).collect())
        .collect();
    let (total, pairs) = assign(&w);
    out.extend(pairs.into_iter().map(|(i, j)| (rows[i], cols[j])));
    total
}

/// Pairs bones the match left out where the family allows: a bone plays one whose nearest paired ancestors play each
/// other, and whose paired bones below play bones below it. The match keeps disjoint limbs disjoint, so it can't put
/// two source limbs into one target limb (a hand's five fingers into a palm bone holding index and thumb); this pass
/// can, best likeness first. Then it pairs, though less alike than not, bones held in place by pairs both above and
/// below them: the match leaves out a bone whose likeness is low, as an odd rig's chest or upper arm, even where the
/// pairs around it leave it nothing else to play.
fn fill(ka: &[Vec<usize>], kb: &[Vec<usize>], like: &[f32], out: &mut Vec<(usize, usize, f32)>) {
    let (na, nb) = (ka.len(), kb.len());
    let parents = |k: &[Vec<usize>]| {
        let mut p = vec![usize::MAX; k.len()];
        for (x, cs) in k.iter().enumerate() {
            for &c in cs {
                p[c] = x;
            }
        }
        p
    };
    let (pa, pb) = (parents(ka), parents(kb));
    let mut to = vec![None; na];
    let mut from = vec![None; nb];
    to[0] = Some(0);
    from[0] = Some(0);
    for &(a, b, _) in out.iter() {
        to[a + 1] = Some(b + 1);
        from[b + 1] = Some(a + 1);
    }
    // Held in place, a bone may play one less alike than not, but not one on the other side or named for another.
    let floor = -1.0;
    let mut todo = Vec::new();
    for a in (1..na).filter(|&a| to[a].is_none()) {
        todo.extend(
            (1..nb)
                .filter(|&b| from[b].is_none() && like[a * nb + b] > floor)
                .map(|b| (a, b)),
        );
    }
    if todo.is_empty() {
        return;
    }
    todo.sort_by(|x, y| like[y.0 * nb + y.1].total_cmp(&like[x.0 * nb + x.1]));
    // The nearest paired bone above, the stand-in root at the latest.
    let anchor = |p: &[usize], paired: &dyn Fn(usize) -> bool, mut x: usize| loop {
        x = p[x];
        if paired(x) {
            return x;
        }
    };
    let below = |p: &[usize], mut x: usize, top: usize| loop {
        if x == top {
            return true;
        }
        if x == 0 {
            return false;
        }
        x = p[x];
    };
    // Whether every paired bone under `top` has its partner under `other`.
    fn under_all(k: &[Vec<usize>], top: usize, partner: &[Option<usize>], ok: &dyn Fn(usize) -> bool) -> bool {
        let mut stack = k[top].clone();
        while let Some(x) = stack.pop() {
            if partner[x].is_some_and(|y| !ok(y)) {
                return false;
            }
            stack.extend_from_slice(&k[x]);
        }
        true
    }
    // Whether any bone under `top` is paired.
    fn holds(k: &[Vec<usize>], top: usize, partner: &[Option<usize>]) -> bool {
        let mut stack = k[top].clone();
        while let Some(x) = stack.pop() {
            if partner[x].is_some() {
                return true;
            }
            stack.extend_from_slice(&k[x]);
        }
        false
    }
    // Each the only child of bones that play each other, as hands at the ends of forearms: nothing else could play
    // either, however unlike their rest poses leave them.
    let only = |k: &[Vec<usize>], p: &[usize], x: usize| p[x] != 0 && k[p[x]].len() == 1;
    let carried =
        |to: &[Option<usize>], a: usize, b: usize| only(ka, &pa, a) && only(kb, &pb, b) && to[pa[a]] == Some(pb[b]);
    // First the bones more alike than not; then, less alike, bones held in place by pairs above and below both, or
    // carrying on a pair's chain alone.
    for sandwiched in [false, true] {
        loop {
            let mut added = false;
            for &(a, b) in &todo {
                let l = like[a * nb + b];
                if to[a].is_some() || from[b].is_some() || (l <= 0.0 && !sandwiched) {
                    continue;
                }
                let up_a = anchor(&pa, &|x| to[x].is_some(), a);
                let up_b = anchor(&pb, &|x| from[x].is_some(), b);
                if to[up_a] != Some(up_b)
                    || !under_all(ka, a, &to, &|y| below(&pb, y, b))
                    || !under_all(kb, b, &from, &|x| below(&pa, x, a))
                    || (l <= 0.0 && !(holds(ka, a, &to) && holds(kb, b, &from)) && !carried(&to, a, b))
                {
                    continue;
                }
                to[a] = Some(b);
                from[b] = Some(a);
                out.push((a - 1, b - 1, l.max(0.0)));
                added = true;
            }
            if !added {
                break;
            }
        }
    }
}
