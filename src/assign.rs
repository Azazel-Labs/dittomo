//! Pairing rows with columns for the most total weight (the Hungarian method), leaving a row or column unpaired rather
//! than pairing it at no gain.

/// Room for [`Assign::pairs`] to work in, kept from one pairing to the next: the matching weighs hundreds of them.
#[derive(Default)]
pub(crate) struct Assign {
    w: Vec<f32>,
    u: Vec<f64>,
    v: Vec<f64>,
    p: Vec<usize>,
    way: Vec<usize>,
    minv: Vec<f64>,
    used: Vec<bool>,
}

impl Assign {
    /// The pairs `(row, column)` of `n` rows and `m` columns with the most total weight, each row and column in at
    /// most one, sorted, into `out`, and that total. Only pairs of positive weight are made.
    pub(crate) fn pairs(
        &mut self,
        n: usize,
        m: usize,
        weight: impl Fn(usize, usize) -> f32,
        out: &mut Vec<(usize, usize)>,
    ) -> f32 {
        out.clear();
        if n == 0 || m == 0 {
            return 0.0;
        }
        // Fewer rows than columns, turning the table if need be.
        let flip = n > m;
        let (n, m) = if flip { (m, n) } else { (n, m) };
        self.w.clear();
        for i in 0..n {
            self.w
                .extend((0..m).map(|j| if flip { weight(j, i) } else { weight(i, j) }));
        }
        let total = self.solve(n, m, out);
        if flip {
            for p in out.iter_mut() {
                *p = (p.1, p.0);
            }
        }
        total
    }

    fn solve(&mut self, n: usize, m: usize, out: &mut Vec<(usize, usize)>) -> f32 {
        let w = &self.w;
        if n == 1 {
            let mut best = (0.0, None);
            for (j, &x) in w.iter().enumerate() {
                if x > best.0 {
                    best = (x, Some((0, j)));
                }
            }
            out.extend(best.1);
            return best.0;
        }
        // Rows n ≤ columns m, 1-based with a sentinel column 0; minimizing cost, the negated weight.
        let cost = |i: usize, j: usize| -f64::from(w[(i - 1) * m + j - 1].max(0.0));
        let (u, v, p, way) = (&mut self.u, &mut self.v, &mut self.p, &mut self.way);
        let (minv, used) = (&mut self.minv, &mut self.used);
        u.clear();
        u.resize(n + 1, 0.0);
        v.clear();
        v.resize(m + 1, 0.0);
        p.clear();
        p.resize(m + 1, 0);
        way.clear();
        way.resize(m + 1, 0);
        for i in 1..=n {
            p[0] = i;
            let mut j0 = 0;
            minv.clear();
            minv.resize(m + 1, f64::INFINITY);
            used.clear();
            used.resize(m + 1, false);
            loop {
                used[j0] = true;
                let i0 = p[j0];
                let (mut delta, mut j1) = (f64::INFINITY, 0);
                for j in 1..=m {
                    if !used[j] {
                        let cur = cost(i0, j) - u[i0] - v[j];
                        if cur < minv[j] {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if minv[j] < delta {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for j in 0..=m {
                    if used[j] {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
                if p[j0] == 0 {
                    break;
                }
            }
            loop {
                let j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
                if j0 == 0 {
                    break;
                }
            }
        }
        let mut total = 0.0;
        for j in 1..=m {
            if p[j] != 0 && w[(p[j] - 1) * m + j - 1] > 0.0 {
                total += w[(p[j] - 1) * m + j - 1];
                out.push((p[j] - 1, j - 1));
            }
        }
        out.sort_unstable();
        total
    }
}

#[cfg(test)]
mod tests {
    use super::Assign;

    fn assign(w: &[Vec<f32>]) -> (f32, Vec<(usize, usize)>) {
        let mut out = Vec::new();
        let total = Assign::default().pairs(w.len(), w.first().map_or(0, Vec::len), |i, j| w[i][j], &mut out);
        (total, out)
    }

    #[test]
    fn the_heaviest_pairing_wins_over_the_greedy_one() {
        // Greedy takes 9 then 1; the best is 8 + 7.
        let (total, pairs) = assign(&[vec![9.0, 8.0], vec![7.0, 1.0]]);
        assert_eq!(pairs, vec![(0, 1), (1, 0)]);
        assert!((total - 15.0).abs() < 1e-6);
    }

    #[test]
    fn rows_beyond_the_columns_and_worthless_pairs_go_unpaired() {
        let (total, pairs) = assign(&[vec![1.0], vec![5.0], vec![-2.0]]);
        assert_eq!(pairs, vec![(1, 0)]);
        assert!((total - 5.0).abs() < 1e-6);
        let (_, pairs) = assign(&[vec![-1.0, 0.0], vec![-3.0, -4.0]]);
        assert!(pairs.is_empty());
    }
}
