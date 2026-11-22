//! Pairing rows with columns for the most total weight (the Hungarian method), leaving a row or column unpaired rather
//! than pairing it at no gain.

/// The pairs `(row, column)` with the most total weight, each row and column in at most one, and that total. Only
/// pairs of positive weight are made.
pub(crate) fn assign(w: &[Vec<f32>]) -> (f32, Vec<(usize, usize)>) {
    let n = w.len();
    let m = w.first().map_or(0, Vec::len);
    if n == 0 || m == 0 {
        return (0.0, Vec::new());
    }
    if n > m {
        let t: Vec<Vec<f32>> = (0..m).map(|j| (0..n).map(|i| w[i][j]).collect()).collect();
        let (total, pairs) = assign(&t);
        return (total, pairs.into_iter().map(|(j, i)| (i, j)).collect());
    }
    if n == 1 {
        let mut best = (0.0, Vec::new());
        for (j, &x) in w[0].iter().enumerate() {
            if x > best.0 {
                best = (x, vec![(0, j)]);
            }
        }
        return best;
    }
    // Rows n ≤ columns m, 1-based with a sentinel column 0; minimising cost, the negated weight.
    let cost = |i: usize, j: usize| -f64::from(w[i - 1][j - 1].max(0.0));
    let (mut u, mut v) = (vec![0.0f64; n + 1], vec![0.0f64; m + 1]);
    let (mut p, mut way) = (vec![0usize; m + 1], vec![0usize; m + 1]);
    for i in 1..=n {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![f64::INFINITY; m + 1];
        let mut used = vec![false; m + 1];
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
    let mut pairs = Vec::new();
    for j in 1..=m {
        if p[j] != 0 && w[p[j] - 1][j - 1] > 0.0 {
            total += w[p[j] - 1][j - 1];
            pairs.push((p[j] - 1, j - 1));
        }
    }
    pairs.sort_unstable();
    (total, pairs)
}

#[cfg(test)]
mod tests {
    use super::assign;

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
