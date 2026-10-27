use crate::quantify::tracker::CTEntry;
use std::ops::{Index, IndexMut};
use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

/// Snapshots/workers share immutable collision data until their first write.
/// Slice access keeps the contiguous layout required by SIMD row scans.
#[derive(Debug, Clone)]
pub struct PairData(Arc<Vec<CTEntry>>);
impl Deref for PairData {
    type Target = [CTEntry];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for PairData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        Arc::make_mut(&mut self.0).as_mut_slice()
    }
}
impl<'a> IntoIterator for &'a PairData {
    type Item = &'a CTEntry;
    type IntoIter = std::slice::Iter<'a, CTEntry>;
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

// triangular matrix of pair-wise collision loss and weights
// supporting data structure for the `CollisionTracker`
#[derive(Debug, Clone)]
pub struct PairMatrix {
    pub size: usize,
    pub data: PairData,
}

impl PairMatrix {
    pub fn new(size: usize) -> Self {
        let next = size.checked_add(1).expect(crate::dmsg!("pair matrix size overflow"));
        let (a, b) = if size.is_multiple_of(2) {
            (size / 2, next)
        } else {
            (size, next / 2)
        };
        let len = a.checked_mul(b).expect(crate::dmsg!("pair matrix size overflow"));
        Self {
            size,
            data: PairData(Arc::new(vec![
                CTEntry {
                    weight: 1.0,
                    loss: 0.0
                };
                len
            ])),
        }
    }
}

impl Index<(usize, usize)> for PairMatrix {
    type Output = CTEntry;

    fn index(&self, (row, col): (usize, usize)) -> &Self::Output {
        &self.data[calc_idx(row, col, self.size)]
    }
}

impl IndexMut<(usize, usize)> for PairMatrix {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut Self::Output {
        &mut self.data[calc_idx(row, col, self.size)]
    }
}

fn calc_idx(row: usize, col: usize, size: usize) -> usize {
    /* Example:
        0 1 2 3
          4 5 6
            7 8
              9
    */
    debug_assert!(row < size && col < size);
    if row <= col {
        (row * size) + col - ((row * (row + 1)) / 2)
    } else {
        (col * size) + row - ((col * (col + 1)) / 2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shared_snapshots_detach_before_writes_and_preserve_symmetry() {
        let mut source = PairMatrix::new(4);
        source[(1, 3)].loss = 7.;
        let mut copy = source.clone();
        assert!(Arc::ptr_eq(&source.data.0, &copy.data.0));
        copy[(3, 1)].loss = 9.;
        assert_eq!(source[(1, 3)].loss, 7.);
        assert_eq!(copy[(1, 3)].loss, 9.);
        assert!(!Arc::ptr_eq(&source.data.0, &copy.data.0));
    }
    #[test]
    #[should_panic(expected = "pair matrix size overflow")]
    fn rejects_unrepresentable_size_before_allocation() {
        PairMatrix::new(usize::MAX);
    }
}
