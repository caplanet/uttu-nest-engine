//! Source-compatible no-ops for the removed hole-fill operators.
use crate::config::{HoleFillConfig, NfpIntegrationConfig};
use crate::optimizer::separator::Separator;
use crate::util::terminator::Terminator;
#[derive(Default)]
pub struct NfpHoleCache;
impl NfpHoleCache {
    pub fn new() -> Self {
        Self
    }
    pub fn feasible_region_calls(&self) -> usize {
        0
    }
}
pub fn hole_fill_pass(
    _sep: &mut Separator,
    _cfg: &HoleFillConfig,
    _term: &impl Terminator,
) -> usize {
    0
}
pub fn hole_fill_pass_with_nfp(
    _sep: &mut Separator,
    _cfg: &HoleFillConfig,
    _nfp: &NfpIntegrationConfig,
    _cache: Option<&mut NfpHoleCache>,
    _term: &impl Terminator,
) -> usize {
    0
}
