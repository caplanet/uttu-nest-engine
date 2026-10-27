//! Experimental structure-first nesting primitives.
//!
//! These modules are deliberately independent from the public `PlaceGroup`
//! contract. A [`motif::PlacementMotif`] is a reversible construction hint,
//! not a user-enforced rigid constraint.

mod certification;
pub mod config;
pub mod evaluator;
pub mod motif;
pub mod pipeline;
pub mod profile;
