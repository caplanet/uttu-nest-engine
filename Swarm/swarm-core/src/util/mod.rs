pub mod assertions;
pub mod frozen;

pub mod bit_reversal_iterator;
pub mod io;
pub mod parts;
pub mod listener;
pub mod simplify_mm;
pub mod terminator;

#[cfg(not(target_arch = "wasm32"))]
pub mod ctrlc_terminator;

/// Collision-engine key of a placed item (jagua-rs 1.x looks hazards up by entity).
pub fn haz_key(layout: &jagua_rs::entities::Layout, pk: jagua_rs::entities::PItemKey) -> Option<jagua_rs::collision_detection::hazards::HazKey> {
    let pi = layout.placed_items().get(pk)?;
    layout.cde().haz_key(&jagua_rs::collision_detection::hazards::BasicHazardEntity::from((pk, pi)))
}
