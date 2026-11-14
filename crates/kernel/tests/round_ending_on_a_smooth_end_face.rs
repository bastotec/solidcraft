//! A round along an edge whose end face runs on smoothly past the round's contact: a 2 mm flat
//! end face continuing tangentially onto an R20 arc, the edge beside it rounded R6. Where the
//! arc leans outward, the round's rail on the rounded face stops short of the flat's edge, so
//! its end is solved on the face's edge beyond (the arc's), and the end rib is left to the
//! global merge, which cuts the round across the flat and the arc. Before, the rail end was
//! left 0.404 mm inside the face (a slit that splits nothing) and the round failed (SMLib
//! 1001). Where the arc leans inward the old end serves and is kept. Found porting the Mini
//! VMC Motor Mount (Fillet 5).

use solidcore_record::profile::{Loop2, Region2, Seg2};
use solidcore_record::{Plane, Vec2, Vec3};
use solidcraft_kernel::*;

/// A prism 19 deep (along +Y) over a 60 x 60 profile whose left side runs from a 2 mm flat
/// onto an R20 arc turned by 30 degrees: outward (`outward`) or inward.
fn prism(outward: bool) -> Body {
    let s = if outward { 1.0 } else { -1.0 };
    // exactly where the arc ends (20 - 10 sqrt 3): a rounded point leaves the flat and the arc
    // meeting at a slight kink, and the inward round then takes the flat alone.
    let top = Vec2::new(-s * (20.0 - 10.0 * 3f64.sqrt()), 12.0);
    let pts = [Vec2::new(0.0, 2.0), Vec2::new(0.0, 0.0), Vec2::new(60.0, 0.0), Vec2::new(60.0, 60.0), Vec2::new(top.x, 60.0), top];
    let mut segs: Vec<Seg2> = pts.windows(2).map(|w| Seg2::Line { a: w[0], b: w[1] }).collect();
    let sixth = std::f64::consts::FRAC_PI_6;
    segs.push(if outward {
        Seg2::Arc { center: Vec2::new(-20.0, 2.0), radius: 20.0, start: sixth, sweep: -sixth }
    } else {
        Seg2::Arc { center: Vec2::new(20.0, 2.0), radius: 20.0, start: 5.0 * sixth, sweep: sixth }
    });
    let plane = Plane { origin: Vec3::ZERO, x: Vec3::new(-1.0, 0.0, 0.0), y: Vec3::Z };
    let region = Region2 { outer: Loop2 { segs }, holes: vec![] };
    extrude(&plane, &[region], 0.0, 19.0).unwrap().remove(0)
}

/// The edge along the bottom of the y = 19 face rounded R6: a valid closed body, the round
/// taking about (1 - pi/4) r^2 per unit of its 60 mm length (the round's end over the end
/// face adds under 1%).
fn assert_round_builds(outward: bool) {
    let body = prism(outward);
    let before = measure_mass(&body).unwrap().volume;
    let round = fillet(&body, &[Vec3::new(-30.0, 19.0, 0.0)], 6.0).unwrap_or_else(|e| panic!("outward {outward}: the R6 round builds: {e}"));
    assert!(round.validity().is_empty(), "outward {outward}: {:?}", round.validity());
    let after = measure_mass(&round).unwrap().volume;
    let expected = (1.0 - std::f64::consts::FRAC_PI_4) * 36.0 * 60.0;
    let removed = before - after;
    assert!((removed - expected).abs() <= 0.01 * expected, "outward {outward}: removed {removed} vs {expected}");
}

#[test]
fn a_round_ending_on_a_flat_that_runs_onto_an_outward_arc_builds() {
    assert_round_builds(true);
}

#[test]
fn a_round_ending_on_a_flat_that_runs_onto_an_inward_arc_builds() {
    assert_round_builds(false);
}
