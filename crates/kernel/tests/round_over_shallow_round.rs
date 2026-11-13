//! A round chained along a face outline that runs over a shallow round (a corner of a profile
//! turned by under ~17 degrees, rounded R6): the round's faces over the shallow round are exact
//! rounds whose cylinder support's label does not stand on its form to the rounding the reader
//! derives (#3185), so their definitions are not written and the record reads back. Before, the
//! definition was written, and the body built could not be measured or operated on (its record
//! was refused, SM_ERR_INVALID_INPUT). Found porting the Mini VMC X Axis Motor Mount (a conic
//! flank met by a vertical side, rounded R6, then the R1 outline round of the face).

use solidcore_record::profile::{Loop2, Region2};
use solidcore_record::{Plane, Vec2, Vec3};
use solidcraft_kernel::*;

/// A prism 19 thick over a hexagon whose two lower corners turn by atan(slant / 50).
fn hexagon(slant: f64) -> Body {
    let p = [(0.0, 65.0), (80.0, 65.0), (80.0, 20.0), (80.0 - slant, -30.0), (slant, -30.0), (0.0, 20.0)];
    let pts: Vec<Vec2> = p.iter().map(|&(x, y)| Vec2::new(x, y)).collect();
    let r = Region2 { outer: Loop2::polygon(&pts), holes: vec![] };
    extrude(&Plane::XY, &[r], 0.0, 19.0).unwrap().remove(0)
}

/// The middle of the R `r` round at corner `v` between the sides towards `a` and `c`, on the top.
fn round_middle(v: [f64; 2], a: [f64; 2], c: [f64; 2], r: f64) -> Vec3 {
    let unit = |p: [f64; 2]| {
        let l = p[0].hypot(p[1]);
        [p[0] / l, p[1] / l]
    };
    let d1 = unit([a[0] - v[0], a[1] - v[1]]);
    let d2 = unit([c[0] - v[0], c[1] - v[1]]);
    let bisector = unit([d1[0] + d2[0], d1[1] + d2[1]]);
    let half = (d1[0] * d2[0] + d1[1] * d2[1]).clamp(-1.0, 1.0).acos() / 2.0;
    let t = r / half.sin() - r;
    Vec3::new(v[0] + t * bisector[0], v[1] + t * bisector[1], 19.0)
}

/// The prism with its four vertical corners rounded R6 and its top outline (the nine edges from
/// the top side round to the two slanted sides) rounded R1.
fn rounded(slant: f64) -> Body {
    let corners = [Vec3::new(0.0, 65.0, 9.5), Vec3::new(80.0, 65.0, 9.5), Vec3::new(80.0, 20.0, 9.5), Vec3::new(0.0, 20.0, 9.5)];
    let six = fillet(&hexagon(slant), &corners, 6.0).unwrap();
    let k = std::f64::consts::FRAC_1_SQRT_2;
    let outline = [
        Vec3::new(40.0, 65.0, 19.0),
        Vec3::new(6.0 - 6.0 * k, 59.0 + 6.0 * k, 19.0),
        Vec3::new(74.0 + 6.0 * k, 59.0 + 6.0 * k, 19.0),
        Vec3::new(0.0, 40.0, 19.0),
        Vec3::new(80.0, 40.0, 19.0),
        round_middle([0.0, 20.0], [0.0, 65.0], [slant, -30.0], 6.0),
        round_middle([80.0, 20.0], [80.0, 65.0], [80.0 - slant, -30.0], 6.0),
        Vec3::new(slant / 2.0, -5.0, 19.0),
        Vec3::new(80.0 - slant / 2.0, -5.0, 19.0),
    ];
    fillet(&six, &outline, 1.0).unwrap()
}

/// Rounded over a corner turned by 9.1 degrees (and, as before, by 17.7): the body measures, its
/// record reads back into the kernel, and a cut through it builds.
#[test]
fn a_round_over_a_shallow_round_reads_back_and_operates() {
    for slant in [8.0, 16.0] {
        let body = rounded(slant);
        let m = measure_mass(&body).unwrap_or_else(|e| panic!("slant {slant}: the rounded body measures: {e}"));
        assert!(m.volume > 0.0 && m.volume.is_finite(), "slant {slant}: volume {}", m.volume);
        smlib_body(&body).unwrap_or_else(|e| panic!("slant {slant}: its record reads back: {e}"));
        let reopened = decode_native_body(&encode_native_body(&body).unwrap()).unwrap();
        let again = measure_mass(&reopened).unwrap();
        assert!((again.volume - m.volume).abs() <= 1e-9 * m.volume, "slant {slant}: saved and reopened, the same volume");
        let bore = cylinder(Vec3::new(40.0, 40.0, -1.0), Vec3::Z, 5.0, 21.0).unwrap();
        let cut = boolean(&body, &bore, BoolOp::Cut).unwrap().unwrap();
        let after = measure_mass(&cut).unwrap();
        let expected = m.volume - std::f64::consts::PI * 25.0 * 19.0;
        assert!((after.volume - expected).abs() <= 1e-6 * expected, "slant {slant}: a bore cut through it, {} vs {expected}", after.volume);
    }
}

/// The faces of `body`'s record that keep an exact round's definition, and those written as a
/// bare NURBS net.
fn written(body: &Body) -> (usize, usize) {
    use solidcore_record::brep::{Surface, blend::BlendDefinition};
    let rec = brep_record(body).unwrap();
    let mut exact = 0;
    let mut net = 0;
    for f in &rec.faces {
        match &f.surface {
            Surface::Blend { definition } if matches!(**definition, BlendDefinition::ExactRound(_)) => exact += 1,
            Surface::Spline { .. } | Surface::RationalSpline { .. } => net += 1,
            _ => {}
        }
    }
    (exact, net)
}

/// What the writer keeps: over the corner turned by 9.1 degrees one of the four exact rounds
/// does not rebuild from its record, so it is written as its net and the other three keep their
/// definitions; over the corner turned by 17.7 all four rebuild and all four keep them. Before,
/// all four were written at 9.1 degrees and the record did not read back.
#[test]
fn only_the_round_that_does_not_rebuild_is_written_as_its_net() {
    let shallow = rounded(8.0);
    assert_eq!(written(&shallow), (3, 1), "slant 8: three definitions kept, one face as its net");
    smlib_body(&shallow).unwrap();
    let steeper = rounded(16.0);
    assert_eq!(written(&steeper), (4, 0), "slant 16: every definition kept");
    smlib_body(&steeper).unwrap();
}
