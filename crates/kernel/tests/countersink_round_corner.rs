// A countersink overlapping a round band must not split off the round from its smooth corner arcs.
use solidcore_record::profile::{Loop2, Region2};
use solidcore_record::{Plane, Vec2, Vec3};
use solidcraft_kernel::*;
use std::f64::consts::{FRAC_1_SQRT_2 as K, PI};

const W: f64 = 80.0;
const H: f64 = 52.0;
const T: f64 = 12.0;
const RC: f64 = 6.0;

fn slab() -> Body {
    let pts: Vec<Vec2> = [(0.0, 0.0), (W, 0.0), (W, H), (0.0, H)].iter().map(|&(x, y)| Vec2::new(x, y)).collect();
    let r = Region2 { outer: Loop2::polygon(&pts), holes: vec![] };
    let b = extrude(&Plane::XY, &[r], 0.0, T).unwrap().remove(0);
    fillet(&b, &[Vec3::new(0.0, 0.0, 6.0), Vec3::new(W, 0.0, 6.0), Vec3::new(W, H, 6.0), Vec3::new(0.0, H, 6.0)], RC).unwrap()
}
fn outline() -> Vec<Vec3> {
    let c = [(RC, RC, -K, -K), (W - RC, RC, K, -K), (W - RC, H - RC, K, K), (RC, H - RC, -K, K)];
    let mut e = vec![Vec3::new(W / 2.0, 0.0, T), Vec3::new(W, H / 2.0, T), Vec3::new(W / 2.0, H, T), Vec3::new(0.0, H / 2.0, T)];
    for (x, y, dx, dy) in c {
        e.push(Vec3::new(x + RC * dx, y + RC * dy, T));
    }
    e
}
fn cutter(cx: f64, cy: f64, csk: bool) -> Body {
    // revolved hole tool about the vertical axis through (cx, cy): d5.7 through, optional 90 deg countersink d12 at z = T
    let p = Plane { origin: Vec3::new(cx, cy, 0.0), x: Vec3::X, y: Vec3::Z };
    let pts: Vec<Vec2> = if csk {
        vec![(0.0, -1.0), (2.85, -1.0), (2.85, T - 3.15), (7.0, T + 1.0), (0.0, T + 1.0)]
    } else {
        vec![(0.0, -1.0), (2.85, -1.0), (2.85, T + 1.0), (0.0, T + 1.0)]
    }
    .into_iter()
    .map(|(u, v)| Vec2::new(u, v))
    .collect();
    let r = Region2 { outer: Loop2::polygon(&pts), holes: vec![] };
    revolve(&p, &[r], Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0), 2.0 * PI).unwrap().remove(0)
}
fn corner_dev(b: &Body, cx: f64, cy: f64, sx: f64, sy: f64) -> (usize, f64) {
    let m = b.mesh(&MeshTolerance::chord(0.002)).unwrap();
    let c = Vec3::new(cx, cy, T - RC);
    let mut n = 0;
    let mut worst: f64 = 0.0;
    for p in &m.positions {
        let d = *p - c;
        if sx * d.x > 0.05 && sy * d.y > 0.05 && d.z > 0.05 && (d.x * d.x + d.y * d.y + d.z * d.z).sqrt() < 9.0 {
            n += 1;
            worst = worst.max(((d.x * d.x + d.y * d.y + d.z * d.z).sqrt() - RC).abs());
        }
    }
    (n, worst)
}
#[test]
fn countersink_in_round_band_keeps_remote_ball_corners() {
    for (name, hx, csk) in [("plain hole control", 10.0, false), ("inward countersink control", 16.0, true), ("overlapping countersink", 10.0, true)]
    {
        let b = boolean(&slab(), &cutter(hx, 30.0, csk), BoolOp::Cut).unwrap().unwrap();
        let rounded = fillet(&b, &outline(), RC).unwrap();
        assert!(smlib_body(&rounded).is_ok(), "{name}: unreadable result");
        for (cx, cy, sx, sy) in [(RC, RC, -1.0, -1.0), (RC, H - RC, -1.0, 1.0), (W - RC, RC, 1.0, -1.0), (W - RC, H - RC, 1.0, 1.0)] {
            let (n, dev) = corner_dev(&rounded, cx, cy, sx, sy);
            println!("CORNER {name}: center=({cx},{cy}) n={n} deviation={dev}");
            assert!(n > 0, "{name}: corner was not sampled");
            assert!(dev < 0.01, "{name}: corner ({cx},{cy}) deviates {dev} mm from exact R6 sphere");
        }
    }
}
