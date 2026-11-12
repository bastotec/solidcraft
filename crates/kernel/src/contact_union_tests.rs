//! Joins of two bodies that only touch on a face, where the contact is interrupted.

// A wall carrying a slot and a through bore, joined with a block that stands on one of its
// faces and has an arc notch whose chord lies within the slot: the contact is two rectangles
// beside the slot. The join is exact (no operand may be moved to get it).
mod notched_block_on_a_bored_wall {
    use crate::*;
    use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};

    const R: f64 = 3.2;
    /// The notch's axis lies this far inside the wall.
    const D: f64 = 1.8;

    fn cut(a: &Body, b: &Body) -> Body {
        boolean(a, b, BoolOp::Cut).unwrap().unwrap()
    }

    /// The wall (or, with `floor`, an L of a floor and the wall, one profile) with its slot and
    /// its bore along y; `second_bore` adds a bore along z crossing the first.
    fn wall(floor: bool, second_bore: bool) -> Body {
        let base = if floor {
            let pts = [[0., -3.], [16., -3.], [16., 1.], [6., 1.], [6., 12.], [0., 12.]];
            let outer = Loop2::polygon(&pts.iter().map(|p| Vec2::new(p[0], p[1])).collect::<Vec<_>>());
            extrude(&Plane::YZ, &[Region2 { outer, holes: vec![] }], 0.0, 40.0).unwrap().pop().unwrap()
        } else {
            box_solid(Vec3::ZERO, Vec3::new(40.0, 6.0, 12.0)).unwrap()
        };
        let slotted = cut(&base, &box_solid(Vec3::new(6.85, -1.0, 2.0), Vec3::new(13.15, 7.0, 14.0)).unwrap());
        let bored = cut(&slotted, &cylinder(Vec3::new(30.0, -1.0, 6.0), Vec3::Y, 2.0, 8.0).unwrap());
        if second_bore { cut(&bored, &cylinder(Vec3::new(30.0, 3.0, -1.0), Vec3::Z, 1.5, 14.0).unwrap()) } else { bored }
    }

    /// The block, 14 × 7 × 8, on the wall's face y = 0 (`far`: on y = 6), notched by an arc of
    /// radius [`R`] about an axis [`D`] inside the wall.
    fn block(far: bool) -> Body {
        let half = (R * R - D * D).sqrt();
        let p = Vec2::new;
        let line = |a: Vec2, b: Vec2| Seg2::Line { a, b };
        let lift = D.atan2(half);
        let sweep = -(std::f64::consts::PI - 2.0 * lift);
        let segs = if far {
            vec![
                line(p(10.0 + half, 6.0), p(17.0, 6.0)),
                line(p(17.0, 6.0), p(17.0, 13.0)),
                line(p(17.0, 13.0), p(3.0, 13.0)),
                line(p(3.0, 13.0), p(3.0, 6.0)),
                line(p(3.0, 6.0), p(10.0 - half, 6.0)),
                Seg2::Arc { center: p(10.0, 6.0 - D), radius: R, start: std::f64::consts::PI - lift, sweep },
            ]
        } else {
            vec![
                line(p(10.0 - half, 0.0), p(3.0, 0.0)),
                line(p(3.0, 0.0), p(3.0, -7.0)),
                line(p(3.0, -7.0), p(17.0, -7.0)),
                line(p(17.0, -7.0), p(17.0, 0.0)),
                line(p(17.0, 0.0), p(10.0 + half, 0.0)),
                Seg2::Arc { center: p(10.0, D), radius: R, start: -lift, sweep },
            ]
        };
        extrude(&Plane::XY, &[Region2 { outer: Loop2 { segs }, holes: vec![] }], 1.0, 9.0).unwrap().pop().unwrap()
    }

    /// The area where the block meets the wall's face: its side face (14 × 8, less the notch's
    /// chord) less the part over the slot.
    fn wall_contact() -> f64 {
        let half = (R * R - D * D).sqrt();
        2.0 * ((7.0 - half) * 8.0 - (10.0 - half - 6.85) * 7.0)
    }

    /// The area of the block's profile, which is what stands on a floor under it.
    fn floor_contact() -> f64 {
        let half = (R * R - D * D).sqrt();
        14.0 * 7.0 - (R * R * (D / R).acos() - D * half)
    }

    /// A body's volume and area from a mesh of one fixed chord error. `measure` picks its chord error from
    /// the body's size and its tightest edge curvature, both of which a join changes: the meshes
    /// of the operands and of the result would then differ by more than a moved face does.
    fn mesh_measure(b: &Body) -> (f64, f64) {
        let m = b.tessellate_with(2e-4, true).unwrap().measure();
        (m.volume, m.area)
    }

    /// The join is one closed solid of the operands' volume, nothing moved: the block's outer
    /// face and its notch are where they were drawn, and the faces in contact (of area
    /// `contact`) are gone from both bodies rather than left back to back inside the result.
    fn assert_exact(wall: &Body, block: &Body, far: bool, contact: f64, what: &str) {
        let ((wall_volume, wall_area), (block_volume, block_area)) = (mesh_measure(wall), mesh_measure(block));
        let want = wall_volume + block_volume;
        let (r, d) = capture_boolean_diagnostics(|| boolean(wall, block, BoolOp::Union));
        let r = r.unwrap().unwrap();
        assert!(!d.events.iter().any(|e| e.strategy == BooleanStrategy::ToolNudge), "{what}: {d:?}");
        assert_eq!(r.shell_count(), 1, "{what}: one solid");
        assert!(r.validity().is_empty(), "{what}: {:?}", r.validity());
        let (got, area) = mesh_measure(&r);
        assert!((got - want).abs() <= 1e-9 * want, "{what}: volume {got} vs {want}");
        let skin = wall_area + block_area - 2.0 * contact;
        assert!((area - skin).abs() <= 0.05, "{what}: area {area} vs {skin}: faces in contact were left in the result");
        let (out, nominal) = if far { (1.0, 13.0) } else { (-1.0, -7.0) };
        let outer = (0..r.face_count()).any(|i| match face_surface(&r, i) {
            Some(FaceSurface::Plane { normal, point }) => normal.dot(Vec3::Y) * out > 1.0 - 1e-12 && (point.y - nominal).abs() < 1e-9,
            _ => false,
        });
        assert!(outer, "{what}: the block's outer face is not on y = {nominal}, where it was drawn");
        let axis_y = if far { 6.0 - D } else { D };
        let notch = (0..r.face_count()).any(|i| match face_surface(&r, i) {
            Some(FaceSurface::Cylinder { origin, axis, radius }) => {
                let off = Vec3::new(10.0, axis_y, origin.z) - origin;
                (radius - R).abs() < 1e-9 && axis.dot(Vec3::Z).abs() > 1.0 - 1e-12 && off.len() < 1e-9
            }
            _ => false,
        });
        assert!(notch, "{what}: the notch's cylinder (r {R} about x = 10, y = {axis_y}) is not in the result");
    }

    #[test]
    fn joins_exactly_on_the_near_face() {
        assert_exact(&wall(false, false), &block(false), false, wall_contact(), "near face");
    }

    #[test]
    fn joins_exactly_on_the_far_face() {
        assert_exact(&wall(false, false), &block(true), true, wall_contact(), "far face");
    }

    // A second bore crossing the first adds faces far from the contact; the join stays exact.
    // It was also where the fallback cost most: the first of these two joins took 57 s on a
    // machine where each now takes about 2.5 s. Not asserted: a test must not depend on timing.
    #[test]
    fn joins_exactly_beside_crossing_bores() {
        assert_exact(&wall(false, true), &block(false), false, wall_contact(), "near face, crossing bores");
        assert_exact(&wall(false, true), &block(true), true, wall_contact(), "far face, crossing bores");
    }

    #[test]
    fn joins_exactly_in_a_corner() {
        assert_exact(&wall(true, false), &block(true), true, wall_contact() + floor_contact(), "floor and wall");
    }
}
