//! Booleans on solidcore: the C API's own logic round `SM_API`, ported from the fork's
//! `capi/src/smlib_ops.cpp` (`smlib_boolean_with`, `smlib_boolean2d`,
//! `smlib_boolean_union_all`, `finish`, `closed_nonmanifold`) and
//! `capi/src/smlib_imprint.cpp` (`smlib_imprint`).
//!
//! As there, every operation works on copies of its operands (`SmApiBrepCopy`): `SM_API` edits
//! the first operand in place and consumes the second, and a SolidCraft body never changes.

use std::sync::PoisonError;

use solidcore::Result as SolidResult;
use solidcore::SolidError;
use solidcore::boolean::api_brep::{self as sm, Boolean2dOutcome};
use solidcore::boolean::merge::MergeOptions;
use solidcore::geom::a_object::BooleanOperation;
use solidcore::kernel::Kernel;
use solidcore::primitive::primitive_creation::p1::Boolean2d;
use solidcore::topology::brep::p3::brep_copy;
use solidcore::topology::brep::{p1, p2};
use solidcore::topology::{Brep, edge};

use super::{Body, run_on};
use crate::{BooleanOp, Error, Result};

/// What an operation's `SM_API` call gave, before it is a SolidCraft result: the capi's
/// statuses besides success.
enum Done {
    Body(Box<Brep>),
    /// `SMLIB_ERR_EMPTY`.
    Empty,
    /// `SMLIB_ERR_NOT_SOLID`.
    NotSolid,
    /// `SMLIB_ERR_NO_CUT` (`smlib_imprint`).
    NoCut,
}

impl super::dump::Answer for Done {
    fn refusal(&self) -> Option<String> {
        matches!(self, Done::NotSolid).then(|| Error::NotSolid.to_string())
    }
}

/// `GetNumFaces`.
fn num_faces(b: &Brep) -> usize {
    b.face_list().len()
}

/// `closed_nonmanifold` (smlib_ops.cpp): a closed body whose solids touch only along edges or at
/// vertices: at least two regions, no shell vertices, no wire edge, and four edgeuses (two faces,
/// both sides) per face round every edge. As there, an edge whose query fails (`e == nullptr`, a
/// failed `GetEdgeuses`) makes it false: [`edge_closes`].
fn closed_nonmanifold(r: &Brep) -> SolidResult<bool> {
    if p1::regions(r, None)?.len() < 2 {
        return Ok(false);
    }
    if !p2::shell_vertices(r, None)?.is_empty() {
        return Ok(false);
    }
    for e in p1::edges(r, None)? {
        if !edge_closes(edge::is_wire(r, e), edge::edgeuses(r, e, None, None).map(|u| u.len())) {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The capi's test of one edge in `closed_nonmanifold`, given its `IsWire` and the count of its
/// `GetEdgeuses`: not a wire, and four edgeuses per face round it. Either query failing is
/// false, not an error: the capi returns false where `GetEdgeuses` fails.
pub(super) fn edge_closes(wire: SolidResult<bool>, uses: SolidResult<usize>) -> bool {
    matches!(wire, Ok(false)) && matches!(uses, Ok(n) if n >= 4 && n % 4 == 0)
}

/// `finish` (smlib_ops.cpp): the result must have faces, and a solid input must give a closed
/// manifold solid or (`nonmanifold`) a closed one whose solids touch along an edge or at a vertex.
fn finish(r: Brep, solid_in: bool, nonmanifold: bool) -> SolidResult<Done> {
    if num_faces(&r) == 0 {
        return Ok(Done::Empty);
    }
    if solid_in && !p2::is_manifold_solid(&r)? && !(nonmanifold && closed_nonmanifold(&r)?) {
        return Ok(Done::NotSolid);
    }
    Ok(Done::Body(Box::new(r)))
}

/// What a failing call's dump calls its first operands.
const OPERANDS: [&str; 4] = ["a", "b", "c", "d"];

/// `bodies` as a call's inputs, named as its dump names its operands.
fn operands<'a>(bodies: &[&'a Body]) -> Vec<(&'static str, &'a Body)> {
    bodies.iter().enumerate().map(|(n, &b)| (OPERANDS.get(n).copied().unwrap_or("operand"), b)).collect()
}

/// Copies of `bodies`, in order (a body given twice is copied twice, as the capi copies each
/// operand).
fn copies(k: &mut Kernel, bodies: &[&Body]) -> SolidResult<Vec<Brep>> {
    // One body's lock at a time, here as everywhere: two calls never deadlock on them.
    let mut out = Vec::with_capacity(bodies.len());
    for b in bodies {
        let c = brep_copy(k, &b.0.lock().unwrap_or_else(PoisonError::into_inner))?;
        out.push(c);
    }
    Ok(out)
}

fn result(d: Done, empty: Result<Option<Body>>) -> Result<Option<Body>> {
    match d {
        Done::Body(b) => Ok(Some(Body::new(*b))),
        Done::Empty => empty,
        Done::NotSolid => Err(Error::NotSolid),
        Done::NoCut => Ok(None),
    }
}

fn operation(op: BooleanOp) -> BooleanOperation {
    match op {
        BooleanOp::Union => BooleanOperation::Union,
        BooleanOp::Subtract => BooleanOperation::Difference,
        BooleanOp::Intersect => BooleanOperation::Intersection,
    }
}

/// Whether every body is a manifold solid (`IsManifoldSolid` of each copy).
fn all_solid(bodies: &[Brep]) -> SolidResult<bool> {
    for b in bodies {
        if !p2::is_manifold_solid(b)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Fails when a panic left one of `bodies` half-edited: it is never used again.
fn usable(call: &'static str, bodies: &[&Body]) -> Result<()> {
    if bodies.iter().any(|b| b.0.is_poisoned()) {
        return Err(Error::Internal(format!("{call}: the body was left unfinished by an earlier panic")));
    }
    Ok(())
}

/// Moves each vertex of `b` whose tolerance SMLib's check refuses (not under
/// [`LARGE_SMALL_SIZE_RATIO`] times the B-rep's, `Test #82`) to the centre of the box round its
/// edges' ends, where that brings the tolerances it needs inside the limit, and sets its
/// tolerance to what they need.
///
/// The boolean grows a vertex's tolerance to twice its largest gap. A cut whose outline meets a
/// face at a shallow angle places the vertex at one edge's end with the other edges' ends up to
/// the whole gap away, so twice that gap can pass the limit although the edges end well inside
/// it. The box's centre is within half its diagonal (the ends' spread) of every end; the edges'
/// tolerances are grown to the spread, and the vertex's to twice its new largest gap
/// (vertex/face gaps included), the spread and its edges' tolerances, whichever is largest.
///
/// A move is kept only when that is inside the limit and every edge's 3D curve end and every
/// edge use's UV trim end, on its face's surface, lies within the vertex's and edge's
/// tolerances, as the validator requires of the curve ends. Otherwise, or when a gap cannot be
/// computed, the vertex and the tolerances are put back as they were, and the tolerance update
/// that follows decides as it did without this pass. A cancelled kernel is put back the same
/// way and its error returned; the pass otherwise fails only on a vertex or edge missing from
/// `b`, a broken result that the check would refuse anyway.
pub(super) fn centre_wide_vertices(k: &mut Kernel, b: &mut Brep) -> SolidResult<()> {
    use solidcore::geom::tol::LARGE_SMALL_SIZE_RATIO;

    let limit = LARGE_SMALL_SIZE_RATIO * b.tolerance();
    for v in p1::vertices(b, None)? {
        // the check wants a vertex's tolerance strictly under the limit
        if b.vertex(v)?.tolerance < limit {
            continue;
        }
        let before = Tolerances::of(b, v)?;
        match centre_vertex(k, b, v, limit) {
            Ok(true) => {}
            Err(e) if e.is_cancelled() => {
                before.restore(b)?;
                return Err(e);
            }
            Ok(false) | Err(_) => before.restore(b)?,
        }
    }
    Ok(())
}

/// One vertex of [`centre_wide_vertices`]: `true` when it was moved and its tolerances set,
/// `false` (or an error) when the move does not fit and the caller puts it back.
fn centre_vertex(k: &mut Kernel, b: &mut Brep, v: solidcore::topology::VertexId, limit: f64) -> SolidResult<bool> {
    use solidcore::Vec3;
    use solidcore::geom::tol::zone_tol3d;
    use solidcore::heal::gap::Gap;
    use solidcore::topology::{vertex, vertexuse};

    let mut ends = Vec::new();
    for vu in vertex::vertexuses(b, v)? {
        if let Some(g) = vertexuse::vertex_edge_gap(k, b, vu, true)? {
            // the gap's second end point is on the edge's curve
            ends.push(Gap::VertexEdge(g).gap_end_points(k, b)?.1);
        }
    }
    // one end gives no spread to centre on
    if ends.len() < 2 {
        return Ok(false);
    }
    let (mut lo, mut hi) = (ends[0], ends[0]);
    for p in &ends[1..] {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    // the edges' ends stay where they are: their spread is the gap between the edges, which
    // the edges' tolerances must cover as well as the vertex's
    let spread = zone_tol3d(k, lo.distance(hi));
    b.vertex_mut(v)?.point = (lo + hi) / 2.0;
    let gap = largest_gap(k, b, v)?;
    let need = zone_tol3d(k, 2.0 * gap).max(spread);
    let edges = vertex::edges(b, v, None)?;
    let mut widest = b.tolerance();
    for &e in &edges {
        widest = widest.max(b.edge(e)?.tolerance);
    }
    let tolerance = need.max(widest);
    if tolerance >= limit {
        return Ok(false);
    }
    for &e in &edges {
        edge::set_tolerance(b, e, spread, true, true)?;
    }
    b.vertex_mut(v)?.tolerance = tolerance;
    ends_within(k, b, v)
}

/// The largest vertex/edge or vertex/face gap of `v`'s uses, recomputed (their caches refreshed
/// for where the vertex is now); 0 without gaps.
fn largest_gap(k: &mut Kernel, b: &mut Brep, v: solidcore::topology::VertexId) -> SolidResult<f64> {
    use solidcore::topology::{vertex, vertexuse};

    let mut most = 0.0_f64;
    for vu in vertex::vertexuses(b, v)? {
        if let Some(g) = vertexuse::vertex_edge_gap(k, b, vu, true)? {
            most = most.max(g.gap_3d);
        }
        if let Some(g) = vertexuse::vertex_face_gap(k, b, vu, true)? {
            most = most.max(g.gap_3d);
        }
    }
    Ok(most)
}

/// Whether every end at `v` of its edges' 3D curves, and of their uses' UV trim curves on their
/// faces' surfaces, is within the vertex's tolerance plus the edge's. A trim runs with its edge
/// (its domain's start at the edge's start), as the validator compares them.
pub(super) fn ends_within(k: &Kernel, b: &Brep, v: solidcore::topology::VertexId) -> SolidResult<bool> {
    use solidcore::Point2;
    use solidcore::topology::{edgeuse, vertex};

    let at = b.vertex(v)?.point;
    let reach = b.vertex(v)?.tolerance;
    for e in vertex::edges(b, v, None)? {
        let (start, end) = edge::start_end_vertices(b, e)?;
        let ivl = edge::interval(b, e)?;
        let reach = reach + b.edge(e)?.tolerance;
        let Some(curve) = edge::curve(b, e)? else { continue };
        for (here, t) in [(start, ivl.min()), (end, ivl.max())] {
            if here == Some(v) && curve.evaluate_point(k, t)?.distance(at) > reach {
                return Ok(false);
            }
        }
        for &eu in &b.edge(e)?.edgeuses {
            let Some(trim) = b.edgeuse(eu)?.uv_trim_curve.as_ref() else { continue };
            let Some(f) = edgeuse::p1::face(b, eu)? else { continue };
            let Some(surface) = b.face(f)?.surface.as_ref() else { continue };
            let domain = trim.natural_interval();
            for (here, t) in [(start, domain.min()), (end, domain.max())] {
                if here != Some(v) {
                    continue;
                }
                let uv = trim.evaluate_point(k, t)?;
                if surface.evaluate_point(k, Point2::new(uv.x, uv.y))?.distance(at) > reach {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}

/// A vertex's point and tolerance, its edges' tolerances and their other vertices' (which
/// growing an edge's raises), to put back when [`centre_vertex`] does not keep its move. The
/// vertex's cached gaps are cleared on the way back, so they are computed again where it is.
struct Tolerances {
    vertex: solidcore::topology::VertexId,
    point: solidcore::Point3,
    vertices: Vec<(solidcore::topology::VertexId, f64)>,
    edges: Vec<(solidcore::topology::EdgeId, f64)>,
}

impl Tolerances {
    fn of(b: &Brep, v: solidcore::topology::VertexId) -> SolidResult<Self> {
        use solidcore::topology::vertex;

        let mut vertices = vec![(v, b.vertex(v)?.tolerance)];
        let mut edges = Vec::new();
        for e in vertex::edges(b, v, None)? {
            edges.push((e, b.edge(e)?.tolerance));
            if let Some(o) = edge::other_vertex(b, e, Some(v))? {
                vertices.push((o, b.vertex(o)?.tolerance));
            }
        }
        Ok(Self { vertex: v, point: b.vertex(v)?.point, vertices, edges })
    }

    fn restore(&self, b: &mut Brep) -> SolidResult<()> {
        use solidcore::topology::vertex;

        b.vertex_mut(self.vertex)?.point = self.point;
        for &(e, t) in &self.edges {
            b.edge_mut(e)?.tolerance = t;
        }
        for &(v, t) in &self.vertices {
            b.vertex_mut(v)?.tolerance = t;
        }
        for vu in vertex::vertexuses(b, self.vertex)? {
            let u = b.vertexuse_mut(vu)?;
            u.vertex_edge_gap.reset();
            u.vertex_face_gap.reset();
        }
        Ok(())
    }
}

/// `smlib_boolean` (`smlib_boolean_with` with `SMLIB_RESULT_NONMANIFOLD`): `SmApiBoolean` on
/// copies of `a` and `b`; `None` when the result is empty (`SMLIB_ERR_EMPTY`), and
/// [`Error::NotSolid`] when solid operands give a body that is not closed.
pub(crate) fn boolean(a: &Body, b: &Body, op: BooleanOp) -> Result<Option<Body>> {
    usable("SmApiBoolean", &[a, b])?;
    let d = run_on("SmApiBoolean", &operands(&[a, b]), |k| {
        let mut c = copies(k, &[a, b])?.into_iter();
        let (Some(ca), Some(cb)) = (c.next(), c.next()) else {
            return Err(SolidError::invalid_input("smlib_boolean", "no operand"));
        };
        let solid_in = p2::is_manifold_solid(&ca)? && p2::is_manifold_solid(&cb)?;
        let mut r = sm::boolean(k, ca, cb, operation(op))?;
        centre_wide_vertices(k, &mut r.result)?;
        // Grow the result's tolerances to cover marginal edge/vertex, vertex/face and
        // edge/face gaps, so an otherwise-sound union is not refused as "Bad Gap" for a
        // sub-micron gap (SolidCraft issue #10, fuzzer seed 73). Tolerances only grow:
        // a body that already passes is unchanged; a marginal failure becomes sound.
        // Offset and shell already grow their results' tolerances this way.
        solidcore::topology::brep::validation::validate_and_update_tolerances(
            k,
            &mut r.result,
            false,
            None,
            solidcore::topology::brep::validation::CheckLevel::Level2 as u8,
        )?;
        finish(r.result, solid_in, true)
    })?;
    result(d, Ok(None))
}

/// `smlib_boolean_union_all`: copies of `bodies` unioned pairwise, level by level (an odd one
/// carried to the next level as it is), each union `SmApiBoolean`'s; then `finish` with
/// non-manifold results allowed. An empty result is [`Error::Empty`].
pub(crate) fn union_all(bodies: &[&Body]) -> Result<Body> {
    if bodies.is_empty() || bodies.len() > crate::MAX_UNION_BREPS {
        return Err(Error::Argument);
    }
    usable("smlib_boolean_union_all", bodies)?;
    let d = run_on("smlib_boolean_union_all", &operands(bodies), |k| {
        let mut level = copies(k, bodies)?;
        let solid_in = all_solid(&level)?;
        while level.len() > 1 {
            let mut next = Vec::with_capacity(level.len().div_ceil(2));
            let mut it = level.into_iter();
            while let Some(a) = it.next() {
                match it.next() {
                    Some(b) => next.push(sm::boolean(k, a, b, BooleanOperation::Union)?.result),
                    None => next.push(a),
                }
            }
            level = next;
        }
        match level.pop() {
            Some(r) => finish(r, solid_in, true),
            None => Ok(Done::Empty),
        }
    })?;
    result(d, Err(Error::Empty))?.ok_or(Error::Empty)
}

/// `smlib_boolean2d`: `SmApiBoolean2d` (`SmPrimitiveCreation::Boolean2D`) on copies of `a` and
/// `b`, sheets of planar faces on one plane; `None` when either has no face (where
/// `SmApiBoolean2d` succeeds without a result) or the result is empty.
pub(crate) fn boolean2d(a: &Body, b: &Body, op: BooleanOp) -> Result<Option<Body>> {
    usable("SmApiBoolean2d", &[a, b])?;
    let d = run_on("SmApiBoolean2d", &operands(&[a, b]), |k| {
        let mut c = copies(k, &[a, b])?.into_iter();
        let (Some(ca), Some(cb)) = (c.next(), c.next()) else {
            return Err(SolidError::invalid_input("smlib_boolean2d", "no operand"));
        };
        if num_faces(&ca) == 0 || num_faces(&cb) == 0 {
            return Ok(Done::Empty);
        }
        let kind = match op {
            BooleanOp::Union => Boolean2d::Union,
            BooleanOp::Subtract => Boolean2d::Difference,
            BooleanOp::Intersect => Boolean2d::Intersection,
        };
        match sm::boolean_2d(k, ca, cb, kind)? {
            Boolean2dOutcome::Result(r) => finish(r, false, false),
            Boolean2dOutcome::Unassigned { .. } => Ok(Done::Empty),
        }
    })?;
    result(d, Ok(None))
}

/// `smlib_imprint`: `SmApiBooleanWithOptions(SM_BO_IMPRINT)` on copies of `body` and `tool`,
/// imprinting and keeping the tool (cookie cutter off, imprinting on, post-process off,
/// imprint-and-classify off, keep other on). `None` when no face was cut
/// (`SMLIB_ERR_NO_CUT`); an empty result is [`Error::Empty`]; the same body twice is
/// [`Error::Argument`].
pub(crate) fn imprint(body: &Body, tool: &Body) -> Result<Option<Body>> {
    if std::ptr::eq(body, tool) {
        return Err(Error::Argument);
    }
    usable("SmApiBooleanWithOptions", &[body, tool])?;
    let d = run_on("SmApiBooleanWithOptions", &operands(&[body, tool]), |k| {
        let mut c = copies(k, &[body, tool])?.into_iter();
        let (Some(a), Some(t)) = (c.next(), c.next()) else {
            return Err(SolidError::invalid_input("smlib_imprint", "no operand"));
        };
        let before = num_faces(&a);
        let options = MergeOptions::new(false, true, false, false, true);
        let r = sm::boolean_with_options(k, a, t, BooleanOperation::Imprint, options)?.result;
        let after = num_faces(&r);
        Ok(if after == 0 {
            Done::Empty
        } else if after == before {
            Done::NoCut
        } else {
            Done::Body(Box::new(r))
        })
    })?;
    match d {
        Done::Empty => Err(Error::Empty),
        d => result(d, Err(Error::Empty)),
    }
}
