//! Placement & transforms — position + rotation in 3-D space.
//!
//! Uses nalgebra `UnitQuaternion` internally.  Mirrors FreeCAD's
//! `Base::Placement` + `Base::Rotation`.

use nalgebra::{UnitQuaternion, Vector3, Unit as NalUnit};

// ── Rotation ───────────────────────────────────────────────────────

/// 3-D rotation stored as a unit quaternion.
#[derive(Debug, Clone, Copy)]
pub struct Rotation {
    pub(crate) q: UnitQuaternion<f64>,
}

impl Rotation {
    /// Identity rotation.
    pub fn identity() -> Self {
        Self { q: UnitQuaternion::identity() }
    }

    /// From axis (must be non-zero) and angle in radians.
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Self {
        let v = Vector3::new(axis[0], axis[1], axis[2]);
        if let Some(unit) = NalUnit::try_new(v, 1e-15) {
            Self { q: UnitQuaternion::from_axis_angle(&unit, angle) }
        } else {
            Self::identity()
        }
    }

    /// From quaternion components (i, j, k, w). Normalized automatically.
    pub fn from_quaternion(i: f64, j: f64, k: f64, w: f64) -> Self {
        let q = nalgebra::Quaternion::new(w, i, j, k);
        Self { q: UnitQuaternion::from_quaternion(q.normalize()) }
    }

    /// Shortest rotation from direction A to direction B.
    pub fn from_two_vectors(from: [f64; 3], to: [f64; 3]) -> Self {
        let a = Vector3::new(from[0], from[1], from[2]);
        let b = Vector3::new(to[0], to[1], to[2]);
        Self { q: UnitQuaternion::rotation_between(&a, &b).unwrap_or_else(UnitQuaternion::identity) }
    }

    /// Euler angles (intrinsic ZYX convention — yaw, pitch, roll), in radians.
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        Self { q: UnitQuaternion::from_euler_angles(roll, pitch, yaw) }
    }

    /// Extract Euler angles (roll, pitch, yaw) in ZYX convention. Returns radians.
    pub fn to_euler_zyx(&self) -> (f64, f64, f64) {
        let (roll, pitch, yaw) = self.q.euler_angles();
        (yaw, pitch, roll)
    }

    /// Extract axis and angle (radians). Returns `None` if identity.
    pub fn to_axis_angle(&self) -> Option<([f64; 3], f64)> {
        self.q.axis_angle().map(|(axis, angle)| {
            ([axis[0], axis[1], axis[2]], angle)
        })
    }

    /// Get quaternion as (i, j, k, w).
    pub fn as_quaternion(&self) -> (f64, f64, f64, f64) {
        let q = self.q.quaternion();
        (q.i, q.j, q.k, q.w)
    }

    /// Compose two rotations: self then other.
    pub fn compose(&self, other: &Rotation) -> Rotation {
        Rotation { q: self.q * other.q }
    }

    /// Inverse rotation.
    pub fn inverse(&self) -> Rotation {
        Rotation { q: self.q.inverse() }
    }

    /// Rotate a point.
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        let v = self.q * Vector3::new(p[0], p[1], p[2]);
        [v.x, v.y, v.z]
    }

    /// Is this the identity rotation?
    pub fn is_identity(&self, tol: f64) -> bool {
        (self.q.angle()).abs() < tol
    }

    /// Spherical linear interpolation.
    pub fn slerp(&self, other: &Rotation, t: f64) -> Rotation {
        Rotation { q: self.q.slerp(&other.q, t) }
    }
}

impl Default for Rotation {
    fn default() -> Self { Self::identity() }
}

impl PartialEq for Rotation {
    fn eq(&self, other: &Self) -> bool {
        // Quaternions q and -q represent the same rotation
        let d = self.q.quaternion().coords - other.q.quaternion().coords;
        let s = self.q.quaternion().coords + other.q.quaternion().coords;
        d.norm() < 1e-12 || s.norm() < 1e-12
    }
}

// ── Placement ──────────────────────────────────────────────────────

/// Position + rotation in 3-D space.
#[derive(Debug, Clone, Copy, Default)]
pub struct Placement {
    pub position: [f64; 3],
    pub rotation: Rotation,
}

impl Placement {
    pub fn new(position: [f64; 3], rotation: Rotation) -> Self {
        Self { position, rotation }
    }

    /// Identity placement (origin, no rotation).
    pub fn identity() -> Self {
        Self { position: [0.0; 3], rotation: Rotation::identity() }
    }

    /// Translation only.
    pub fn from_translation(x: f64, y: f64, z: f64) -> Self {
        Self { position: [x, y, z], rotation: Rotation::identity() }
    }

    /// From position + axis-angle rotation.
    pub fn from_pos_axis_angle(pos: [f64; 3], axis: [f64; 3], angle: f64) -> Self {
        Self { position: pos, rotation: Rotation::from_axis_angle(axis, angle) }
    }

    /// Compose: self then other.
    /// result.pos = self.rot * other.pos + self.pos
    /// result.rot = self.rot * other.rot
    pub fn compose(&self, other: &Placement) -> Placement {
        let rotated = self.rotation.transform_point(other.position);
        Placement {
            position: [
                self.position[0] + rotated[0],
                self.position[1] + rotated[1],
                self.position[2] + rotated[2],
            ],
            rotation: self.rotation.compose(&other.rotation),
        }
    }

    /// Inverse placement.
    pub fn inverse(&self) -> Placement {
        let inv_rot = self.rotation.inverse();
        let neg_pos = inv_rot.transform_point(self.position);
        Placement {
            position: [-neg_pos[0], -neg_pos[1], -neg_pos[2]],
            rotation: inv_rot,
        }
    }

    /// Transform a point: rotate then translate.
    pub fn transform_point(&self, p: [f64; 3]) -> [f64; 3] {
        let rotated = self.rotation.transform_point(p);
        [
            rotated[0] + self.position[0],
            rotated[1] + self.position[1],
            rotated[2] + self.position[2],
        ]
    }

    /// Translate by a displacement vector.
    pub fn translate(&mut self, dx: f64, dy: f64, dz: f64) {
        self.position[0] += dx;
        self.position[1] += dy;
        self.position[2] += dz;
    }

    pub fn is_identity(&self, tol: f64) -> bool {
        self.position[0].abs() < tol
            && self.position[1].abs() < tol
            && self.position[2].abs() < tol
            && self.rotation.is_identity(tol)
    }

    /// Spherical linear interpolation between two placements.
    pub fn slerp(a: &Placement, b: &Placement, t: f64) -> Placement {
        Placement {
            position: [
                a.position[0] + t * (b.position[0] - a.position[0]),
                a.position[1] + t * (b.position[1] - a.position[1]),
                a.position[2] + t * (b.position[2] - a.position[2]),
            ],
            rotation: a.rotation.slerp(&b.rotation, t),
        }
    }

    pub fn is_same(&self, other: &Placement, tol: f64) -> bool {
        (self.position[0] - other.position[0]).abs() < tol
            && (self.position[1] - other.position[1]).abs() < tol
            && (self.position[2] - other.position[2]).abs() < tol
            && self.rotation == other.rotation
    }
}

impl PartialEq for Placement {
    fn eq(&self, other: &Self) -> bool {
        self.is_same(other, 1e-12)
    }
}

// ── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn rotation_identity() {
        let r = Rotation::identity();
        assert!(r.is_identity(1e-12));
        let p = r.transform_point([1.0, 2.0, 3.0]);
        assert!((p[0] - 1.0).abs() < 1e-12);
        assert!((p[1] - 2.0).abs() < 1e-12);
        assert!((p[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn rotation_90_around_z() {
        let r = Rotation::from_axis_angle([0.0, 0.0, 1.0], PI / 2.0);
        let p = r.transform_point([1.0, 0.0, 0.0]);
        assert!((p[0]).abs() < 1e-10);
        assert!((p[1] - 1.0).abs() < 1e-10);
        assert!((p[2]).abs() < 1e-10);
    }

    #[test]
    fn rotation_compose_inverse() {
        let r1 = Rotation::from_axis_angle([0.0, 0.0, 1.0], PI / 4.0);
        let r2 = Rotation::from_axis_angle([0.0, 0.0, 1.0], PI / 4.0);
        let composed = r1.compose(&r2); // 90°
        let p = composed.transform_point([1.0, 0.0, 0.0]);
        assert!((p[0]).abs() < 1e-10);
        assert!((p[1] - 1.0).abs() < 1e-10);

        let inv = composed.inverse();
        let identity = composed.compose(&inv);
        assert!(identity.is_identity(1e-10));
    }

    #[test]
    fn rotation_axis_angle_roundtrip() {
        let r = Rotation::from_axis_angle([0.0, 1.0, 0.0], 1.23);
        let (axis, angle) = r.to_axis_angle().unwrap();
        assert!((axis[1] - 1.0).abs() < 1e-10);
        assert!((angle - 1.23).abs() < 1e-10);
    }

    #[test]
    fn rotation_euler_roundtrip() {
        let r = Rotation::from_euler_zyx(0.3, 0.2, 0.1);
        let (yaw, pitch, roll) = r.to_euler_zyx();
        assert!((yaw - 0.3).abs() < 1e-10);
        assert!((pitch - 0.2).abs() < 1e-10);
        assert!((roll - 0.1).abs() < 1e-10);
    }

    #[test]
    fn rotation_slerp() {
        let r0 = Rotation::identity();
        let r1 = Rotation::from_axis_angle([0.0, 0.0, 1.0], PI);
        let mid = r0.slerp(&r1, 0.5);
        let (_, angle) = mid.to_axis_angle().unwrap();
        assert!((angle - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn rotation_from_two_vectors() {
        let r = Rotation::from_two_vectors([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let p = r.transform_point([1.0, 0.0, 0.0]);
        assert!((p[0]).abs() < 1e-10);
        assert!((p[1] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn placement_compose() {
        let p1 = Placement::from_translation(1.0, 0.0, 0.0);
        let p2 = Placement::from_translation(0.0, 2.0, 0.0);
        let composed = p1.compose(&p2);
        assert!((composed.position[0] - 1.0).abs() < 1e-12);
        assert!((composed.position[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn placement_compose_with_rotation() {
        // Place at (1,0,0), rotate 90° around Z
        let p1 = Placement::new(
            [1.0, 0.0, 0.0],
            Rotation::from_axis_angle([0.0, 0.0, 1.0], PI / 2.0),
        );
        // Place at (1,0,0), no rotation
        let p2 = Placement::from_translation(1.0, 0.0, 0.0);
        let composed = p1.compose(&p2);
        // p1 rotates (1,0,0) to (0,1,0), then adds (1,0,0) → (1,1,0)
        assert!((composed.position[0] - 1.0).abs() < 1e-10);
        assert!((composed.position[1] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn placement_inverse() {
        let p = Placement::new(
            [1.0, 2.0, 3.0],
            Rotation::from_axis_angle([0.0, 0.0, 1.0], PI / 3.0),
        );
        let inv = p.inverse();
        let identity = p.compose(&inv);
        assert!(identity.is_identity(1e-10));
    }

    #[test]
    fn placement_transform_point() {
        let p = Placement::from_translation(10.0, 20.0, 30.0);
        let result = p.transform_point([1.0, 2.0, 3.0]);
        assert!((result[0] - 11.0).abs() < 1e-12);
        assert!((result[1] - 22.0).abs() < 1e-12);
        assert!((result[2] - 33.0).abs() < 1e-12);
    }

    #[test]
    fn placement_slerp() {
        let a = Placement::from_translation(0.0, 0.0, 0.0);
        let b = Placement::from_translation(10.0, 0.0, 0.0);
        let mid = Placement::slerp(&a, &b, 0.5);
        assert!((mid.position[0] - 5.0).abs() < 1e-12);
    }
}
