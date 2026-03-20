//! Primitive shape creation: box, cylinder, sphere, cone, torus.

use truck_modeling::*;
use crate::shape::Shape;

/// Create a box (cuboid) with given dimensions at the origin.
///
/// The box extends from (0,0,0) to (length, width, height).
/// Uses tsweep of a rectangular face along Z.
pub fn make_box(length: f64, width: f64, height: f64) -> Shape {
    assert!(length > 0.0, "Box length must be positive");
    assert!(width > 0.0, "Box width must be positive");
    assert!(height > 0.0, "Box height must be positive");

    let wire = crate::builder::make_rect_wire(length, width);
    let face = builder::try_attach_plane(&[wire]).expect("bottom face");
    let solid: Solid = builder::tsweep(&face, Vector3::new(0.0, 0.0, height));
    Shape::from_solid(solid)
}

/// Create a cylinder along Z axis with given radius and height, centered at origin base.
///
/// Uses tsweep of a circular face along Z.
pub fn make_cylinder(radius: f64, height: f64) -> Shape {
    assert!(radius > 0.0, "Cylinder radius must be positive");
    assert!(height > 0.0, "Cylinder height must be positive");

    let wire = crate::builder::make_circle_wire(radius);
    let face = builder::try_attach_plane(&[wire]).expect("bottom disk");
    let solid: Solid = builder::tsweep(&face, Vector3::new(0.0, 0.0, height));
    Shape::from_solid(solid)
}

/// Create a sphere at the origin with the given radius.
///
/// Creates a semicircular profile in XZ plane and revolves around Z axis.
pub fn make_sphere(radius: f64) -> Shape {
    assert!(radius > 0.0, "Sphere radius must be positive");

    let top = builder::vertex(Point3::new(0.0, 0.0, radius));
    let bottom = builder::vertex(Point3::new(0.0, 0.0, -radius));

    // Semicircle from bottom to top through (radius, 0, 0) in XZ plane
    let arc = builder::circle_arc(&bottom, &top, Point3::new(radius, 0.0, 0.0));
    // Close with a line along the Z axis
    let axis_line = builder::line(&top, &bottom);

    let wire = Wire::from(vec![arc, axis_line]);
    let shell: Shell = builder::rsweep(&wire, Point3::origin(), Vector3::unit_z(), Rad(std::f64::consts::TAU));
    let solid = Solid::new(vec![shell]);
    Shape::from_solid(solid)
}

/// Create a cone/frustum with given bottom radius, top radius, and height along Z axis.
///
/// If `top_radius` is 0, creates a pointed cone.
pub fn make_cone(bottom_radius: f64, top_radius: f64, height: f64) -> Shape {
    assert!(bottom_radius >= 0.0, "Bottom radius must be non-negative");
    assert!(top_radius >= 0.0, "Top radius must be non-negative");
    assert!(
        bottom_radius > 0.0 || top_radius > 0.0,
        "At least one radius must be positive"
    );
    assert!(height > 0.0, "Cone height must be positive");

    // Build profile in XZ plane (y=0) and revolve around Z axis
    let v_axis_bot = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v_axis_top = builder::vertex(Point3::new(0.0, 0.0, height));

    let wire = if bottom_radius > 1e-14 && top_radius > 1e-14 {
        // Frustum: 4 vertices
        let v_bot_r = builder::vertex(Point3::new(bottom_radius, 0.0, 0.0));
        let v_top_r = builder::vertex(Point3::new(top_radius, 0.0, height));
        Wire::from(vec![
            builder::line(&v_axis_bot, &v_bot_r),
            builder::line(&v_bot_r, &v_top_r),
            builder::line(&v_top_r, &v_axis_top),
            builder::line(&v_axis_top, &v_axis_bot),
        ])
    } else if top_radius <= 1e-14 {
        // Pointed cone: apex at top = axis_top
        let v_bot_r = builder::vertex(Point3::new(bottom_radius, 0.0, 0.0));
        Wire::from(vec![
            builder::line(&v_axis_bot, &v_bot_r),
            builder::line(&v_bot_r, &v_axis_top),
            builder::line(&v_axis_top, &v_axis_bot),
        ])
    } else {
        // Inverted cone: apex at bottom = axis_bot
        let v_top_r = builder::vertex(Point3::new(top_radius, 0.0, height));
        Wire::from(vec![
            builder::line(&v_axis_bot, &v_top_r),
            builder::line(&v_top_r, &v_axis_top),
            builder::line(&v_axis_top, &v_axis_bot),
        ])
    };

    let shell: Shell = builder::rsweep(&wire, Point3::origin(), Vector3::unit_z(), Rad(std::f64::consts::TAU));
    let solid = Solid::new(vec![shell]);
    Shape::from_solid(solid)
}

/// Create a torus centered at the origin in the XY plane.
///
/// Uses the truck idiom: rsweep a vertex to create a tube circle,
/// then rsweep the tube circle around a second axis.
pub fn make_torus(major_radius: f64, minor_radius: f64) -> Shape {
    assert!(major_radius > 0.0, "Major radius must be positive");
    assert!(minor_radius > 0.0, "Minor radius must be positive");
    assert!(
        major_radius > minor_radius,
        "Major radius must be greater than minor radius"
    );

    // Create tube circle: vertex at outer equator, revolve around tube center along Y
    let v = builder::vertex(Point3::new(major_radius + minor_radius, 0.0, 0.0));
    let tube_circle: Wire = builder::rsweep(
        &v,
        Point3::new(major_radius, 0.0, 0.0),
        Vector3::unit_y(),
        Rad(std::f64::consts::TAU),
    );

    // Revolve tube circle around Z axis to create the torus
    let shell: Shell = builder::rsweep(
        &tube_circle,
        Point3::origin(),
        Vector3::unit_z(),
        Rad(std::f64::consts::TAU),
    );
    let solid = Solid::new(vec![shell]);
    Shape::from_solid(solid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn box_basic_dimensions() {
        let shape = make_box(10.0, 5.0, 3.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.min.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.x, 10.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.y, 5.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.z, 3.0, epsilon = 0.1);
        assert_eq!(shape.face_count(), 6);
    }

    #[test]
    fn box_cube() {
        let shape = make_box(1.0, 1.0, 1.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.volume(), 1.0, epsilon = 0.1);
        assert_eq!(shape.face_count(), 6);
    }

    #[test]
    #[should_panic(expected = "Box length must be positive")]
    fn box_zero_length_panics() {
        make_box(0.0, 1.0, 1.0);
    }

    #[test]
    fn sphere_basic() {
        let shape = make_sphere(5.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.x, -5.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.x, 5.0, epsilon = 0.5);
        assert_relative_eq!(bb.min.z, -5.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 5.0, epsilon = 0.5);
    }

    #[test]
    #[should_panic(expected = "Sphere radius must be positive")]
    fn sphere_zero_radius_panics() {
        make_sphere(0.0);
    }

    #[test]
    fn cone_basic() {
        let shape = make_cone(5.0, 0.0, 10.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 10.0, epsilon = 0.5);
    }

    #[test]
    fn cone_frustum() {
        let shape = make_cone(5.0, 2.0, 10.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 10.0, epsilon = 0.5);
    }

    #[test]
    fn torus_basic() {
        let shape = make_torus(10.0, 3.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.x, -13.0, epsilon = 1.0);
        assert_relative_eq!(bb.max.x, 13.0, epsilon = 1.0);
        assert_relative_eq!(bb.min.z, -3.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 3.0, epsilon = 0.5);
    }
}
