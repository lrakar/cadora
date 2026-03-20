use truck_modeling::*;
fn main() {
    let top = builder::vertex(Point3::new(0.0, 0.0, 5.0));
    let bottom = builder::vertex(Point3::new(0.0, 0.0, -5.0));
    let arc = builder::circle_arc(&bottom, &top, Point3::new(5.0, 0.0, 0.0));
    let curve = arc.oriented_curve();
    match &curve {
        Curve::NurbsCurve(nbs) => {
            let kv = nbs.knot_vec();
            println!("arc knot_vec: {:?}", kv);
            println!("range: {} .. {}", kv[0], kv[kv.len()-1]);
            // Sample some points
            for i in 0..=10 {
                let t = kv[0] + (kv[kv.len()-1] - kv[0]) * (i as f64 / 10.0);
                let pt = nbs.subs(t);
                println!("  t={:.3} -> ({:.3}, {:.3}, {:.3})", t, pt.x, pt.y, pt.z);
            }
        }
        other => println!("curve type: {:?}", std::mem::discriminant(other)),
    }
}
