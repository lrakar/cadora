use serde::Serialize;
use tauri::command;

#[derive(Serialize)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<[u32; 3]>,
    pub normals: Vec<[f32; 3]>,
}

#[command]
fn create_box(width: f64, height: f64, depth: f64) -> Result<MeshData, String> {
    use cadora_brep::{Shape, TessellationParams, tessellate};
    use truck_modeling::builder;

    let v = builder::vertex(truck_modeling::Point3::new(0.0, 0.0, 0.0));
    let e = builder::tsweep(&v, truck_modeling::Vector3::new(width, 0.0, 0.0));
    let f = builder::tsweep(&e, truck_modeling::Vector3::new(0.0, height, 0.0));
    let solid = builder::tsweep(&f, truck_modeling::Vector3::new(0.0, 0.0, depth));

    let shape = Shape::from_solid(solid);
    let params = TessellationParams {
        u_divisions: 16,
        v_divisions: 16,
    };
    let mesh = tessellate(&shape, &params);

    Ok(MeshData {
        positions: mesh.positions.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect(),
        indices: mesh.indices.iter().map(|t| [t[0] as u32, t[1] as u32, t[2] as u32]).collect(),
        normals: mesh.normals.iter().map(|n| [n[0] as f32, n[1] as f32, n[2] as f32]).collect(),
    })
}

#[command]
fn create_cylinder(radius: f64, height: f64) -> Result<MeshData, String> {
    use cadora_brep::{Shape, TessellationParams, tessellate};
    use truck_modeling::{builder, Point3, Vector3, Rad, EuclideanSpace};
    use std::f64::consts::PI;

    let v = builder::vertex(Point3::new(radius, 0.0, 0.0));
    let w = builder::rsweep(&v, Point3::origin(), Vector3::unit_y(), Rad(2.0 * PI));
    let f = builder::try_attach_plane(&[w]).map_err(|e| format!("{e:?}"))?;
    let solid = builder::tsweep(&f, Vector3::new(0.0, height, 0.0));

    let shape = Shape::from_solid(solid);
    let params = TessellationParams {
        u_divisions: 32,
        v_divisions: 16,
    };
    let mesh = tessellate(&shape, &params);

    Ok(MeshData {
        positions: mesh.positions.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect(),
        indices: mesh.indices.iter().map(|t| [t[0] as u32, t[1] as u32, t[2] as u32]).collect(),
        normals: mesh.normals.iter().map(|n| [n[0] as f32, n[1] as f32, n[2] as f32]).collect(),
    })
}

#[command]
fn get_app_info() -> String {
    "Cadora CAD v0.1.0".to_string()
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .setup(|app| {
            if cfg!(debug_assertions) {
                app.handle().plugin(
                    tauri_plugin_log::Builder::default()
                        .level(log::LevelFilter::Info)
                        .build(),
                )?;
            }
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            create_box,
            create_cylinder,
            get_app_info,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
