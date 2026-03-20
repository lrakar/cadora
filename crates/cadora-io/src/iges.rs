//! IGES import (legacy CAD format, read-only).
//!
//! IGES (Initial Graphics Exchange Specification) is a legacy format still found
//! in older CAD files. This module provides a basic parser for IGES entity 128
//! (rational B-spline surface) and 126 (rational B-spline curve), assembling
//! them into a triangle mesh for import.
//!
//! Full IGES → BRep conversion would require a complete IGES parser;
//! this implementation handles the most common surface entities.

use cadora_brep::Mesh;
use std::io::Read;
use std::path::Path;

/// Errors that can occur during IGES import.
#[derive(Debug)]
pub enum IgesError {
    /// I/O error.
    Io(std::io::Error),
    /// The IGES data could not be parsed.
    ParseFailed(String),
    /// Unsupported IGES entity type.
    UnsupportedEntity(i32),
}

impl std::fmt::Display for IgesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "IGES I/O error: {e}"),
            Self::ParseFailed(s) => write!(f, "IGES parse error: {s}"),
            Self::UnsupportedEntity(t) => write!(f, "unsupported IGES entity type: {t}"),
        }
    }
}

impl std::error::Error for IgesError {}

impl From<std::io::Error> for IgesError {
    fn from(e: std::io::Error) -> Self { Self::Io(e) }
}

/// An IGES section identifier (column 73 of fixed-width lines).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Section {
    Start,      // S
    Global,     // G
    Directory,  // D
    Parameter,  // P
    Terminate,  // T
}

/// A directory entry (two lines in the D section).
#[derive(Debug, Clone)]
struct DirectoryEntry {
    entity_type: i32,
    param_start: usize,
    param_count: usize,
}

/// Parse section identifier from column 73 of an 80-char IGES line.
fn parse_section(line: &str) -> Option<Section> {
    let bytes = line.as_bytes();
    if bytes.len() < 73 {
        return None;
    }
    match bytes[72] {
        b'S' => Some(Section::Start),
        b'G' => Some(Section::Global),
        b'D' => Some(Section::Directory),
        b'P' => Some(Section::Parameter),
        b'T' => Some(Section::Terminate),
        _ => None,
    }
}

/// Parse all directory entries from D-section lines.
fn parse_directory(d_lines: &[String]) -> Vec<DirectoryEntry> {
    let mut entries = Vec::new();
    // Directory entries come in pairs of lines
    let mut i = 0;
    while i + 1 < d_lines.len() {
        let line1 = &d_lines[i];
        let line2 = &d_lines[i + 1];

        let entity_type = line1.get(0..8).unwrap_or("").trim().parse::<i32>().unwrap_or(0);
        let param_start = line1.get(8..16).unwrap_or("").trim().parse::<usize>().unwrap_or(0);
        let param_count = line2.get(24..32).unwrap_or("").trim().parse::<usize>().unwrap_or(0);

        entries.push(DirectoryEntry {
            entity_type,
            param_start,
            param_count,
        });
        i += 2;
    }
    entries
}

/// Collect parameter data for a directory entry from the P-section lines.
fn collect_params(p_lines: &[String], start: usize, count: usize) -> Vec<f64> {
    let mut raw = String::new();
    // P-section lines are 1-indexed; extract the relevant lines
    let begin = start.saturating_sub(1);
    let end = (begin + count).min(p_lines.len());
    for line in &p_lines[begin..end] {
        // Parameter data is columns 0..64
        let data = line.get(0..64).unwrap_or(line.as_str());
        raw.push_str(data.trim());
    }
    // Remove trailing semicolon and split by comma/semicolon
    raw = raw.trim_end_matches(';').to_string();
    // Skip entity type number (first value before first comma)
    let values: Vec<&str> = raw.split(',').collect();
    if values.len() <= 1 {
        return Vec::new();
    }
    values[1..]
        .iter()
        .filter_map(|s| {
            let s = s.trim().replace('D', "E"); // IGES uses 'D' for exponent
            s.parse::<f64>().ok()
        })
        .collect()
}

/// Evaluate a rational B-spline surface (IGES entity 128) at (u, v) and
/// tessellate into a triangle mesh.
fn tessellate_bspline_surface_128(params: &[f64], divisions: usize) -> Option<Mesh> {
    // Entity 128 parameter layout:
    //   K1, K2, M1, M2, PROP1, PROP2, PROP3, PROP4, PROP5,
    //   knots_u(K1+M1+2), knots_v(K2+M2+2),
    //   weights((K1+1)*(K2+1)),
    //   control_points((K1+1)*(K2+1)*3),
    //   u_start, u_end, v_start, v_end
    if params.len() < 9 {
        return None;
    }

    let k1 = params[0] as usize;
    let k2 = params[1] as usize;
    let m1 = params[2] as usize;
    let m2 = params[3] as usize;
    let n_knots_u = k1 + m1 + 2;
    let n_knots_v = k2 + m2 + 2;
    let n_cp_u = k1 + 1;
    let n_cp_v = k2 + 1;
    let n_cp = n_cp_u * n_cp_v;

    let offset = 9;
    let needed = offset + n_knots_u + n_knots_v + n_cp + n_cp * 3 + 4;
    if params.len() < needed {
        return None;
    }

    let knots_u = &params[offset..offset + n_knots_u];
    let knots_v = &params[offset + n_knots_u..offset + n_knots_u + n_knots_v];
    let weights_start = offset + n_knots_u + n_knots_v;
    let weights = &params[weights_start..weights_start + n_cp];
    let cp_start = weights_start + n_cp;
    let cp_end = cp_start + n_cp * 3;
    let range_start = cp_end;

    let u_start = params[range_start];
    let u_end = params[range_start + 1];
    let v_start = params[range_start + 2];
    let v_end = params[range_start + 3];

    // Evaluate using de Boor's algorithm with NURBS weights
    let eval = |u: f64, v: f64| -> [f64; 3] {
        // Simple tensor-product evaluation via basis functions
        let basis_u = bspline_basis(m1, knots_u, u, n_cp_u);
        let basis_v = bspline_basis(m2, knots_v, v, n_cp_v);

        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut wz = 0.0;
        let mut w_sum = 0.0;

        for iv in 0..n_cp_v {
            for iu in 0..n_cp_u {
                let idx = iv * n_cp_u + iu;
                let bw = basis_u[iu] * basis_v[iv] * weights[idx];
                wx += bw * params[cp_start + idx * 3];
                wy += bw * params[cp_start + idx * 3 + 1];
                wz += bw * params[cp_start + idx * 3 + 2];
                w_sum += bw;
            }
        }
        if w_sum.abs() > 1e-30 {
            [wx / w_sum, wy / w_sum, wz / w_sum]
        } else {
            [0.0, 0.0, 0.0]
        }
    };

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();

    let steps = divisions;
    for vi in 0..=steps {
        for ui in 0..=steps {
            let u = u_start + (u_end - u_start) * (ui as f64 / steps as f64);
            let v = v_start + (v_end - v_start) * (vi as f64 / steps as f64);
            let p = eval(u, v);
            positions.push(p);

            // Approximate normal
            let du = (u_end - u_start) * 1e-5;
            let dv = (v_end - v_start) * 1e-5;
            let pu = eval(u + du, v);
            let pv = eval(u, v + dv);
            let eu = [pu[0] - p[0], pu[1] - p[1], pu[2] - p[2]];
            let ev = [pv[0] - p[0], pv[1] - p[1], pv[2] - p[2]];
            let nx = eu[1] * ev[2] - eu[2] * ev[1];
            let ny = eu[2] * ev[0] - eu[0] * ev[2];
            let nz = eu[0] * ev[1] - eu[1] * ev[0];
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-14 {
                normals.push([nx / len, ny / len, nz / len]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
    }

    for vi in 0..steps {
        for ui in 0..steps {
            let i00 = vi * (steps + 1) + ui;
            let i10 = i00 + 1;
            let i01 = i00 + (steps + 1);
            let i11 = i01 + 1;
            indices.push([i00, i10, i11]);
            indices.push([i00, i11, i01]);
        }
    }

    Some(Mesh { positions, indices, normals })
}

/// Compute B-spline basis functions at parameter t.
fn bspline_basis(degree: usize, knots: &[f64], t: f64, n: usize) -> Vec<f64> {
    let p = degree;
    let m = knots.len();
    if m < 2 || n == 0 {
        return vec![0.0; n];
    }

    // Zero-degree basis
    let mut prev = vec![0.0; m];
    for i in 0..m - 1 {
        if (knots[i] <= t && t < knots[i + 1])
            || (t >= knots[m - 1] && knots[i] < knots[i + 1] && i + 1 == m - 1)
        {
            prev[i] = 1.0;
        }
    }
    // Special case: t == right endpoint with clamped knots
    // Find last span where knots[i] < knots[i+1] and activate it
    if t >= knots[m - 1] {
        prev = vec![0.0; m];
        for i in (0..m - 1).rev() {
            if knots[i] < knots[i + 1] {
                prev[i] = 1.0;
                break;
            }
        }
    }

    // Build up using Cox–de Boor recursion
    for d in 1..=p {
        let mut next = vec![0.0; m];
        for i in 0..m.saturating_sub(1 + d) {
            let denom1 = knots[i + d] - knots[i];
            let denom2 = knots[i + d + 1] - knots[i + 1];
            let a = if denom1.abs() > 1e-14 { (t - knots[i]) / denom1 * prev[i] } else { 0.0 };
            let b = if denom2.abs() > 1e-14 { (knots[i + d + 1] - t) / denom2 * prev[i + 1] } else { 0.0 };
            next[i] = a + b;
        }
        prev = next;
    }

    let mut basis = vec![0.0; n];
    for i in 0..n.min(m) {
        basis[i] = prev[i];
    }
    basis
}

/// Import IGES data from a reader.
///
/// Returns a mesh assembled from supported surface entities (type 128).
/// Unsupported entities are silently skipped.
pub fn iges_import<R: Read>(reader: &mut R) -> std::result::Result<Mesh, IgesError> {
    let mut text = String::new();
    reader.read_to_string(&mut text)?;
    iges_import_from_str(&text)
}

/// Import IGES data from a string.
pub fn iges_import_from_str(text: &str) -> std::result::Result<Mesh, IgesError> {
    let lines: Vec<String> = text.lines().map(|l| {
        // Pad short lines to 80 chars
        let mut s = l.to_string();
        while s.len() < 80 {
            s.push(' ');
        }
        s
    }).collect();

    let mut d_lines = Vec::new();
    let mut p_lines = Vec::new();

    for line in &lines {
        match parse_section(line) {
            Some(Section::Directory) => d_lines.push(line.clone()),
            Some(Section::Parameter) => p_lines.push(line.clone()),
            _ => {}
        }
    }

    if d_lines.is_empty() {
        return Err(IgesError::ParseFailed("no directory entries found".into()));
    }

    let entries = parse_directory(&d_lines);

    let mut combined = Mesh {
        positions: Vec::new(),
        indices: Vec::new(),
        normals: Vec::new(),
    };

    for entry in &entries {
        if entry.entity_type == 128 {
            // Rational B-spline surface
            let params = collect_params(&p_lines, entry.param_start, entry.param_count);
            if let Some(mesh) = tessellate_bspline_surface_128(&params, 16) {
                let base = combined.positions.len();
                combined.positions.extend_from_slice(&mesh.positions);
                combined.normals.extend_from_slice(&mesh.normals);
                for tri in &mesh.indices {
                    combined.indices.push([tri[0] + base, tri[1] + base, tri[2] + base]);
                }
            }
        }
        // Entity types 100 (circular arc), 110 (line), 126 (B-spline curve),
        // 142 (curve on surface), 144 (trimmed surface) could be added later.
    }

    Ok(combined)
}

/// Import IGES from a file on disk.
pub fn iges_import_file(path: &Path) -> std::result::Result<Mesh, IgesError> {
    let mut file = std::fs::File::open(path)?;
    iges_import(&mut file)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bspline_basis_sum_to_one() {
        // Uniform knot vector for cubic (degree 3) with 4 control points
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let n = 4;
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let basis = bspline_basis(3, &knots, t, n);
            let sum: f64 = basis.iter().sum();
            assert!(
                (sum - 1.0).abs() < 1e-10,
                "basis sum at t={t}: {sum}"
            );
        }
    }

    #[test]
    fn bspline_basis_endpoints() {
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let basis_start = bspline_basis(2, &knots, 0.0, 3);
        assert!((basis_start[0] - 1.0).abs() < 1e-10);

        let basis_end = bspline_basis(2, &knots, 1.0, 3);
        assert!((basis_end[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn parse_empty_iges() {
        // Minimal IGES with no geometry
        let iges = format!(
            "{:<72}S{:>7}\n{:<72}G{:>7}\n{:<72}D{:>7}\n{:<72}D{:>7}\n{:<72}T{:>7}\n",
            "Test IGES file", 1,
            "1H,,1H;", 1,
            "       0       0       0       0       0       0       0       0", 1,
            "       0       0       0       0       0       0       0       0", 2,
            "S      1G      1D      2P      0", 1
        );
        let result = iges_import_from_str(&iges).unwrap();
        // No entity 128 → empty mesh
        assert!(result.positions.is_empty());
    }

    #[test]
    fn section_parser() {
        let line = format!("{:<72}D{:>7}", "some data", 1);
        assert_eq!(parse_section(&line), Some(Section::Directory));

        let line = format!("{:<72}P{:>7}", "param data", 1);
        assert_eq!(parse_section(&line), Some(Section::Parameter));
    }
}
