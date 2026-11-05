use std::collections::HashMap;
use std::io::Write;

use crate::geometry::Point3;
use crate::tessellate::Tessellate;

use super::{StlError, StlWriteOptions};

type PointBits = [u32; 3];

/// Tessellates and writes binary STL. Geometry errors leave `sink` untouched;
/// an I/O error may leave a partial write, as with any `Write` operation.
pub fn write_stl(
    sink: &mut impl Write,
    shape: &(impl Tessellate + ?Sized),
    options: &StlWriteOptions,
) -> Result<(), StlError> {
    sink.write_all(&stl_to_bytes(shape, options)?)?;
    Ok(())
}

/// Returns a validated binary STL, suitable for downloads and in-memory use.
/// Coordinates are model coordinates rounded to `f32`. Facets contracted onto
/// repeated positions by rounding are omitted before checking closure. Other
/// degenerate facets, non-finite coordinates and, by default, open or
/// inconsistent edges are errors.
pub fn stl_to_bytes(
    shape: &(impl Tessellate + ?Sized),
    options: &StlWriteOptions,
) -> Result<Vec<u8>, StlError> {
    let opts = options.tessellation;
    if opts.curve.segments < 4 || opts.surface.nu == 0 || opts.surface.nv == 0 {
        return Err(StlError::InvalidSampling);
    }
    let mesh = shape.tessellate_with(opts)?.mesh;
    if mesh.indices.is_empty() || !mesh.indices.len().is_multiple_of(3) {
        return Err(StlError::InvalidTriangles);
    }
    let count = u32::try_from(mesh.indices.len() / 3).map_err(|_| StlError::TooManyTriangles)?;
    let mut out = vec![0; 80];
    out[..10].copy_from_slice(b"NGK binary");
    out.extend_from_slice(&count.to_le_bytes());
    let mut edges: HashMap<(PointBits, PointBits), (usize, i64)> = HashMap::new();
    for (triangle, indices) in mesh.indices.chunks_exact(3).enumerate() {
        let mut points = [[0.0f32; 3]; 3];
        for (point, &index) in points.iter_mut().zip(indices) {
            let position = mesh
                .positions
                .get(index as usize)
                .ok_or(StlError::MissingPosition { triangle })?;
            *point = [position.x as f32, position.y as f32, position.z as f32];
            if point.iter().any(|coordinate| !coordinate.is_finite()) {
                return Err(StlError::InvalidCoordinate { triangle });
            }
            // Signed zero denotes one point in STL geometry.
            for coordinate in point.iter_mut() {
                if *coordinate == 0.0 {
                    *coordinate = 0.0;
                }
            }
        }
        let [a, b, c] = points.map(|p| Point3::new(p[0] as f64, p[1] as f64, p[2] as f64));
        if a == b || b == c || c == a {
            let original =
                [indices[0], indices[1], indices[2]].map(|index| mesh.positions[index as usize]);
            if original[0] == original[1]
                || original[1] == original[2]
                || original[2] == original[0]
            {
                return Err(StlError::DegenerateTriangle { triangle });
            }
            // Rounding can contract a tiny edge, particularly at a pole.
            // Its incident facets vanish; closure is checked afterwards.
            continue;
        }
        let normal = (b - a).cross(&(c - a));
        let length = normal.norm();
        if length == 0.0 {
            return Err(StlError::DegenerateTriangle { triangle });
        }
        for coordinate in (normal / length).iter() {
            out.extend_from_slice(&(*coordinate as f32).to_le_bytes());
        }
        for point in points {
            for coordinate in point {
                out.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
        if options.require_closed {
            let bits = points.map(|point| point.map(f32::to_bits));
            for (a, b) in [(bits[0], bits[1]), (bits[1], bits[2]), (bits[2], bits[0])] {
                let (key, sign) = if a < b { ((a, b), 1) } else { ((b, a), -1) };
                let entry = edges.entry(key).or_default();
                entry.0 += 1;
                entry.1 += sign;
            }
        }
    }
    if edges
        .values()
        .any(|&(count, balance)| count != 2 || balance != 0)
    {
        return Err(StlError::NotClosed);
    }
    let count = (out.len() - 84) / 50;
    if count == 0 {
        return Err(StlError::InvalidTriangles);
    }
    out[80..84].copy_from_slice(&(count as u32).to_le_bytes());
    Ok(out)
}
