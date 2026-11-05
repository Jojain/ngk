use ngk::exchange::stl::{StlError, StlWriteOptions, stl_to_bytes, write_stl};
use ngk::geometry::Point3;
use ngk::modeling::solids;
use ngk::tessellate::{IndexedMesh, Tessellate, TessellateOpts, Tessellation, TessellationError};

#[test]
fn binary_stl_has_twelve_outward_facets_for_a_block() {
    let block = solids::block(2.0, 3.0, 4.0).unwrap();
    let bytes = stl_to_bytes(&block.solid(), &StlWriteOptions::default()).unwrap();
    assert_eq!(u32::from_le_bytes(bytes[80..84].try_into().unwrap()), 12);
    assert_eq!(bytes.len(), 84 + 50 * 12);
    let mut volume = 0.0;
    for facet in bytes[84..].chunks_exact(50) {
        let numbers: Vec<f64> = facet[..48]
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes(b.try_into().unwrap()) as f64)
            .collect();
        let point = |i| Point3::new(numbers[i], numbers[i + 1], numbers[i + 2]);
        let (a, b, c) = (point(3), point(6), point(9));
        let normal = (b - a).cross(&(c - a)).normalize();
        assert!((normal - point(0).coords).norm() < 1e-6);
        volume += a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
        assert_eq!(&facet[48..], &[0, 0]);
    }
    assert!((volume - 24.0).abs() < 1e-6);
}

struct Mesh(IndexedMesh);
impl Tessellate for Mesh {
    fn tessellate_with(&self, _: TessellateOpts) -> Result<Tessellation, TessellationError> {
        Ok(Tessellation {
            mesh: self.0.clone(),
            ..Default::default()
        })
    }
}

#[test]
fn invalid_meshes_leave_the_sink_untouched() {
    for mesh in [
        IndexedMesh::default(),
        IndexedMesh {
            indices: vec![0, 1],
            ..Default::default()
        },
        IndexedMesh {
            indices: vec![0, 1, 2],
            ..Default::default()
        },
        IndexedMesh {
            positions: vec![Point3::origin(); 3],
            indices: vec![0, 1, 2],
            ..Default::default()
        },
    ] {
        let mut sink = vec![42];
        assert!(write_stl(&mut sink, &Mesh(mesh), &StlWriteOptions::default()).is_err());
        assert_eq!(sink, [42]);
    }
}

#[test]
fn an_open_surface_requires_explicit_opt_in() {
    let mesh = Mesh(IndexedMesh {
        positions: vec![
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        indices: vec![0, 1, 2],
        ..Default::default()
    });
    assert!(matches!(
        stl_to_bytes(&mesh, &StlWriteOptions::default()),
        Err(StlError::NotClosed)
    ));
    let options = StlWriteOptions {
        require_closed: false,
        ..Default::default()
    };
    assert_eq!(stl_to_bytes(&mesh, &options).unwrap().len(), 134);
}

#[test]
fn curved_solids_export_as_closed_stl() {
    for (case, shape) in [
        solids::cylinder(2.0, 5.0).unwrap(),
        solids::sphere(3.0).unwrap(),
        solids::torus(5.0, 1.0).unwrap(),
    ]
    .into_iter()
    .enumerate()
    {
        let bytes = stl_to_bytes(&shape.solid(), &StlWriteOptions::default())
            .unwrap_or_else(|e| panic!("case {case}: {e}"));
        assert!(bytes.len() > 84);
    }
}
