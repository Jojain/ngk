use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::project::Project;
use super::result::{PointProjection, TargetParameters};
use crate::geometry::Point3;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

impl<P: Payload> Project<Point3> for Edge<'_, P> {
    type Mode = ();
    type Hit = EdgeKey;
    type Output = PointProjection<EdgeKey>;

    /// The nearest point on the edge's support curve, when it
    /// falls within the edge's bounds; a nearest point past either end is a
    /// [`Miss`](ProjectionError::Miss), as the edge does not extend there.
    ///
    /// A point with several equally near points on the support is refused as for a curve.
    fn project_with(
        &self,
        source: &Point3,
        _mode: (),
        options: &ProjectOptions,
    ) -> Result<PointProjection<EdgeKey>, ProjectionError> {
        let span = self.trimmed_curve();
        let nearest = span.curve().project_with(source, (), options)?;
        if !span.contains(nearest.point, options.tolerance) {
            return Err(ProjectionError::Miss);
        }
        Ok(PointProjection {
            point: nearest.point,
            // On the branch the edge's span covers, which the support's own
            // nearest point need not be on.
            parameters: Some(TargetParameters::Curve(
                span.native_parameter_at(nearest.point),
            )),
            distance: nearest.distance,
            hit: self.key(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builders::edges::add_line;
    use crate::model::Model;
    use crate::projection::project;
    use crate::topology::StandardPayload;

    #[test]
    fn a_nearest_point_within_an_edge_lands_on_it() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_line(&mut model, Point3::origin(), Point3::new(2.0, 0.0, 0.0)).unwrap();
        let edge = model.edge(key).unwrap();
        let result = project(&Point3::new(1.0, 3.0, 0.0), &edge, ()).unwrap();
        assert!((result.point - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((result.distance - 3.0).abs() < 1e-9);
        assert_eq!(result.hit, key);
        let Some(TargetParameters::Curve(t)) = result.parameters else {
            panic!("an edge's nearest point reports its parameter");
        };
        // A line spans parameter 0 to 1 between its defining points.
        assert!((t.value() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_nearest_point_past_either_end_of_an_edge_misses() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_line(&mut model, Point3::origin(), Point3::new(2.0, 0.0, 0.0)).unwrap();
        let edge = model.edge(key).unwrap();
        for x in [3.0, -1.0] {
            let result = project(&Point3::new(x, 1.0, 0.0), &edge, ());
            assert!(matches!(result, Err(ProjectionError::Miss)));
        }
    }
}
