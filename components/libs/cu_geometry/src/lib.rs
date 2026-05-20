/// Geometry algorithms utilities.

/// Checks whether a 2D point lies inside a convex polygon.
///
/// The polygon vertices must be provided in counter-clockwise order.
/// Returns `true` if the point lies inside or on the boundary of the polygon.
///
/// # Arguments
///
/// * `polygon` - Slice of vertices defined as `[x, y]` pairs.
/// * `point` - Point to test defined as `[x, y]`.
///
/// # Examples
///
/// ```
/// use cu_geometry::point_in_convex_polygon;
/// let polygon = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
/// let inside = point_in_convex_polygon(&polygon, [0.2, 0.2]);
/// assert!(inside);
/// ```
pub fn point_in_convex_polygon(polygon: &[[f32; 2]], point: [f32; 2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut prev_sign = 0.0f32;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let edge = [b[0] - a[0], b[1] - a[1]];
        let to_point = [point[0] - a[0], point[1] - a[1]];
        let cross = edge[0] * to_point[1] - edge[1] * to_point[0];
        if i == 0 {
            prev_sign = cross;
        } else if cross * prev_sign < 0.0 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::point_in_convex_polygon;

    #[test]
    fn triangle_point_inside() {
        let triangle = vec![[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]];
        assert!(point_in_convex_polygon(&triangle, [0.5, 0.5]));
    }

    #[test]
    fn triangle_point_outside() {
        let triangle = vec![[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]];
        assert!(!point_in_convex_polygon(&triangle, [2.0, 2.0]));
    }

    #[test]
    fn square_point_inside() {
        let square = vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        assert!(point_in_convex_polygon(&square, [0.0, 0.0]));
    }

    #[test]
    fn square_point_outside() {
        let square = vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        assert!(!point_in_convex_polygon(&square, [2.0, 0.0]));
    }

    #[test]
    fn point_on_edge() {
        let triangle = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
        assert!(point_in_convex_polygon(&triangle, [0.5, 0.5]));
        assert!(point_in_convex_polygon(&triangle, [0.5, 0.0]));
    }
}
