//! Inspect Copper points and bounding boxes in Rerun.
use cu_spatial_payloads::{BBox2i, BBox3f, Point2i, Point2iSoa, Point3f, Point3fSoa};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "geometry.rrd".into());
    let rec = rerun::RecordingStreamBuilder::new("Copper geometry").save(path)?;
    let mut points2 = Point2iSoa::<64>::default();
    let mut points3 = Point3fSoa::<64>::default();
    for i in 0..48 {
        let angle = i as f32 * core::f32::consts::TAU / 48.0;
        points2.push(Point2i::new(
            (100.0 + 70.0 * angle.cos()) as i32,
            (100.0 + 70.0 * angle.sin()) as i32,
        ));
        points3.push(Point3f::from_meters(
            angle.cos(),
            angle.sin(),
            i as f32 / 24.0,
        ));
    }
    rec.log("image/points", &points2)?;
    rec.log(
        "image/box",
        &BBox2i::new(Point2i::new(20, 20), Point2i::new(180, 180)),
    )?;
    rec.log("world/points", &points3)?;
    rec.log(
        "world/box",
        &BBox3f::new(
            Point3f::from_meters(-1.1, -1.1, -0.1),
            Point3f::from_meters(1.1, 1.1, 2.1),
        ),
    )?;
    rec.flush_blocking()?;
    Ok(())
}
