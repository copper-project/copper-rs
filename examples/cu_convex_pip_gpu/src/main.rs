use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use cu_geometry::point_in_convex_polygon;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Params {
    polygon_len: u32,
}

const WORKGROUP_SIZE: u32 = 64;

fn regular_polygon(n: usize, radius: f32) -> Vec<[f32; 2]> {
    (0..n)
        .map(|i| {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (n as f32);
            [radius * angle.cos(), radius * angle.sin()]
        })
        .collect()
}

fn generate_points(n: usize) -> Vec<[f32; 2]> {
    let mut pts = Vec::with_capacity(n);
    let side = (n as f32).sqrt().ceil() as usize;
    for i in 0..n {
        let x = (i % side) as f32 / (side as f32) * 2.0 - 1.0;
        let y = (i / side) as f32 / (side as f32) * 2.0 - 1.0;
        pts.push([x, y]);
    }
    pts
}

fn main() {
    pollster::block_on(run());
}

async fn run() {
    // init wgpu
    let instance = wgpu::Instance::default();
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions::default())
        .await
        .expect("No adapter");
    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor::default(), None)
        .await
        .expect("Device");

    // load shader
    let shader_spv = wgpu::util::make_spirv(include_bytes!(env!("KERNEL_SPV")));
    let shader_module = device.create_shader_module(&wgpu::ShaderModuleDescriptor {
        label: Some("pip"),
        source: shader_spv,
    });

    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("pip_bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });

    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("pip_pl"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[wgpu::PushConstantRange {
            stages: wgpu::ShaderStages::COMPUTE,
            range: 0..std::mem::size_of::<Params>() as u32,
        }],
    });

    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("pip_pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "point_in_polygon_kernel",
    });

    for &poly_size in &[3usize, 8usize] {
        for &num_points in &[1000usize, 10000usize] {
            let polygon = regular_polygon(poly_size, 1.0);
            let points = generate_points(num_points);

            // CPU
            let start = Instant::now();
            let mut cpu_results = 0u32;
            for p in &points {
                if point_in_convex_polygon(&polygon, *p) {
                    cpu_results += 1;
                }
            }
            let cpu_time = start.elapsed();

            // GPU buffers
            let polygon_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("poly"),
                contents: bytemuck::cast_slice(&polygon),
                usage: wgpu::BufferUsages::STORAGE,
            });
            let points_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("points"),
                contents: bytemuck::cast_slice(&points),
                usage: wgpu::BufferUsages::STORAGE,
            });
            let result_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("results"),
                size: (num_points * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("staging"),
                size: (num_points * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });

            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("pip_bg"),
                layout: &bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: polygon_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: points_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: result_buffer.as_entire_binding(),
                    },
                ],
            });

            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("pip_enc"),
            });
            {
                let mut cpass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("pip_pass"),
                });
                cpass.set_pipeline(&pipeline);
                cpass.set_bind_group(0, &bind_group, &[]);
                let params = Params {
                    polygon_len: polygon.len() as u32,
                };
                cpass.set_push_constants(0, bytemuck::bytes_of(&params));
                cpass.dispatch_workgroups(
                    ((num_points as u32) + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE,
                    1,
                    1,
                );
            }
            encoder.copy_buffer_to_buffer(
                &result_buffer,
                0,
                &staging_buffer,
                0,
                (num_points * std::mem::size_of::<u32>()) as u64,
            );
            let gpu_start = Instant::now();
            queue.submit(Some(encoder.finish()));
            let buffer_slice = staging_buffer.slice(..);
            let (tx, rx) = futures_intrusive::channel::shared::oneshot_channel();
            buffer_slice.map_async(wgpu::MapMode::Read, move |v| tx.send(v).unwrap());
            device.poll(wgpu::Maintain::Wait);
            rx.receive().await.unwrap().unwrap();
            let data = buffer_slice.get_mapped_range();
            let gpu_results = bytemuck::cast_slice::<u8, u32>(&data).iter().sum::<u32>();
            drop(data);
            staging_buffer.unmap();
            let gpu_time = gpu_start.elapsed();

            println!(
                "poly {:2} verts, points {:6}: CPU {:?} ({} inside) GPU {:?} ({} inside)",
                poly_size, num_points, cpu_time, cpu_results, gpu_time, gpu_results
            );
        }
    }
}
