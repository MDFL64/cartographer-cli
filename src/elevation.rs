use std::collections::HashMap;

use baby_shark::{decimation::{edge_decimation::ConstantErrorDecimationCriteria, prelude::EdgeDecimator}, exports::nalgebra::Vector3, mesh::{corner_table::table::CornerTable, traits::Mesh}};

use crate::{region::TileNeighbors, Buffer};

fn make_grid(width: usize, height: usize, scale: f64, mut f: impl FnMut(usize,usize)->f64) -> CornerTable<f64> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for y in 0..height {
        for x in 0..width {
            let z = f(x,y);
            vertices.push(Vector3::new(x as f64, y as f64, z) * scale);
            if x < width - 1 && y < height - 1 {
                let index = y*width + x;
                indices.push(index+0);
                indices.push(index+1);
                indices.push(index+width);

                indices.push(index+1);
                indices.push(index+width+1);
                indices.push(index+width);
            }
        }
    }

    CornerTable::from_vertices_and_indices(&vertices, &indices)
}

pub fn build_terrain_mesh(tile: &[f32], width: usize, height: usize, neighbors: TileNeighbors) -> Buffer {
    if tile.len() != width*height {
        panic!("tile sized wrongly")
    }

    let scale = 1.0;
    let max_error = 1.0;

    let criteria = ConstantErrorDecimationCriteria::new(scale * max_error);
    let mut decimator = EdgeDecimator::new()
        .decimation_criteria(criteria)
        .min_faces_count(Some(10_000))
        .keep_boundary(true);

    let fixed_width = if width == 512 { width + 1 } else { width };
    let fixed_height = if height == 512 { height + 1 } else { height };

    let mut mesh = make_grid(fixed_width, fixed_height, scale, |x,y| {
        if x >= width && y >= width {
            let neighbor = neighbors.corner.as_ref().unwrap();
            neighbor.get(0,0) as f64
        } else if x >= width {
            let neighbor = neighbors.next_x.as_ref().unwrap();
            neighbor.get(0,y) as f64
        } else if y >= height {
            let neighbor = neighbors.next_y.as_ref().unwrap();
            neighbor.get(x,0) as f64
        } else {
            let e = tile[y * width + x];
            e as f64
        }
    });

    println!("initial: {} / {}",mesh.vertices().count(),mesh.faces().count());
    //StlWriter::new().write_stl_to_file(&mesh, Path::new("C:\\Users\\cogg\\Documents\\init.stl")).unwrap();
    decimator.decimate(&mut mesh);
    println!("decimated: {} / {}",mesh.vertices().count(),mesh.faces().count());

    assert!(mesh.vertices().count() < 60_000);
    assert!(mesh.faces().count() < 60_000);

    //StlWriter::new().write_stl_to_file(&mesh, Path::new("C:\\Users\\cogg\\Documents\\dec1.stl")).unwrap();

    let mut buffer = Buffer::default();

    let mut min_z = 1.0f64/0.0;
    let mut max_z = -1.0f64/0.0;
    for i in mesh.vertices() {
        let pos = mesh.vertex_position(&i);
        //let normal = mesh.vertex_normal(&i).unwrap();
        min_z = min_z.min(pos.z);
        max_z = max_z.max(pos.z);
    }
    let range_z = max_z - min_z;

    buffer.write_float(min_z as f32);
    buffer.write_float(range_z as f32);
    buffer.write_short(mesh.vertices().count() as u16);

    let mut map = HashMap::<usize,u16>::new();
    let mut next_vert_index = 0;
    for i in mesh.vertices() {
        map.insert(i, next_vert_index);
        next_vert_index += 1;
        {
            let pos = mesh.vertex_position(&i);
            let x = pos.x / 512.0 * 65535.0;
            let y = pos.y / 512.0 * 65535.0;
            let z = (pos.z - min_z) / range_z * 65535.0;
            buffer.write_short(x as u16);
            buffer.write_short(y as u16);
            buffer.write_short(z as u16);
        }
        {
            let normal = mesh.vertex_normal(&i).unwrap();
            let x = normal.x * 127.0;
            let y = normal.y * 127.0;
            let z = normal.z * 127.0;
            buffer.write_byte(x as i8 as u8);
            buffer.write_byte(y as i8 as u8);
            buffer.write_byte(z as i8 as u8);
        }
    }

    buffer.write_short(mesh.faces().count() as u16);
    for i in mesh.faces() {
        let (a,b,c) = mesh.face_vertices(&i);
        let a = *map.get(&a).unwrap();
        let b = *map.get(&b).unwrap();
        let c = *map.get(&c).unwrap();
        buffer.write_short(b);
        buffer.write_short(a);
        buffer.write_short(c);
    }
    buffer
}
