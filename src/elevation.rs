use std::collections::HashMap;

use baby_shark::{decimation::{edge_decimation::ConstantErrorDecimationCriteria, prelude::EdgeDecimator}, exports::nalgebra::Vector3, mesh::{corner_table::table::CornerTable, traits::Mesh}};

use crate::Buffer;

fn make_grid(size: usize, scale: f64, mut f: impl FnMut(usize,usize)->f64) -> CornerTable<f64> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for y in 0..size {
        for x in 0..size {
            let z = f(x,y);
            vertices.push(Vector3::new(x as f64, y as f64, z) * scale);
            if x < size - 1 && y < size - 1 {
                let index = y*size + x;
                indices.push(index+0);
                indices.push(index+1);
                indices.push(index+size);

                indices.push(index+1);
                indices.push(index+size+1);
                indices.push(index+size);
            }
        }
    }

    CornerTable::from_vertices_and_indices(&vertices, &indices)
}

pub fn build_terrain_mesh(tile: &[f32]) -> Option<Buffer> {
    let size = 512;
    if tile.len() != size*size {
        return None;
    }

    let scale = 1.0;
    let max_error = 1.0;

    let criteria = ConstantErrorDecimationCriteria::new(scale * max_error);
    let mut decimator = EdgeDecimator::new()
        .decimation_criteria(criteria)
        .min_faces_count(Some(10_000))
        .keep_boundary(true);

    let mut mesh = make_grid(512, scale, |x,y| {
        let e = tile[y * 512 + x];
        e as f64
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
    Some(buffer)
}
