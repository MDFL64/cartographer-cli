use std::{collections::VecDeque, sync::{Arc, Mutex}, thread::available_parallelism};

use tiff::{decoder::DecodingResult, tags::Tag};

use crate::elevation::build_terrain_mesh;

#[derive(Debug)]
struct UTMCoord {
    zone_number: u8,
    easting: f64,
    northing: f64,
}

pub struct Region {
    pub name: String,
    tiles: Vec<Arc<Tile>>,
    coord: UTMCoord
}

pub struct Tile {
    pub data: Vec<f32>,
    pub width: u32,
    pub height: u32
}

impl Region {
    pub fn new(name: String, zone_number: u8) -> Self {
        let path = format!("input/{name}.tif");
        let file = std::fs::File::open(path).expect("failed to open elevation map");
        let mut tiff = tiff::decoder::Decoder::new(file).expect("failed to decode elevation map");

        let dims= tiff.dimensions().unwrap();
        assert_eq!(dims,(10012,10012));

        let chunk_dims = tiff.chunk_dimensions();
        assert_eq!(chunk_dims,(512,512));

        let tie_point = tiff.get_tag_f64_vec(Tag::ModelTiepointTag).unwrap();
        let coord = UTMCoord {
            zone_number,
            easting: tie_point[3],
            northing: tie_point[4]
        };
        
        let mut region = Region{
            name,
            coord,
            tiles: Vec::with_capacity(400)
        };

        for i in 0..400 {
            let (width,height) = tiff.chunk_data_dimensions(i);
            let data = tiff.read_chunk(i).expect("failed to read chunk");
            let DecodingResult::F32(data) = data else {
                panic!("chunk in wrong format");
            };
            println!("> read chunk {}",i);
            region.tiles.push(Arc::new(Tile { data, width, height }));
        }
        region
    }

    pub fn ensure_out_dir_exists(&self) {
        std::fs::create_dir(format!("output/{}",self.name)).ok();
    }

    pub fn process_elevation(&self) {
        let thread_count = available_parallelism().unwrap().get();

        let queue = self.tiles.iter().enumerate().map(|(index,tile)| {
            (index,tile.clone())
        }).collect::<VecDeque<_>>();

        let queue = Arc::new(Mutex::new(queue));

        let mut threads = Vec::new();

        for _ in 0..thread_count {
            let queue = queue.clone();
            let name = self.name.to_owned();
            let thread = std::thread::spawn(move || {
                loop {
                    let item = {
                        let mut queue = queue.lock().unwrap();
                        queue.pop_front()
                    };
                    let Some((index,tile)) = item else {
                        break;
                    };
                    let buffer = build_terrain_mesh(&tile.data);
                    if let Some(buffer) = buffer {
                        std::fs::write(format!("output/{}/tile{}",name,index), buffer.bytes).unwrap();
                    }
                    println!("> elevation mesh {}",index);
                }
            });
            threads.push(thread);
        }

        for thread in threads {
            thread.join().unwrap();
        }
    }
}
