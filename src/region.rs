use std::{collections::VecDeque, path::Path, sync::{Arc, Mutex}, thread::available_parallelism};

use tiff::{decoder::DecodingResult, tags::Tag};

use crate::{elevation::build_terrain_mesh, osm_fetch, read_osm};

#[derive(Debug)]
pub struct UTMCoord {
    pub zone_number: u8,
    pub easting: f64,
    pub northing: f64,
}

pub struct Region {
    pub name: String,
    pub coord: UTMCoord,
    tiles: Vec<Arc<Tile>>,
}

pub struct Tile {
    pub data: Vec<f32>,
    pub width: u32,
    pub height: u32
}

impl Tile {
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width as usize + x]
    }
}

pub struct TileNeighbors {
    pub next_x: Option<Arc<Tile>>,
    pub next_y: Option<Arc<Tile>>,
    pub corner: Option<Arc<Tile>>
}

pub struct Bounds {
    pub south: f64,
    pub north: f64,
    pub east: f64,
    pub west: f64
}

const REGION_SIZE: u32 = 10012;

impl Region {
    pub fn new(name: String, zone_number: u8) -> Self {
        let path = format!("input/{name}.tif");
        let file = std::fs::File::open(path).expect("failed to open elevation map");
        let mut tiff = tiff::decoder::Decoder::new(file).expect("failed to decode elevation map");

        let dims= tiff.dimensions().unwrap();
        assert_eq!(dims,(REGION_SIZE,REGION_SIZE));

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
            let neighbors = TileNeighbors{
                next_x: if tile.width == 512 { Some(self.tiles[index + 1].clone()) } else { None },
                next_y: if tile.height == 512 { Some(self.tiles[index + 20].clone()) } else { None },
                corner: if tile.width == 512 && tile.height == 512 { Some(self.tiles[index + 21].clone()) } else { None },
            };

            (index,tile.clone(),neighbors)
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
                    let Some((index,tile, neighbors)) = item else {
                        break;
                    };
                    let buffer = build_terrain_mesh(&tile.data, tile.width as usize, tile.height as usize, neighbors);
                    //std::fs::write(format!("output/{}/tile{}",name,index), buffer.bytes).unwrap();
                    buffer.save(&name, &format!("tile{}",index));
                    println!("> elevation mesh {}",index);
                }
            });
            threads.push(thread);
        }

        for thread in threads {
            thread.join().unwrap();
        }
    }

    pub fn process_osm(&self) {
        let path = format!("input/{}.osm",self.name);
        if std::fs::metadata(&path).is_err() {
            osm_fetch::fetch(self.get_bounds(), Path::new(&path));
        }

        let buffer = read_osm(Path::new(&path), self);
        buffer.save(&self.name, "map");
        println!("> map done");
    }

    pub fn get_elevation(&self, x: f32, y: f32) -> f32 {
        let x = x.clamp(0.01, REGION_SIZE as f32 - 0.01);
        let y = y.clamp(0.01, REGION_SIZE as f32 - 0.01);

        let chunk_size = 512.0;
        let chunk_count = 20;
        let cx = (x / chunk_size).floor() as i32;
        let cy = (y / chunk_size).floor() as i32;
        if cx < 0 || cy < 0 || cx >= chunk_count || cy >= chunk_count {
            panic!("bad coord");
        }

        let chunk_index = (cy * chunk_count + cx) as usize;
        let tile = &self.tiles[chunk_index];

        let xx = (x % chunk_size) as u32;
        let yy = (y % chunk_size) as u32;

        tile.data[(tile.width * yy + xx) as usize]
    }

    pub fn get_bounds(&self) -> Bounds {
        // todo southern hemisphere
        let zone_letter = 'T';

        let (north,west) = utm::wsg84_utm_to_lat_lon(self.coord.easting, self.coord.northing, self.coord.zone_number, zone_letter).unwrap();
        let (south,east) = utm::wsg84_utm_to_lat_lon(self.coord.easting + REGION_SIZE as f64, self.coord.northing - REGION_SIZE as f64, self.coord.zone_number, zone_letter).unwrap();

        Bounds { north, south, east, west }
    }
}
