use core::f32;
use std::{collections::HashMap, path::{Path, PathBuf}};

use baby_shark::{decimation::{edge_decimation::{AlwaysDecimate, BoundingSphereDecimationCriteria, ConstantErrorDecimationCriteria}, prelude::EdgeDecimator}, exports::nalgebra::Vector3, io::stl::{StlReader, StlWriter}, mesh::{corner_table::table::CornerTable, traits::Mesh}};
use osmio::{obj_types::StringWay, Node, OSMObj, OSMObjBase, OSMReader, Way};
use region::Region;
use rouille::{Response, ResponseBody};
use tiff::{decoder::DecodingResult, tags::Tag};
use clap::Parser;

mod region;
mod elevation;

#[derive(Parser, Debug)]
#[command()]
struct CommandArgs {
    /// The input height-map to process
    name: String,

    /// The UTM zone of the region
    zone_number: u8,

    /// Number of times to greet
    #[arg(short, long)]
    elevation: bool,
}

fn request_error(text: &str) -> Response {
    Response{
        data: ResponseBody::from_data(text),
        headers: vec!(),
        status_code: 400,
        upgrade: None
    }
}

fn respond_bytes(buffer: Buffer) -> Response {
    Response{
        data: ResponseBody::from_data(buffer.bytes),
        headers: vec!(),
        status_code: 200,
        upgrade: None
    }
}

fn main() {
    let cli_args = CommandArgs::parse();

    let region = Region::new(cli_args.name, cli_args.zone_number);

    if cli_args.elevation {
        region.process_elevation();
    }

    //println!("{:?}",cli_args);
    panic!();

    /*for i in 0..30 {
        build_grid("donkey_west",i);
    }*/
    rouille::start_server("localhost:8080",move |request| {
        let url = request.url();
        let mut path = url.split("/");
        let empty = path.next();
        if empty != Some("") {
            return request_error("bad url");
        }

        let Some(region) = path.next() else {
            return request_error("no region");
        };
        if !region.chars().all(|c: char| c.is_ascii_alphanumeric() || c == '_') {
            return request_error("invalid region");
        }
        match path.next() {
            Some("e") => {
                if let Some(n) = path.next() {
                    match n {
                        "info" => {
                            panic!("info");
                        }
                        "base" => {
                            panic!("lod");
                        }
                        n => {
                            let Ok(n) = n.parse::<u32>() else {
                                return request_error("bad elevation sub-resource");
                            };
                            match read_tile(region,n) {
                                Ok(buffer) => {
                                    respond_bytes(buffer)
                                }
                                Err(err) => request_error(&err)
                            }
                        }
                    }
                } else {
                    request_error("no elevation sub-resource specified")
                }
            }
            Some("osm") => {
                let buffer = read_osm(Path::new("map_source/map.osm"), region, 449993.99997825973, 4910006.000037198);
                respond_bytes(buffer)
            }
            Some("file") => {
                let name = path.next().unwrap();
                let buffer = Buffer{
                    bytes: std::fs::read(Path::new(name)).unwrap()
                };
                respond_bytes(buffer)
            }
            _ => {
                request_error("bad kind index")
            }
        }
    });
}

const OBJ_BUILDING: u8 = 0;
const OBJ_ROAD: u8 = 1;

fn read_osm(path: &Path, name: &str, base_x: f64, base_y: f64) -> Buffer {
    //let region_size = 10012.0;
    //let base_y = base_y - 10012.0;

    let file = std::fs::File::open(format!("elevation/{name}.tif")).unwrap();
    let tiff = tiff::decoder::Decoder::new(file).unwrap();
    let mut finder = ElevationFinder{
        tiff,
        cache: vec!()
    };

    fn is_building(way: &StringWay) -> bool {
        way.tag("building").is_some()
    }

    fn building_height(way: &StringWay) -> f32 {
        if let Some(levels) = way.tag("building:levels") {
            let levels: Result<f32,_> = levels.parse();
            if let Ok(levels) = levels {
                return levels * 3.0;
            }
        }
        3.0
    }

    fn is_road(way: &StringWay) -> bool {
        way.tag("highway").is_some()
    }

    fn mean_pos(way: &StringWay, nodes: &HashMap<i64,(f32,f32)>) -> (f32,f32) {
        let mut count = 0;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        for id in way.nodes() {
            let (x,y) = nodes.get(id).unwrap();
            sum_x += *x;
            sum_y += *y;
            count += 1;
        }
        (sum_x / count as f32, sum_y / count as f32)
    }

    fn is_ccw(points: &[(f32,f32)]) -> bool {
        let mut sum = 0.0;
        for i in 0..points.len() {
            let (x1,y1) = points[i];
            let (x2,y2) = points[(i+1)%points.len()];
            sum += (x2 - x1)*(y2 + y1);
        }
        sum < 0.0
    }

    let mut buffer = Buffer::default();

    let file = std::fs::File::open(path).unwrap();
    let mut reader = osmio::xml::XMLReader::new(file);

    let mut nodes = HashMap::new();

    for obj in reader.objects() {
        if let Some(node) = obj.as_node() {
            let (lat,long) = node.lat_lon_f64().unwrap();
            let (mut y,mut x,_) = utm::to_utm_wgs84_no_zone(lat, long);
            x -= base_x;
            y -= base_y;
            y = -y;
            //println!("{} {}",x,y);
            nodes.insert(node.id(), (x as f32,y as f32));
        } else if let Some(way) = obj.as_way() {
            if is_building(&way) {
                let (base_x,base_y) = mean_pos(way, &nodes);
                if let Some(elevation) = finder.get_elevation(base_x, base_y) {
                    buffer.write_byte(OBJ_BUILDING);
                    buffer.write_float(base_x);
                    buffer.write_float(base_y);
                    buffer.write_float(elevation);
                    buffer.write_float(building_height(way));
                    
                    let ids = way.nodes();
                    // do not include duplicate final node
                    let path_len = ids.len()-1;
                    let mut path = Vec::with_capacity(path_len);
                    for i in 0..path_len {
                        let (x,y) = nodes.get(&ids[i]).unwrap();
                        path.push((*x - base_x, -(*y - base_y)));
                    }
                    if !is_ccw(&path) {
                        path.reverse();
                    }
    
                    buffer.write_short(path.len().try_into().expect("too many nodes"));
                    for (x,y) in path {
                        buffer.write_float(x);
                        buffer.write_float(y);
                    }
                }
            } else if is_road(&way) {
                let (base_x,base_y) = mean_pos(way, &nodes);
                if let Some(base_elevation) = finder.get_elevation(base_x, base_y) {
                    buffer.write_byte(OBJ_ROAD);
                    buffer.write_float(base_x);
                    buffer.write_float(base_y);
                    buffer.write_float(base_elevation);

                    let ids = way.nodes();
                    // do not include duplicate final node
                    let path_len = ids.len();
                    buffer.write_short(path_len.try_into().expect("too many nodes"));
                    for id in ids {
                        let (x,y) = nodes.get(id).unwrap();
                        let e = finder.get_elevation(*x, *y).unwrap_or(base_elevation);
                        buffer.write_float(*x - base_x);
                        buffer.write_float(-(*y - base_y));
                        buffer.write_float(e - base_elevation);
                    }
                }
            }
        }
    }

    buffer
}

struct ElevationFinder {
    tiff: tiff::decoder::Decoder<std::fs::File>,
    cache: Vec<CachedTile>
}

struct CachedTile {
    index: u32,
    data: Vec<f32>
}

impl ElevationFinder {
    fn get_tile(&mut self, index: u32) -> Option<&[f32]> {
        for i in (0..self.cache.len()).rev() {
            let tile = &self.cache[i];
            if tile.index == index {
                if i != self.cache.len()-1 {
                    let item = self.cache.remove(i);
                    self.cache.push(item);
                }
                return self.cache.last().map(|item| item.data.as_slice());
            }
        }

        // limit cache size
        if self.cache.len() > 10 {
            self.cache.remove(0);
        }

        if let Ok(chunk) = self.tiff.read_chunk(index) {
            if let DecodingResult::F32(data) = chunk {
                self.cache.push(CachedTile {
                    index,
                    data
                });
                return self.cache.last().map(|item| item.data.as_slice());
            }
        }

        None
    }

    fn get_elevation(&mut self, x: f32, y: f32) -> Option<f32> {
        let chunk_size = 512.0;
        let chunk_count = 20;
        let cx = (x / chunk_size).floor() as i32;
        let cy = (y / chunk_size).floor() as i32;
        if cx < 0 || cy < 0 || cx >= chunk_count || cy >= chunk_count {
            None
        } else {
            let chunk_index = (cy * chunk_count + cx) as u32;
            let (width,height) = self.tiff.chunk_data_dimensions(chunk_index);

            if let Some(data) = self.get_tile(chunk_index) {
                let xx = (x % chunk_size) as u32;
                let yy = (y % chunk_size) as u32;

                let e = data[(width * yy + xx) as usize];
                return Some(e);
            }
            
            None
        }
    }
}

fn read_tile_raw(name: &str, index: u32) -> Result<Vec<f32>, String> {
    let file = std::fs::File::open(format!("elevation/{name}.tif")).map_err(|_| "failed to load data for region, does it exist?")?;
    let mut tiff = tiff::decoder::Decoder::new(file).map_err(|_| "failed to read data for region, maybe corrupted?")?;

    let data = tiff.read_chunk(index).map_err(|_| "failed to read tile at index, is it in bounds?")?;
    if let DecodingResult::F32(data) = data {
        Ok(data)
    } else {
        Err("elevation data is in incorrect format".to_owned())
    }
}

fn read_tile(name: &str, index: u32) -> Result<Buffer, String> {
    let file = std::fs::File::open(format!("elevation/{name}.tif")).map_err(|_| "failed to load data for region, does it exist?")?;
    let mut tiff = tiff::decoder::Decoder::new(file).map_err(|_| "failed to read data for region, maybe corrupted?")?;
    //let dims= tiff.dimensions().unwrap();
    //println!("dims {:?}",dims);
    let tie_point = tiff.get_tag_f64_vec(Tag::ModelTiepointTag).unwrap();
    //println!("tie {:?}",tie_point);

    let data = tiff.read_chunk(index).map_err(|_| "failed to read tile at index, is it in bounds?")?;
    if let DecodingResult::F32(data) = data {
        let (width,height) = tiff.chunk_data_dimensions(index);
        let mut buffer = Buffer::default();

        buffer.write_short(width as u16);
        buffer.write_short(height as u16);

        let mut min = f32::INFINITY;
        let mut max = -f32::INFINITY;
        for n in data.iter().copied() {
            if n < min {
                min = n;
            }
            if n > max {
                max = n;
            }
        }
        let range = max - min;

        buffer.write_float(min);
        buffer.write_float(range);

        // invert image
        for y in (0..height).rev() {
            for x in 0..width {
                let index = (y * width + x) as usize;
                let height = data[index];
                buffer.write_short( ((height - min) / range * 65535.0) as u16 );
            }
        }
        Ok(buffer)
    } else {
        return Err("elevation data is in incorrect format".to_owned());
    }
}

#[derive(Default)]
struct Buffer {
    bytes: Vec<u8>,
}

impl Buffer {
    pub fn write_byte(&mut self, x: u8) {
        self.bytes.push(x);
    }

    pub fn write_short(&mut self, x: u16) {
        let bytes = x.to_le_bytes();
        self.bytes.push(bytes[0]);
        self.bytes.push(bytes[1]);
    }

    pub fn write_float(&mut self, x: f32) {
        let bytes = x.to_le_bytes();
        self.bytes.push(bytes[0]);
        self.bytes.push(bytes[1]);
        self.bytes.push(bytes[2]);
        self.bytes.push(bytes[3]);
    }
}
