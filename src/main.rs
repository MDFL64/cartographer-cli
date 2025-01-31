use core::f32;
use std::{collections::HashMap, io::Write, path::{Path, PathBuf}};

use baby_shark::{decimation::{edge_decimation::{AlwaysDecimate, BoundingSphereDecimationCriteria, ConstantErrorDecimationCriteria}, prelude::EdgeDecimator}, exports::nalgebra::{Vector2, Vector3}, io::stl::{StlReader, StlWriter}, mesh::{corner_table::table::CornerTable, traits::Mesh}};
use flate2::{write::GzEncoder, Compression};
use osmio::{obj_types::StringWay, Node, OSMObj, OSMObjBase, OSMReader, Way};
use region::Region;
use tiff::{decoder::DecodingResult, tags::Tag};
use clap::Parser;

mod region;
mod elevation;
mod osm_fetch;

#[derive(Parser, Debug)]
#[command()]
struct CommandArgs {
    /// The input height-map to process
    name: String,

    /// The UTM zone of the region
    zone_number: u8,

    /// Generate elevation tiles?
    #[arg(short, long)]
    elevation: bool,

    /// Generate map file?
    #[arg(short, long)]
    map: bool
}

fn main() {
    //osm_fetch::fetch();
    //panic!();

    let cli_args = CommandArgs::parse();

    let region = Region::new(cli_args.name, cli_args.zone_number);

    region.ensure_out_dir_exists();
    if cli_args.elevation {
        region.process_elevation();
    }
    if cli_args.map {
        region.process_osm();
    }
}

const OBJ_BUILDING: u8 = 0;
const OBJ_ROAD: u8 = 1;

#[repr(u8)]
enum BuildingKind {
    House, // siding, maybe brick, usually pitched roofs
    Tower, // skyscraper
    Commercial, // flat with few windows -- shops, theaters, etc
    Industrial, // Icky
    Parking,    // Parking garage
    School,     // Bricks?
    Hospital
}

#[repr(u8)]
enum RoofKind {
    Flat
}

fn read_osm(path: &Path, region: &Region) -> Buffer {
    let base_x = region.coord.easting;
    let base_y = region.coord.northing;

    fn is_building(way: &StringWay) -> bool {
        way.tag("building").is_some()
    }

    fn building_height(way: &StringWay) -> f32 {
        if let Some(height) = way.tag("height") {
            // very bare-bones height parsing attempt, TODO units
            let height: Result<f32,_> = height.parse();
            if let Ok(height) = height {
                return height;
            }
        }
        if let Some(levels) = way.tag("building:levels") {
            let levels: Result<f32,_> = levels.parse();
            if let Ok(levels) = levels {
                return levels * 3.0;
            }
        }
        3.0
    }

    fn building_infer_kind(way: &StringWay, area: f32, height: f32) -> BuildingKind {
        if height > 10.0 {
            BuildingKind::Tower
        } else if area > 500.0 {
            BuildingKind::Commercial
        } else {
            BuildingKind::House
        }
    }

    fn path_area(path: &[(f32,f32)]) -> f32 {
        // just finds the area of the bounds
        // TODO actually calculate area
        if path.len() < 3 {
            return 0.0;
        }
        let mut x_min = 1f32/0.0;
        let mut y_min = 1f32/0.0;
        let mut x_max = -1f32/0.0;
        let mut y_max = -1f32/0.0;
        for (x,y) in path {
            x_min = x_min.min(*x);
            y_min = y_min.min(*y);
            x_max = x_max.max(*x);
            y_max = y_max.max(*y);
        }
        let w = x_max - x_min;
        let h = y_max - y_min;
        w * h
    }

    fn building_color(way: &StringWay) -> u32 {
        if let Some(color) = way.tag("building:colour") {
            println!("color = {}",color);
        }
        if let Some(color) = way.tag("roof:colour") {
            println!("roof color = {}",color);
        }
        // ~
        if let Some(color) = way.tag("building:material") {
            println!("mat = {}",color);
        }
        if let Some(color) = way.tag("material") {
            println!("dumb mat = {}",color);
        }
        if let Some(color) = way.tag("roof:material") {
            println!("roof mat = {}",color);
        }
        0
    }

    fn is_road(way: &StringWay) -> bool {
        way.tag("highway").is_some()
    }
    
    fn is_road_oneway(way: &StringWay) -> bool {
        way.tag("oneway").is_some()
    }

    fn road_lanes(way: &StringWay) -> f32 {
        if let Some(lanes) = way.tag("lanes") {
            let lanes: Result<f32,_> = lanes.parse();
            if let Ok(lanes) = lanes {
                if lanes >= 1.0 {
                    return lanes;
                } else {
                    return 1.0;
                }
            }
        }
        2.0
    }

    fn should_skip_road(way: &StringWay) -> bool {
        way.tag("tunnel").is_some() || way.tag("bridge").is_some() || way.tag("highway") == Some("steps")
    }

    enum RoadKind {
        Road{lanes: f32},
        FootPath,
        BikePath
    }

    impl RoadKind {
        pub fn is_level_path(&self) -> bool {
            match self {
                Self::BikePath | Self::FootPath => true,
                _ => false
            }
        }
    }

    fn road_kind(way: &StringWay) -> RoadKind {
        let highway_val = way.tag("highway");
        if highway_val == Some("footway") || highway_val == Some("path") || way.tag("footway").is_some() {
            RoadKind::FootPath
        } else if highway_val == Some("cycleway") {
            RoadKind::BikePath
        } else {
            let lanes = road_lanes(way);
            RoadKind::Road{lanes}
        }
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
            nodes.insert(node.id(), (x as f32,y as f32));
        } else if let Some(way) = obj.as_way() {
            if is_building(&way) {
                let (base_x,base_y) = mean_pos(way, &nodes);
                let mut ground_top = -1.0 / 0.0;
                let mut ground_bot = 1.0 / 0.0;

                let ids = way.nodes();
                // do not include duplicate final node
                let path_len = ids.len()-1;
                let mut path = Vec::with_capacity(path_len);
                for i in 0..path_len {
                    let (x,y) = nodes.get(&ids[i]).unwrap();
                    let e = region.get_elevation(*x, *y);
                    if e > ground_top {
                        ground_top = e;
                    }
                    if e < ground_bot {
                        ground_bot = e;
                    }
                    path.push((*x - base_x, *y - base_y));
                }
                if is_ccw(&path) {
                    path.reverse();
                }

                let mut height = building_height(way);
                let area = path_area(&path);
                let kind = building_infer_kind(way, area, height);
                let roof_kind = RoofKind::Flat;
                // bump up height for non-houses
                match kind {
                    BuildingKind::Commercial | BuildingKind::Industrial => {
                        height = height.max(6.0)
                    }
                    _ => ()
                }

                buffer.write_byte(OBJ_BUILDING);
                buffer.write_float(base_x);
                buffer.write_float(base_y);
                buffer.write_float(ground_bot);
                buffer.write_float(ground_top);
                buffer.write_float(height);
                buffer.write_byte(kind as u8);
                buffer.write_byte(roof_kind as u8);
                buffer.write_short(path.len().try_into().expect("too many nodes"));
                for (x,y) in path {
                    buffer.write_float(x);
                    buffer.write_float(y);
                }
                
            } else if is_road(&way) {
                if should_skip_road(&way) {
                    continue;
                }
                let kind = road_kind(&way);
                let half_width = match kind {
                    RoadKind::FootPath | RoadKind::BikePath => 1.0,
                    RoadKind::Road { lanes } => lanes as f32 * 1.5
                };

                let (base_x,base_y) = mean_pos(way, &nodes);
                let base_elevation = region.get_elevation(base_x, base_y);

                buffer.write_byte(OBJ_ROAD);
                buffer.write_float(base_x);
                buffer.write_float(base_y);
                buffer.write_float(base_elevation);

                if let RoadKind::Road { lanes } = kind {
                    let kind = if is_road_oneway(way) { 2 } else { 1 };
                    buffer.write_byte(kind);
                    buffer.write_byte(lanes.ceil() as u8);
                } else {
                    buffer.write_byte(0);
                    buffer.write_byte(1);
                }
                // type

                let ids = way.nodes();
                let path_len = ids.len();
                buffer.write_short(path_len.try_into().expect("too many nodes"));
                
                struct RoadNode {
                    center: Vector2<f32>,
                    left: Vector3<f32>,
                    right: Vector3<f32>,
                    normal: Vector3<f32>,
                    direction: Vector3<f32>,
                }

                let mut base_path = Vec::with_capacity(path_len);

                for id in ids {
                    let (x,y) = nodes.get(id).unwrap();
                    base_path.push(RoadNode{
                        center: Vector2::new(*x, *y),
                        left: Vector3::default(),
                        right: Vector3::default(),
                        normal: Vector3::new(0.0,0.0,1.0),
                        direction: Vector3::new(1.0,0.0,0.0)
                    });
                }

                let make3d = |coord: Vector2<f32>| {
                    let e = region.get_elevation(coord.x, coord.y);
                    Vector3::new(coord.x - base_x,coord.y - base_y, e - base_elevation)
                };

                // place left and right nodes
                for i in 0..base_path.len() {
                    let node = &base_path[i];

                    let dir_1 = if i > 0 {
                        let prev = &base_path[i-1];
                        Some( (node.center - prev.center).normalize() )
                    } else {
                        None
                    };
                    let dir_2 = if i < base_path.len()-1 {
                        let next = &base_path[i+1];
                        Some( (next.center - node.center).normalize() )
                    } else {
                        None
                    };

                    let dir = match (dir_1,dir_2) {
                        (Some(a),Some(b)) => (a + b) * 0.5,
                        (Some(a),None) => a,
                        (None,Some(a)) => a,
                        _ => panic!("bad dir")
                    };

                    let mut width_mul = 1.0;

                    if let (Some(a),Some(b)) = (dir_1,dir_2) {
                        // correction maxes out at 90 degrees
                        let angle = a.angle(&b).min(1.57);
                        width_mul = 1.0 / (angle / 2.0).cos();
                    }

                    let dir_side = Vector2::new(dir.y,-dir.x);

                    let mut left = make3d(node.center + dir_side * half_width * width_mul);
                    let mut right = make3d(node.center - dir_side * half_width * width_mul);

                    if kind.is_level_path() {
                        let z = left.z.max(right.z);
                        left.z = z;
                        right.z = z;
                    }

                    let node = &mut base_path[i];
                    node.left = left;
                    node.right = right;
                }

                // calculate normal -- requires 3d node coords
                for i in 0..base_path.len() {
                    let node = &base_path[i];

                    let dir_1 = if i > 0 {
                        let prev = &base_path[i-1];
                        Some( (node.left - prev.left).normalize() )
                    } else {
                        None
                    };

                    let dir_2 = if i < base_path.len()-1 {
                        let next = &base_path[i+1];
                        Some( (next.left - node.left).normalize() )
                    } else {
                        None
                    };

                    let dir_fwd = match (dir_1,dir_2) {
                        (Some(a),Some(b)) => (a + b) * 0.5,
                        (Some(a),None) => a,
                        (None,Some(a)) => a,
                        _ => panic!("bad dir")
                    };

                    let dir_side = (node.right - node.left).normalize();

                    let dir_up = dir_fwd.cross(&dir_side);
                    base_path[i].normal = dir_up;
                    base_path[i].direction = dir_fwd;
                }

                for node in base_path {
                    buffer.write_float(node.left.x);
                    buffer.write_float(node.left.y);
                    buffer.write_float(node.left.z);
                    buffer.write_float(node.right.x);
                    buffer.write_float(node.right.y);
                    buffer.write_float(node.right.z);
                    buffer.write_float(node.normal.x);
                    buffer.write_float(node.normal.y);
                    buffer.write_float(node.normal.z);
                    buffer.write_float(node.direction.x);
                    buffer.write_float(node.direction.y);
                    buffer.write_float(node.direction.z);
                }
            }
        }
    }

    buffer
}

#[derive(Default)]
struct Buffer {
    bytes: Vec<u8>,
}

impl Buffer {
    pub fn save(&self, region: &str, filename: &str) {
        let out_path = format!("output/{}/{}.bin.gz",region,filename);

        let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
        encoder.write_all(&self.bytes).unwrap();
        let data = encoder.finish().unwrap();

        std::fs::write(Path::new(&out_path), data).unwrap();
    }

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
