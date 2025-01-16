use core::f32;
use std::{collections::HashMap, path::{Path, PathBuf}};

use baby_shark::{decimation::{edge_decimation::{AlwaysDecimate, BoundingSphereDecimationCriteria, ConstantErrorDecimationCriteria}, prelude::EdgeDecimator}, exports::nalgebra::{Vector2, Vector3}, io::stl::{StlReader, StlWriter}, mesh::{corner_table::table::CornerTable, traits::Mesh}};
use osmio::{obj_types::StringWay, Node, OSMObj, OSMObjBase, OSMReader, Way};
use region::Region;
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

    /// Generate elevation tiles?
    #[arg(short, long)]
    elevation: bool,

    /// Generate map file?
    #[arg(short, long)]
    map: bool
}

fn main() {
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

fn read_osm(path: &Path, region: &Region) -> Buffer {
    let base_x = region.coord.easting;
    let base_y = region.coord.northing;

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

    fn road_lanes(way: &StringWay) -> i32 {
        if let Some(lanes) = way.tag("lanes") {
            let lanes: Result<i32,_> = lanes.parse();
            if let Ok(lanes) = lanes {
                return lanes;
            }
        }
        2
    }

    fn is_road(way: &StringWay) -> bool {
        way.tag("highway").is_some()
    }

    enum RoadKind {
        Road{lanes: i32},
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

                buffer.write_byte(OBJ_BUILDING);
                buffer.write_float(base_x);
                buffer.write_float(base_y);
                buffer.write_float(ground_bot);
                buffer.write_float(ground_top);
                buffer.write_float(building_height(way));
                buffer.write_short(path.len().try_into().expect("too many nodes"));
                for (x,y) in path {
                    buffer.write_float(x);
                    buffer.write_float(y);
                }
                
            } else if is_road(&way) {
                let kind = road_kind(&way);
                let half_width = match kind {
                    RoadKind::FootPath | RoadKind::BikePath => 1.0,
                    RoadKind::Road { lanes } => lanes as f32 * 3.0
                };

                let (base_x,base_y) = mean_pos(way, &nodes);
                let base_elevation = region.get_elevation(base_x, base_y);

                buffer.write_byte(OBJ_ROAD);
                buffer.write_float(base_x);
                buffer.write_float(base_y);
                buffer.write_float(base_elevation);

                let ids = way.nodes();
                let path_len = ids.len();
                buffer.write_short(path_len.try_into().expect("too many nodes"));
                
                struct RoadNode {
                    center: Vector2<f32>,
                    left: Vector3<f32>,
                    right: Vector3<f32>,
                    normal: Vector3<f32>
                }

                let mut base_path = Vec::with_capacity(path_len);

                for id in ids {
                    let (x,y) = nodes.get(id).unwrap();
                    base_path.push(RoadNode{
                        center: Vector2::new(*x, *y),
                        left: Vector3::default(),
                        right: Vector3::default(),
                        normal: Vector3::new(0.0,0.0,1.0),
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

                    let dir_side = Vector2::new(dir.y,-dir.x);

                    let mut left = make3d(node.center + dir_side * half_width);
                    let mut right = make3d(node.center - dir_side * half_width);

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
                    println!("{:?}",dir_up);
                    base_path[i].normal = dir_up;
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
