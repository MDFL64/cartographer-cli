use std::{path::Path, time::Duration};

use crate::region::Bounds;

pub fn fetch(bounds: Bounds, path: &Path) {
    let client = reqwest::blocking::Client::new();
    let bbox = format!("{},{},{},{}",bounds.south,bounds.west,bounds.north,bounds.east);
    // south west north east
    let query = format!(r#"
        [out:xml]
        [timeout:60]
        ;
        (
            node({bbox});
            <;
            >;

            //node(w);

            //way({bbox});
            //relation({bbox});

            // select children
            //node(r);
            //<;
            //way(r);
        );
        out body;
    "#);

    let encoded = format!("data={}",urlencoding::encode(&query));

    println!("> fetching osm...");
    let res = client.post("https://overpass-api.de/api/interpreter")
        .body(encoded)
        .timeout(Duration::from_secs(600))
        .send().unwrap();
    std::fs::write(path, res.text().unwrap()).unwrap();
}
