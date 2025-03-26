use std::fs::File;
use std::io;

use lzip::LzipIndexOptions;

fn main() {
    let path = std::env::args()
        .skip(1)
        .next()
        .unwrap_or_else(|| "assets/lzip.main.lz".to_string());
    let file = File::open(path).unwrap();
    let mut reader = io::BufReader::new(file);

    if let Err(err) = lzip::list_files(&mut reader, &LzipIndexOptions::default()) {
        eprintln!("Error: {:?}", err);
    }
}
