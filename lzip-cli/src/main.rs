use std::error::Error;

use clap::Parser;
use lzip::{LzipIndexOptions, list_files};

use crate::options::Args;

struct LzmaOptions {
    dictionary_size: i32, // 4 KiB .. 512 MiB
    match_len_limit: i32, // 5 .. 273
}

#[derive(Debug, PartialEq)]
enum Mode {
    Compress,
    Decompress,
    List,
    Test,
}

mod options;

fn try_main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let program_mode = if args.list {
        Mode::List
    } else if args.test {
        Mode::Test
    } else {
        unreachable!()
    };
    let opts = LzipIndexOptions::default();
    let verbosity = args.verbose;
    if program_mode == Mode::List {
        list_files(&args.files, verbosity, &opts)?;
        return Ok(());
    }
    Ok(())
}

fn main() {
    if let Err(err) = try_main() {
        eprintln!("Error: {:?}", err);
        std::process::exit(1);
    }
}
