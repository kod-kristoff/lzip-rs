use std::error::Error;

use clap::Parser;

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

fn try_main() -> Box<dyn Error> {
    let args = Args::parse();
    if args.list {
        program_mode = Mode::List;
    } else if args.test {
        program_mode = Mode::Test;
    }
    if program_mode == Mode::List {
        list_files(&args.files, &cl_opts)?;
        return Ok(());
    }
}

fn main() {
    if let Err(err) = try_main() {
        eprintln!("Error: {:?}", err);
        sys::exit(1);
    }
}
