use crate::options::Args;
use clap::Parser;
use lzip::ClOptions;

mod options;

fn main() {
    let args = Args::parse();
    dbg!(&args);
    let cl_opts = ClOptions::default();
    if args.list {
        lzip::list_files(&args.files, &cl_opts);
    }
    println!("lzip-rs");
}
