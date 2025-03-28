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
#[derive(Debug, clap::Parser)]
struct Args {
    /// print (un)compressed file sizes
    #[clap(short, long)]
    list: bool,
    /// test compressed file integrity
    #[clap(short = 't', long)]
    test: bool,
    //
    #[clap(short = '0', long = "fast")]
    zero: bool,
    files: Vec<String>,
}

fn main() {
    let args = Args::parse();
    if args.list {
        program_mode = Mode::List;
    } else if args.test {
        program_mode = Mode::Test;
    }
    if program_mode == Mode::List {
        list_files(&args.files, &cl_opts)?;
        return Ok(0);
    }
}
