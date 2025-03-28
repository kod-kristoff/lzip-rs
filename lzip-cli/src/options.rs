
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
