#[derive(Debug, clap::Parser)]
pub struct Args {
    /// print (un)compressed file sizes
    #[clap(short, long)]
    pub list: bool,
    /// test compressed file integrity
    #[clap(short = 't', long)]
    pub test: bool,
    /// compression level 0
    #[clap(short = '0', long = "fast")]
    pub zero: bool,
    /// be verbose (more -v gives more)
    #[arg(short, long, action = clap::ArgAction::Count)]
    pub verbose: u8,
    pub files: Vec<String>,
}
