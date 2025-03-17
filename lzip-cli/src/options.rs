#[derive(Debug, clap::Parser)]
#[clap(author, version, about, long_about = LONG_ABOUT, after_long_help=AFTER_LONG_HELP)]
pub struct Args {
    /// decompress, test compressed file integrity
    #[arg(short, long, group = "mode")]
    pub decompress: bool,
    /// print (un)compressed file sizes
    #[arg(short, long, group = "mode")]
    pub list: bool,
    /// Files to work on, if empty or '-' is given STDIN is used
    pub files: Vec<String>,
}

const LONG_ABOUT: &str = "\
Lzip is a lossless data compressor with a user interface similar to the one
of gzip or bzip2. Lzip uses a simplified form of the 'Lempel-Ziv-Markov
chain-Algorithm' (LZMA) stream format to maximize interoperability. The
maximum dictionary size is 512 MiB so that any lzip file can be decompressed
on 32-bit machines. Lzip provides accurate and robust 3-factor integrity
checking. Lzip can compress about as fast as gzip (lzip -0) or compress most
files more than bzip2 (lzip -9). Decompression speed is intermediate between
gzip and bzip2. Lzip is better than gzip and bzip2 from a data recovery
perspective. Lzip has been designed, written, and tested with great care to
replace gzip and bzip2 as the standard general-purpose compressed format for
Unix-like systems.";

const AFTER_LONG_HELP: &str = "
If no file names are given, or if a file is '-', lzip compresses or
decompresses from standard input to standard output.
Numbers may be followed by a multiplier: k = kB = 10^3 = 1000,
Ki = KiB = 2^10 = 1024, M = 10^6, Mi = 2^20, G = 10^9, Gi = 2^30, etc...
Dictionary sizes 12 to 29 are interpreted as powers of two, meaning 2^12 to
2^29 bytes.

The bidimensional parameter space of LZMA can't be mapped to a linear scale
optimal for all files. If your files are large, very repetitive, etc, you
may need to use the options --dictionary-size and --match-length directly
to achieve optimal performance.

To extract all the files from archive 'foo.tar.lz', use the commands
'tar -xf foo.tar.lz' or 'lzip -cd foo.tar.lz | tar -xf -'.

Exit status: 0 for a normal exit, 1 for environmental problems
(file not found, invalid command-line options, I/O errors, etc), 2 to
indicate a corrupt or invalid input file, 3 for an internal consistency
error (e.g., bug) which caused lzip to panic.

The ideas embodied in lzip are due to (at least) the following people:
Abraham Lempel and Jacob Ziv (for the LZ algorithm), Andrei Markov (for the
definition of Markov chains), G.N.N. Martin (for the definition of range
encoding), Igor Pavlov (for putting all the above together in LZMA), and
Julian Seward (for bzip2's CLI).

Report bugs to lzip-bug@nongnu.org
Lzip home page: http://www.nongnu.org/lzip/lzip.html";
