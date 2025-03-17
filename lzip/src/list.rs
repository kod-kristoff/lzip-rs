use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use crate::{cl_options::ClOptions, LzipIndex};

pub fn list_files(filenames: &[String], cl_opts: &ClOptions) -> io::Result<()> {
    let mut stdin_used = false;
    for filename in filenames {
        let from_stdin = filename == "-";
        if from_stdin {
            if stdin_used {
                continue;
            } else {
                stdin_used = true;
            }
        }
        let mut infd: Box<dyn BufRead> = if from_stdin {
            Box::new(BufReader::new(std::io::stdin()))
        } else {
            Box::new(BufReader::new(File::open(filename)?))
        };

        let lzip_index = LzipIndex::from_reader(infd, cl_opts)
    }
}
