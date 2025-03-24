use std::{
    fs::File,
    io::{self, BufRead, BufReader},
};

use crate::{LzipIndex, cl_options::ClOptions};

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

        let lzip_index = {
            let lock = std::io::stdin().lock();
            let infd = if from_stdin {
                #[cfg(any(target_family = "unix", target_family = "wasm"))]
                unsafe {
                    use std::os::unix::io::{AsRawFd, FromRawFd};
                    std::fs::File::from_raw_fd(lock.as_raw_fd())
                }

                #[cfg(target_family = "windows")]
                unsafe {
                    use std::os::windows::io::{AsRawHandle, FromRawHandle};
                    std::fs::File::from_raw_handle(lock.as_raw_handle())
                }
            } else {
                std::fs::File::open(filename)?
            };
            let infd = BufReader::new(infd);
            LzipIndex::from_reader_with_options(infd, cl_opts)
        };
    }
    todo!()
}
