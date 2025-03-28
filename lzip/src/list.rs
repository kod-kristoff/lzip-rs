use std::{
    fs::File,
    io::{self, BufRead, BufReader, Seek},
    path::PathBuf,
};

use crate::lzip_index::{LzipIndex, LzipIndexOptions};

// void list_line( const unsigned long long uncomp_size,
//                 const unsigned long long comp_size,
//                 const char * const input_filename )
//   {
//   if( uncomp_size > 0 )
//     std::printf( "%14llu %14llu %6.2f%%  %s\n", uncomp_size, comp_size,
//                   100.0 - ( ( 100.0 * comp_size ) / uncomp_size ),
//                   input_filename );
//   else
//     std::printf( "%14llu %14llu   -INF%%  %s\n", uncomp_size, comp_size,
//                   input_filename );
//   }

// } // end namespace

// const STDIN_NAME: &Path = &Path::new(i"-");
// int list_files( const std::vector< std::string > & filenames,
//                 const Cl_options & cl_opts )

fn format_ds(dictionary_size: u32) -> String {
    const BUFSIZE: usize = 16;
    const FACTOR: u32 = 1024;
    const N: usize = 3;
    const PREFIX: [&str; N] = ["Ki", "Mi", "Gi"];
    let mut num = dictionary_size;
    let mut exact = num % FACTOR == 0;
    let mut p = "";
    let mut np = " ";
    let mut i = 0;
    while i < N && (num > 9999 || (exact && num >= FACTOR)) {
        num /= FACTOR;
        if num % FACTOR != 0 {
            exact = false;
        }
        p = PREFIX[i];
        np = "";
        i += 1;
    }
    format!("{}{} {}B", np, num, p)
}
pub fn list_files(
    filenames: &[String],
    verbosity: u8,
    opts: &LzipIndexOptions,
) -> Result<(), io::Error> {
    let mut total_comp = 0;
    let mut total_uncomp = 0;
    let mut num_files = 0;
    //   unsigned long long total_comp = 0, total_uncomp = 0;
    //   int files = 0, retval = 0;
    let mut first_post = true;

    //   bool stdin_used = false;
    let mut stdin_used = false;

    //   for( unsigned i = 0; i < filenames.size(); ++i )

    for filename in filenames {
        let from_stdin = filename == "-";
        //     const bool from_stdin = ( filenames[i] == "-" );
        if from_stdin {
            if stdin_used {
                continue;
            } else {
                stdin_used = true;
            }
        }
        //     if( from_stdin ) { if( stdin_used ) continue; else stdin_used = true; }
        //     const char * const input_filename =
        //       from_stdin ? "(stdin)" : filenames[i].c_str();
        let input_filename = if from_stdin {
            "(stdin)"
        } else {
            filename.as_str()
        };
        //     struct stat in_stats;				// not used
        //     const int infd = from_stdin ? STDIN_FILENO :
        //       open_instream( input_filename, &in_stats, false, true );
        //     if( infd < 0 ) { set_retval( retval, 1 ); continue; }

        //     const Lzip_index lzip_index( infd, cl_opts );
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
            let mut infd = BufReader::new(infd);
            LzipIndex::from_reader(&mut infd, opts)
        };
        //     close( infd );

        match lzip_index {
            Err(err) => {
                eprintln!("{err:?}");
                todo!();
            }
            Ok(lzip_index) => {
                //     if( lzip_index.retval() != 0 )
                //       {
                //       show_file_error( input_filename, lzip_index.error().c_str() );
                //       set_retval( retval, lzip_index.retval() );
                //       continue;
                //       }
                //     if( verbosity < 0 ) continue;
                let udata_size = lzip_index.udata_size();
                let cdata_size = lzip_index.cdata_size();
                total_comp += cdata_size;
                total_uncomp += udata_size;
                num_files += 1;
                //     const long members = lzip_index.members();
                if first_post {
                    first_post = false;
                    //       if( verbosity >= 1 ) std::fputs( "   dict   memb  trail ", stdout );
                    if verbosity >= 1 {
                        print!("   dict   memb  trail ");
                    }
                    //
                    //       std::fputs( "  uncompressed     compressed   saved  name\n", stdout );
                    println!("  uncompressed     compressed   saved  name");
                }
                //     if( verbosity >= 1 )
                //       std::printf( "%s %5ld %6lld ", format_ds( lzip_index.dictionary_size() ),
                //                    members, lzip_index.file_size() - cdata_size );
                if verbosity >= 1 {
                    print!(
                        "{:>8} {:>5} {:>6} ",
                        format_ds(lzip_index.dictionary_size()),
                        lzip_index.num_members(),
                        lzip_index.file_size() - cdata_size
                    );
                }
                //     list_line( udata_size, cdata_size, input_filename );
                list_line(udata_size, cdata_size, input_filename);

                //     if( verbosity >= 2 && members > 1 )
                //       {
                //       std::fputs( " member      data_pos      data_size     member_pos    member_size\n", stdout );
                //       for( long i = 0; i < members; ++i )
                //         {
                //         const Block & db = lzip_index.dblock( i );
                //         const Block & mb = lzip_index.mblock( i );
                //         std::printf( "%6ld %14llu %14llu %14llu %14llu\n",
                //                      i + 1, db.pos(), db.size(), mb.pos(), mb.size() );
                //         }
                //       first_post = true;	// reprint heading after list of members
                //       }
                //     std::fflush( stdout );
                if verbosity >= 2 && lzip_index.num_members() > 1 {
                    println!(" member      data_pos      data_size     member_pos    member_size");
                    for i in 0..lzip_index.num_members() {
                        let db = lzip_index.dblock(i);
                        let mb = lzip_index.mblock(i);
                        println!(
                            "{:>6} {:>14} {:>14} {:>14} {:>14}",
                            i + 1,
                            db.pos(),
                            db.size(),
                            mb.pos(),
                            mb.size()
                        );
                    }
                }
            }
        }
    }
    //   if( verbosity >= 0 && files > 1 )
    //     {
    //     if( verbosity >= 1 ) std::fputs( "                      ", stdout );
    //     list_line( total_uncomp, total_comp, "(totals)" );
    //     std::fflush( stdout );
    //     }
    //   return retval;
    Ok(())
}

fn list_line(uncomp_size: u64, comp_size: u64, input_filename: &str) {
    println!(
        "{:>14} {:>14} {:6.2}%  {}",
        uncomp_size,
        comp_size,
        100.0 - 100.0 * comp_size as f64 / uncomp_size as f64,
        input_filename
    );
}
// pub fn list_files2<R: io::BufRead + io::Seek>(
//     input: &mut R,
//     opts: &LzipIndexOptions,
// ) -> Result<(), LzipError> {
//     let index = LzipIndex::from_reader(input, opts)?;
//     let udata_size = index.udata_size();
//     let cdata_size = index.cdata_size();
//     todo!()
// }
