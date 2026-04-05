use alloc::{boxed::Box, sync::Arc, vec, vec::Vec};
use core::cmp::min;
use core::str;

use libkernel::{
    error::{FsError, KernelError, Result},
    fs::{FileType, Inode, attr::FilePermissions, path::Path, pathbuf::PathBuf},
};
use log::warn;
use miniz_oxide::{
    DataFormat, MZFlush, MZStatus,
    inflate::stream::{InflateState, inflate},
};

const NEWC_MAGIC: &[u8; 6] = b"070701";
const CRC_MAGIC: &[u8; 6] = b"070702";
const STREAM_BUF_SIZE: usize = 64 * 1024;
const MAX_SYMLINK_SIZE: usize = 64 * 1024;

const S_IFMT: u32 = 0o170000;
const S_IFIFO: u32 = 0o010000;
const S_IFCHR: u32 = 0o020000;
const S_IFDIR: u32 = 0o040000;
const S_IFBLK: u32 = 0o060000;
const S_IFREG: u32 = 0o100000;
const S_IFLNK: u32 = 0o120000;
const S_IFSOCK: u32 = 0o140000;

struct CpioHeader {
    mode: u32,
    filesize: usize,
    namesize: usize,
}

fn align4(value: usize) -> usize {
    (value + 3) & !3
}

fn parse_hex_u32(input: &[u8]) -> Result<u32> {
    let s = str::from_utf8(input).map_err(|_| KernelError::InvalidValue)?;
    u32::from_str_radix(s, 16).map_err(|_| KernelError::InvalidValue)
}

fn parse_header(input: &[u8]) -> Result<CpioHeader> {
    if input.len() < 110 {
        return Err(KernelError::InvalidValue);
    }

    if &input[..6] != NEWC_MAGIC && &input[..6] != CRC_MAGIC {
        return Err(KernelError::InvalidValue);
    }

    Ok(CpioHeader {
        mode: parse_hex_u32(&input[14..22])?,
        filesize: parse_hex_u32(&input[54..62])? as usize,
        namesize: parse_hex_u32(&input[94..102])? as usize,
    })
}

fn normalize_member_path(name: &str) -> Result<Option<PathBuf>> {
    let trimmed = name.trim_start_matches('/').trim_start_matches("./");

    if trimmed.is_empty() || trimmed == "." {
        return Ok(None);
    }

    if trimmed.split('/').any(|component| component == "..") {
        return Err(KernelError::InvalidValue);
    }

    Ok(Some(PathBuf::from(trimmed)))
}

fn permissions_from_mode(mode: u32) -> FilePermissions {
    FilePermissions::from_bits_truncate((mode & 0o7777) as u16)
}

async fn set_permissions(inode: &Arc<dyn Inode>, perms: FilePermissions) -> Result<()> {
    let mut attr = inode.getattr().await?;
    attr.permissions = perms;
    inode.setattr(attr).await
}

async fn ensure_dir(
    root: Arc<dyn Inode>,
    path: &Path,
    perms: FilePermissions,
) -> Result<Arc<dyn Inode>> {
    let mut current = root;

    for component in path.components() {
        match current.lookup(component).await {
            Ok(next) => current = next,
            Err(KernelError::Fs(FsError::NotFound)) => {
                current = current
                    .create(component, FileType::Directory, perms, None)
                    .await?;
            }
            Err(e) => return Err(e),
        }
    }

    set_permissions(&current, perms).await?;

    Ok(current)
}

async fn create_symlink(root: Arc<dyn Inode>, path: &Path, target: &Path) -> Result<()> {
    let parent = if let Some(parent) = path.parent() {
        ensure_dir(root, parent, FilePermissions::from_bits_truncate(0o755)).await?
    } else {
        root
    };

    let name = path.file_name().ok_or(KernelError::InvalidValue)?;
    parent.symlink(name, target).await
}

trait ByteStream {
    fn read(&mut self, out: &mut [u8]) -> Result<usize>;
}

struct SliceStream<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> SliceStream<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }
}

impl ByteStream for SliceStream<'_> {
    fn read(&mut self, out: &mut [u8]) -> Result<usize> {
        let available = self.input.len().saturating_sub(self.offset);
        let count = min(available, out.len());
        out[..count].copy_from_slice(&self.input[self.offset..self.offset + count]);
        self.offset += count;
        Ok(count)
    }
}

struct GzipStream<'a> {
    compressed: &'a [u8],
    offset: usize,
    state: Box<InflateState>,
    finished: bool,
}

impl<'a> GzipStream<'a> {
    fn new(input: &'a [u8]) -> Result<Self> {
        if input.len() < 18 || input[0] != 0x1f || input[1] != 0x8b || input[2] != 8 {
            return Err(KernelError::InvalidValue);
        }

        let flags = input[3];
        let mut offset = 10usize;

        if flags & 0x04 != 0 {
            if input.len() < offset + 2 {
                return Err(KernelError::InvalidValue);
            }
            let xlen = u16::from_le_bytes([input[offset], input[offset + 1]]) as usize;
            offset += 2 + xlen;
        }

        if flags & 0x08 != 0 {
            while offset < input.len() && input[offset] != 0 {
                offset += 1;
            }
            offset += 1;
        }

        if flags & 0x10 != 0 {
            while offset < input.len() && input[offset] != 0 {
                offset += 1;
            }
            offset += 1;
        }

        if flags & 0x02 != 0 {
            offset += 2;
        }

        if offset >= input.len().saturating_sub(8) {
            return Err(KernelError::InvalidValue);
        }

        Ok(Self {
            compressed: &input[..input.len() - 8],
            offset,
            state: InflateState::new_boxed(DataFormat::Raw),
            finished: false,
        })
    }
}

impl ByteStream for GzipStream<'_> {
    fn read(&mut self, out: &mut [u8]) -> Result<usize> {
        if out.is_empty() || self.finished {
            return Ok(0);
        }

        loop {
            let flush = if self.offset == self.compressed.len() {
                MZFlush::Finish
            } else {
                MZFlush::None
            };

            let result = inflate(&mut self.state, &self.compressed[self.offset..], out, flush);
            self.offset += result.bytes_consumed;

            match result.status {
                Ok(MZStatus::Ok) => {
                    if result.bytes_written != 0 {
                        return Ok(result.bytes_written);
                    }

                    if self.offset == self.compressed.len() {
                        return Err(KernelError::InvalidValue);
                    }
                }
                Ok(MZStatus::StreamEnd) => {
                    self.finished = true;
                    return Ok(result.bytes_written);
                }
                Ok(MZStatus::NeedDict) => return Err(KernelError::InvalidValue),
                Err(_) => return Err(KernelError::InvalidValue),
            }
        }
    }
}

struct Reader<S: ByteStream> {
    stream: S,
    buf: Vec<u8>,
    start: usize,
    end: usize,
}

impl<S: ByteStream> Reader<S> {
    fn new(stream: S) -> Self {
        Self {
            stream,
            buf: vec![0; STREAM_BUF_SIZE],
            start: 0,
            end: 0,
        }
    }

    fn available(&self) -> usize {
        self.end - self.start
    }

    fn refill(&mut self) -> Result<usize> {
        if self.start != 0 && self.start != self.end {
            self.buf.copy_within(self.start..self.end, 0);
            self.end -= self.start;
            self.start = 0;
        } else if self.start == self.end {
            self.start = 0;
            self.end = 0;
        }

        let count = self.stream.read(&mut self.buf[self.end..])?;
        self.end += count;
        Ok(count)
    }

    fn read_exact(&mut self, out: &mut [u8]) -> Result<()> {
        let mut filled = 0;

        while filled < out.len() {
            if self.available() == 0 && self.refill()? == 0 {
                return Err(KernelError::InvalidValue);
            }

            let copy = min(self.available(), out.len() - filled);
            out[filled..filled + copy].copy_from_slice(&self.buf[self.start..self.start + copy]);
            self.start += copy;
            filled += copy;
        }

        Ok(())
    }

    fn skip_exact(&mut self, mut len: usize) -> Result<()> {
        while len != 0 {
            if self.available() == 0 && self.refill()? == 0 {
                return Err(KernelError::InvalidValue);
            }

            let step = min(self.available(), len);
            self.start += step;
            len -= step;
        }

        Ok(())
    }
}

async fn populate_root_from_stream<S: ByteStream>(
    root: Arc<dyn Inode>,
    mut reader: Reader<S>,
) -> Result<()> {
    let mut header_buf = [0u8; 110];
    let mut data_buf = vec![0u8; STREAM_BUF_SIZE];

    loop {
        reader.read_exact(&mut header_buf)?;
        let header = parse_header(&header_buf)?;

        if header.namesize == 0 {
            return Err(KernelError::InvalidValue);
        }

        let mut name_buf = vec![0u8; header.namesize];
        reader.read_exact(&mut name_buf)?;
        let name = str::from_utf8(&name_buf[..header.namesize - 1])
            .map_err(|_| KernelError::InvalidValue)?;

        let header_and_name = 110 + header.namesize;
        reader.skip_exact(align4(header_and_name) - header_and_name)?;

        if name == "TRAILER!!!" {
            break;
        }

        let data_padded = align4(header.filesize);
        let Some(path) = normalize_member_path(name)? else {
            reader.skip_exact(data_padded)?;
            continue;
        };

        let perms = permissions_from_mode(header.mode);

        match header.mode & S_IFMT {
            S_IFDIR => {
                ensure_dir(root.clone(), path.as_path(), perms).await?;
                reader.skip_exact(data_padded)?;
            }
            S_IFREG => {
                let parent = if let Some(parent) = path.parent() {
                    ensure_dir(
                        root.clone(),
                        parent,
                        FilePermissions::from_bits_truncate(0o755),
                    )
                    .await?
                } else {
                    root.clone()
                };

                let name = path.file_name().ok_or(KernelError::InvalidValue)?;
                let inode = parent.create(name, FileType::File, perms, None).await?;

                let mut remaining = header.filesize;
                let mut offset = 0u64;
                while remaining != 0 {
                    let chunk = min(remaining, data_buf.len());
                    reader.read_exact(&mut data_buf[..chunk])?;
                    let mut written = 0usize;
                    while written < chunk {
                        let n = inode
                            .write_at(offset + written as u64, &data_buf[written..chunk])
                            .await?;
                        if n == 0 {
                            return Err(KernelError::Other("short initramfs write"));
                        }
                        written += n;
                    }
                    offset += chunk as u64;
                    remaining -= chunk;
                }

                reader.skip_exact(data_padded - header.filesize)?;
            }
            S_IFLNK => {
                if header.filesize > MAX_SYMLINK_SIZE {
                    return Err(KernelError::TooLarge);
                }

                let mut target = vec![0u8; header.filesize];
                reader.read_exact(&mut target)?;
                let target = str::from_utf8(&target).map_err(|_| KernelError::InvalidValue)?;
                create_symlink(root.clone(), path.as_path(), Path::new(target)).await?;
                reader.skip_exact(data_padded - header.filesize)?;
            }
            S_IFCHR | S_IFBLK | S_IFIFO | S_IFSOCK => {
                warn!(
                    "Skipping unsupported initramfs special file {}",
                    path.as_str()
                );
                reader.skip_exact(data_padded)?;
            }
            _ => return Err(KernelError::InvalidValue),
        }
    }

    Ok(())
}

pub async fn populate_root(root: Arc<dyn Inode>, initrd: &[u8]) -> Result<()> {
    if initrd.starts_with(NEWC_MAGIC) || initrd.starts_with(CRC_MAGIC) {
        return populate_root_from_stream(root, Reader::new(SliceStream::new(initrd))).await;
    }

    if initrd.starts_with(&[0x1f, 0x8b, 0x08]) {
        return populate_root_from_stream(root, Reader::new(GzipStream::new(initrd)?)).await;
    }

    Err(KernelError::NotSupported)
}
