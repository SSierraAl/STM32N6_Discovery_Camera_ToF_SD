"""Manage the persistent append journal used by STM32 raw-image storage.

Run as Administrator on Windows because PhysicalDrive access is required.

Examples:
  python SD_Storage_Journal.py --drive 2 --show
  python SD_Storage_Journal.py --drive 2 --repair --scan-mb 4096
  python SD_Storage_Journal.py --drive 2 --repair --scan-mb 4096 --reclaim
  python SD_Storage_Journal.py --drive 2 --reset --force

Normal "Delete Selected" in SD_Image_Viewer.py does NOT need to alter the
journal: deleted holes stay unused and new photos keep appending safely.
Use --reclaim only when you intentionally want to move the append pointer back
to the end of the highest surviving image. Use --reset only after deleting all
images / formatting the raw photo area.
"""

import argparse
import os
import struct
import sys
import zlib

BLOCK_SIZE = 512
HEADER_SIZE = 64
HEADER_TAG = 0x49444745
SNAP_BASE = 3072
JOURNAL_BLOCK_A = SNAP_BASE - 2
JOURNAL_BLOCK_B = SNAP_BASE - 1
JOURNAL_MAGIC = 0x314A4453  # "SDJ1" little endian
JOURNAL_VERSION = 1
JOURNAL_FMT_NOCRC = '<6I'
JOURNAL_FMT = '<7I'
JOURNAL_SIZE = struct.calcsize(JOURNAL_FMT)


def physical_path(value):
    value = str(value).strip()
    if value.startswith('\\\\.\\PhysicalDrive'):
        return value
    if value.isdigit():
        return f'\\\\.\\PhysicalDrive{value}'
    raise ValueError("--drive must be a disk number (e.g. 2) or \\\\.\\PhysicalDriveN")


def _open(path, write=False):
    flags = (os.O_RDWR if write else os.O_RDONLY) | getattr(os, 'O_BINARY', 0)
    return os.open(path, flags)


def _read_block_fd(fd, block):
    os.lseek(fd, int(block) * BLOCK_SIZE, os.SEEK_SET)
    return os.read(fd, BLOCK_SIZE)


def _write_block_fd(fd, block, data):
    if len(data) != BLOCK_SIZE:
        raise ValueError("journal write must be exactly one 512-byte block")
    os.lseek(fd, int(block) * BLOCK_SIZE, os.SEEK_SET)
    n = os.write(fd, data)
    if n != BLOCK_SIZE:
        raise OSError(f"short journal write: {n}/{BLOCK_SIZE} bytes")
    try:
        os.fsync(fd)
    except OSError:
        pass


def _crc(payload):
    return zlib.crc32(payload) & 0xFFFFFFFF


def _decode_record(raw, slot):
    if raw is None or len(raw) < JOURNAL_SIZE:
        return None
    vals = struct.unpack_from(JOURNAL_FMT, raw, 0)
    rec = {
        'magic': vals[0], 'version': vals[1], 'sequence': vals[2],
        'next_block': vals[3], 'next_snap_id': vals[4],
        'snap_base': vals[5], 'crc32': vals[6], 'slot': slot,
    }
    payload = struct.pack(JOURNAL_FMT_NOCRC, *vals[:6])
    if rec['magic'] != JOURNAL_MAGIC or rec['version'] != JOURNAL_VERSION:
        return None
    if rec['snap_base'] != SNAP_BASE or rec['next_block'] < SNAP_BASE:
        return None
    if rec['crc32'] != _crc(payload):
        return None
    return rec


def _newer(a, b):
    """Sequence comparison matching the firmware's signed uint32 delta."""
    delta = (a['sequence'] - b['sequence']) & 0xFFFFFFFF
    return a if delta != 0 and delta < 0x80000000 else b


def read_journal(path):
    fd = _open(path, False)
    try:
        a = _decode_record(_read_block_fd(fd, JOURNAL_BLOCK_A), 0)
        b = _decode_record(_read_block_fd(fd, JOURNAL_BLOCK_B), 1)
    finally:
        os.close(fd)
    if a and b:
        return _newer(a, b)
    return a or b


def _pack_record(sequence, next_block, next_snap_id):
    values = (JOURNAL_MAGIC, JOURNAL_VERSION, sequence & 0xFFFFFFFF,
              int(next_block), int(next_snap_id), SNAP_BASE)
    payload = struct.pack(JOURNAL_FMT_NOCRC, *values)
    raw = payload + struct.pack('<I', _crc(payload))
    return raw + b'\x00' * (BLOCK_SIZE - len(raw))


def write_journal(path, next_block, next_snap_id, duplicate_if_new=True):
    if next_block < SNAP_BASE:
        raise ValueError(f"next_block must be >= {SNAP_BASE}")
    if next_snap_id < 0:
        raise ValueError("next_snap_id must be >= 0")

    current = read_journal(path)
    sequence = ((current['sequence'] + 1) & 0xFFFFFFFF) if current else 1
    slot = (1 - current['slot']) if current else 0
    block = JOURNAL_BLOCK_B if slot else JOURNAL_BLOCK_A
    raw = _pack_record(sequence, next_block, next_snap_id)

    fd = _open(path, True)
    try:
        _write_block_fd(fd, block, raw)
        verify = _decode_record(_read_block_fd(fd, block), slot)
        if not verify or verify['sequence'] != sequence or \
                verify['next_block'] != next_block or verify['next_snap_id'] != next_snap_id:
            raise OSError("journal read-back verification failed")
    finally:
        os.close(fd)

    # A brand-new card gets both A and B populated immediately so one torn
    # sector can still be recovered. Normal updates alternate one sector only.
    if current is None and duplicate_if_new:
        return write_journal(path, next_block, next_snap_id, duplicate_if_new=False)
    return read_journal(path)


def parse_image_header(raw):
    if raw is None or len(raw) < HEADER_SIZE:
        return None
    vals = struct.unpack_from('<11I', raw, 0)
    magic, width, height, pixel_format, data_size = vals[:5]
    if magic != HEADER_TAG or width == 0 or height == 0 or width > 4096 or height > 4096:
        return None
    bpp = 1 if pixel_format == 2 else 2
    expected = width * height * bpp
    if data_size != expected:
        return None
    blocks = (HEADER_SIZE + data_size + BLOCK_SIZE - 1) // BLOCK_SIZE
    return {'width': width, 'height': height, 'pixel_format': pixel_format,
            'data_size': data_size, 'snap_id': vals[7], 'blocks': blocks}


def scan_images(path, scan_mb, progress=True):
    """Layout-agnostic scan at every 512-byte boundary within scan_mb.

    This intentionally tolerates holes from Delete Selected and mixed Mode 0 /
    Mode 4 resolutions. Only the 64-byte header is validated; image payloads are
    skipped after a valid hit.
    """
    chunk_mb = 8
    chunk_bytes = chunk_mb * 1024 * 1024
    limit_bytes = SNAP_BASE * BLOCK_SIZE + int(scan_mb) * 1024 * 1024
    magic = struct.pack('<I', HEADER_TAG)
    images = []

    fd = _open(path, False)
    try:
        offset = SNAP_BASE * BLOCK_SIZE
        while offset < limit_bytes:
            to_read = min(chunk_bytes, limit_bytes - offset)
            os.lseek(fd, offset, os.SEEK_SET)
            data = os.read(fd, to_read)
            if not data:
                break
            usable = len(data) - (len(data) % BLOCK_SIZE)
            if usable <= 0:
                break

            pos = 0
            while pos <= usable - HEADER_SIZE:
                hit = data.find(magic, pos)
                if hit < 0:
                    break
                if hit % BLOCK_SIZE == 0:
                    h = parse_image_header(data[hit:hit + HEADER_SIZE])
                    if h:
                        block = (offset + hit) // BLOCK_SIZE
                        h['block'] = block
                        images.append(h)
                        pos = hit + h['blocks'] * BLOCK_SIZE
                        continue
                pos = hit + 1

            offset += usable
            if progress:
                done_mb = max(0, (offset - SNAP_BASE * BLOCK_SIZE) // (1024 * 1024))
                print(f"\rScanning raw image area: {done_mb}/{scan_mb} MB, "
                      f"found {len(images)} image(s)", end='', flush=True)
            if len(data) < to_read:
                break
    finally:
        os.close(fd)
    if progress:
        print()
    return images


def repair(path, scan_mb, reclaim=False):
    current = read_journal(path)
    images = scan_images(path, scan_mb)

    if images:
        highest_end = max(i['block'] + i['blocks'] for i in images)
        highest_id = max(i['snap_id'] for i in images) + 1
    else:
        highest_end = SNAP_BASE
        highest_id = 0

    if current and not reclaim:
        # Default repair NEVER moves backwards: a limited scan or a deleted
        # trailing photo cannot make an old block writable again by accident.
        next_block = max(current['next_block'], highest_end)
        next_id = max(current['next_snap_id'], highest_id)
    else:
        next_block = highest_end
        next_id = highest_id

    if not current:
        print("WARNING: this card had no valid journal. Ensure --scan-mb covered "
              "the entire region where legacy photos may exist before reconnecting it to the board.")
    if reclaim:
        print("RECLAIM enabled: append pointer may move backwards to the highest surviving image.")

    rec = write_journal(path, next_block, next_id)
    return rec, images


def show(rec):
    if not rec:
        print("No valid SD append journal found.")
        print(f"Expected redundant sectors: blocks {JOURNAL_BLOCK_A} and {JOURNAL_BLOCK_B}")
        return
    slot = 'A' if rec['slot'] == 0 else 'B'
    print(f"Journal slot : {slot}")
    print(f"Sequence     : {rec['sequence']}")
    print(f"Next block   : {rec['next_block']}")
    print(f"Next snap ID : {rec['next_snap_id']}")
    print(f"Image base   : {rec['snap_base']}")


def main():
    ap = argparse.ArgumentParser(description="STM32 raw-SD append journal utility")
    ap.add_argument('--drive', required=True,
                    help="PhysicalDrive number, e.g. 2, or \\\\.\\PhysicalDrive2")
    op = ap.add_mutually_exclusive_group(required=True)
    op.add_argument('--show', action='store_true', help="show current journal")
    op.add_argument('--repair', action='store_true', help="scan images and create/repair journal")
    op.add_argument('--reset', action='store_true', help="set next block=3072 and next ID=0")
    op.add_argument('--set', action='store_true', help="manually set next block and ID")
    ap.add_argument('--scan-mb', type=int, default=4096,
                    help="MB of raw image area to inspect with --repair (default 4096)")
    ap.add_argument('--reclaim', action='store_true',
                    help="allow --repair to move pointer backwards after deleting trailing images")
    ap.add_argument('--next-block', type=int)
    ap.add_argument('--next-id', type=int)
    ap.add_argument('--force', action='store_true',
                    help="required for destructive --reset or manual --set")
    args = ap.parse_args()

    if os.name != 'nt':
        print("This tool is intended for Windows PhysicalDrive access.", file=sys.stderr)
        return 2

    try:
        path = physical_path(args.drive)
        if args.show:
            show(read_journal(path))
            return 0
        if args.repair:
            rec, images = repair(path, args.scan_mb, args.reclaim)
            print(f"Found {len(images)} valid image header(s).")
            show(rec)
            return 0
        if args.reset:
            if not args.force:
                raise RuntimeError("--reset requires --force; it allows the board to write again from block 3072")
            rec = write_journal(path, SNAP_BASE, 0)
            show(rec)
            return 0
        if args.set:
            if not args.force or args.next_block is None or args.next_id is None:
                raise RuntimeError("--set requires --force --next-block N --next-id N")
            rec = write_journal(path, args.next_block, args.next_id)
            show(rec)
            return 0
    except PermissionError:
        print("Permission denied. Run the terminal as Administrator.", file=sys.stderr)
        return 3
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
