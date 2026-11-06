"""UTC metadata in the existing 64-byte raw SD header (no filesystem)."""
from datetime import datetime, timezone
import struct

RTC_TAG = 0x31435452
IMAGE_KINDS = {
    0x314B0001: 'overview',
    0x314B0002: 'crop_native',
    0x314B0003: 'full_5mp_large',
    0x314B0004: 'full_5mp_multi',
    0x314B0005: 'overview_uncertain',
    0x314B0006: 'overview_multi',
    0x314B0007: 'overview_large',
}

def image_kind_label(header):
    return IMAGE_KINDS.get(header.get('image_kind'), '')

def parse_header(data):
    if len(data) < 64:
        return None
    fields = struct.unpack_from('<12I', data)
    names = ('magic', 'width', 'height', 'pixel_format', 'data_size',
             'timestamp', 'checksum', 'snap_id', 'time_tag', 'time_flags', 'capture_tick', 'image_kind')
    return dict(zip(names, fields))

def utc_datetime(header):
    if header.get('time_tag') != RTC_TAG or not header.get('time_flags', 0) & 1:
        return None
    seconds = header.get('timestamp', 0)
    if not 946684800 <= seconds <= 4102444799:
        return None
    return datetime.fromtimestamp(seconds, timezone.utc)

def time_label(header):
    dt = utc_datetime(header)
    if dt is not None:
        return (dt.strftime('%Y-%m-%d %H:%M:%SZ') +
                f" | frame tick {header.get('capture_tick', 0)} ms")
    if header.get('time_tag') == RTC_TAG:
        return 'RTC unavailable'
    return f"Legacy uptime {header.get('timestamp', 0)} ms"

def image_filename(header, index, suffix=''):
    dt = utc_datetime(header)
    stamp = dt.strftime('%Y%m%d_%H%M%SZ') if dt else 'UNTIMED'
    # snap_id uniquely separates burst frames even when DS3231 UTC is the same second.
    tail = ''.join(f'_{part}' for part in (image_kind_label(header), suffix) if part)
    return f"IMG_{stamp}_{header.get('snap_id', 0):06d}_{index:04d}{tail}.png"



def snapshot_list_label(header, index):
    """Describe a saved record in the same order as the viewer columns."""
    name = image_filename(header, index, 'original')
    resolution = f"{header['width']}x{header['height']}"
    category = image_kind_label(header) or 'legacy / unspecified'
    captured_at = utc_datetime(header)
    time = captured_at.strftime('%Y-%m-%d %H:%M:%SZ') if captured_at else time_label(header)
    return f"#{index + 1:03d} | {resolution} | {category} | {time} | {name}"
