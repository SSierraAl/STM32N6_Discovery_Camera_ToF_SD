"""UTC metadata in the existing 64-byte raw SD header (no filesystem)."""
from datetime import datetime, timezone
import struct

RTC_TAG = 0x31435452

def parse_header(data):
    if len(data) < 64:
        return None
    fields = struct.unpack_from('<11I', data)
    names = ('magic', 'width', 'height', 'pixel_format', 'data_size',
             'timestamp', 'checksum', 'snap_id', 'time_tag', 'time_flags', 'capture_tick')
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
        return dt.strftime('%Y-%m-%d %H:%M:%SZ') + ' (capture completion)'
    if header.get('time_tag') == RTC_TAG:
        return 'RTC unavailable'
    return f"Legacy uptime {header.get('timestamp', 0)} ms"

def image_filename(header, index, suffix=''):
    dt = utc_datetime(header)
    stamp = dt.strftime('%Y%m%d_%H%M%SZ') if dt else 'UNTIMED'
    # Index also disambiguates old records whose firmware always stored ID=0.
    tail = f'_{suffix}' if suffix else ''
    return f"IMG_{stamp}_{header.get('snap_id', 0):06d}_{index:04d}{tail}.png"
