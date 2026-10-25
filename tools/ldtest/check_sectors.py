#!/usr/bin/env python3
"""Checks a log_sectors run of MiSTer_LD against the disc image.

    python3 check_sectors.py laseractive.log "Road Blaster.dscmmi"

With `log_sectors 1` in laseractive.cfg, MiSTer_LD logs every data sector it
gives the core: its LBA, the header it sent, where the bytes came from and a
CRC32 of the 2048 user bytes. This reads the same sectors from the image and
reports any that differ, any header that doesn't match its LBA, read-ahead
misses, long gaps, and the last sectors before the log ends (where a game
that hangs was last fed).

Python 3.6+, nothing to install.
"""
import re, sys, zipfile, zlib

LINE = re.compile(r'^\s*([\d.]+)\s+sector: lba (-?\d+), header ([0-9A-F]{2}):([0-9A-F]{2}):([0-9A-F]{2}) mode ([0-9A-F]{2}), from (.+?), crc ([0-9A-F]{8}), (\d+) ms after the previous')


def bcd(s):
    v = int(s, 16)
    return (v >> 4) * 10 + (v & 15)


def data_tracks(z):
    """(cue name, member, stride, skip, first LBA offset) for each side's data track."""
    out = []
    for cue in [n for n in z.namelist() if n.lower().endswith('.cue')]:
        text = z.read(cue).decode('latin-1')
        folder = cue.rsplit('/', 1)[0] + '/' if '/' in cue else ''
        f = re.search(r'FILE\s+"([^"]+)"', text)
        t = re.search(r'TRACK\s+01\s+(MODE1/2048|MODE1/2352|MODE2/2352)', text)
        i = re.search(r'TRACK\s+01.*?INDEX\s+01\s+(\d+):(\d+):(\d+)', text, re.S)
        if not f or not t:
            continue
        stride = 2048 if t.group(1) == 'MODE1/2048' else 2352
        skip = 0 if stride == 2048 else (16 if t.group(1).startswith('MODE1') else 24)
        start = 0
        if i:
            start = (int(i.group(1)) * 60 + int(i.group(2))) * 75 + int(i.group(3))
        out.append((cue, folder + f.group(1), stride, skip, start))
    return out


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    log, image = sys.argv[1], sys.argv[2]
    rows = []
    for line in open(log, encoding='latin-1'):
        m = LINE.match(line)
        if m:
            t, lba, mm, ss, ff, mode, src, crc, gap = m.groups()
            rows.append((float(t), int(lba), bcd(mm), bcd(ss), bcd(ff), mode, src, int(crc, 16), int(gap)))
    if not rows:
        print('No "sector:" lines in %s. Is log_sectors 1 in laseractive.cfg?' % log)
        return 1
    print('%d data sectors in the log, %.3f s to %.3f s' % (len(rows), rows[0][0], rows[-1][0]))

    z = zipfile.ZipFile(image)
    tracks = data_tracks(z)
    if not tracks:
        print('No data track found in %s' % image)
        return 1

    best = None
    for cue, member, stride, skip, start in tracks:
        size = z.getinfo(member).file_size
        f = z.open(member)
        bad = []
        for r in rows:
            lba = r[1]
            if lba < 0:
                continue
            off = (lba + start) * stride + skip
            if off + 2048 > size:
                bad.append((r, None))
                continue
            f.seek(off)
            want = zlib.crc32(f.read(2048)) & 0xFFFFFFFF
            if want != r[7]:
                bad.append((r, want))
        if best is None or len(bad) < len(best[1]):
            best = ((cue, member, stride), bad)
    (cue, member, stride), bad = best
    print('data track: %s (%d-byte sectors)' % (member, stride))
    if bad:
        print('\n%d sector(s) DIFFER from the image:' % len(bad))
        for r, want in bad[:40]:
            print('  %8.3f  lba %6d  sent crc %08X, image %s  (from %s)' % (r[0], r[1], r[7], 'past the end' if want is None else '%08X' % want, r[6]))
    else:
        print('every sector sent matches the image')

    hdr = [r for r in rows if (r[2] * 60 + r[3]) * 75 + r[4] - 150 != r[1]]
    if hdr:
        print('\n%d header(s) don\'t match their LBA, e.g. lba %d sent as %02d:%02d:%02d' % (len(hdr), hdr[0][1], hdr[0][2], hdr[0][3], hdr[0][4]))
    misses = [r for r in rows if 'MISS' in r[6]]
    if misses:
        print('\n%d read-ahead miss(es) (read from the card on the spot), first at %.3f s, lba %d' % (len(misses), misses[0][0], misses[0][1]))

    # runs of consecutive sectors: the loads the game asked for
    runs = []
    for r in rows:
        if runs and r[1] == runs[-1][1] + 1 and r[8] < 100:
            runs[-1][1] = r[1]
            runs[-1][3] = r[0]
            runs[-1][4] = max(runs[-1][4], r[8])
        else:
            runs.append([r[1], r[1], r[0], r[0], 0])
    print('\n%d runs of consecutive sectors; the last 15:' % len(runs))
    print('      start s     end s    LBAs            sectors  longest gap inside')
    for a, b, t0, t1, g in runs[-15:]:
        print('  %10.3f  %8.3f   %6d .. %-6d  %4d     %d ms' % (t0, t1, a, b, b - a + 1, g))
    print('\nlast sector sent at %.3f s: lba %d' % (rows[-1][0], rows[-1][1]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
