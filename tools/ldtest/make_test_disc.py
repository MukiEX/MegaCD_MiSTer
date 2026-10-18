#!/usr/bin/env python3
# Builds the harness's test disc from an 8 bpp .dscmmi made by mmidsc
# (its AnalogVideo.dscv, 240 frames or more):
#   make_test_disc.py SOURCE.dscmmi OUT.dscmmi
# Digital track: track 1 data (150 sectors), track 2 audio (450 sectors,
# +1000/-1000). Analog audio: left = (ABA & 0x3FF) * 8 in every sample of a
# sector, right = 0x2000, after the lead-in samples. Video: lead-in 20,
# active 200, lead-out 20 frames; a picture stop code on frame no. 151.
import sys, zipfile, struct, json
src, out = sys.argv[1], sys.argv[2]
z = zipfile.ZipFile(src)
name = [n for n in z.namelist() if n.endswith('.dscv')][0]
dscv = bytearray(z.read(name))
vbi_off = struct.unpack_from('<Q', dscv, 56)[0]; vsz = struct.unpack_from('<I', dscv, 64)[0]
struct.pack_into('<I', dscv, vbi_off + (20 + 150) * vsz, 0x82CFFF)
data = bytearray()
for s in range(150):
    sec = bytearray(2352); sec[0:12] = b'\x00' + b'\xff' * 10 + b'\x00'
    if s == 0: sec[16:30] = b'SEGADISCSYSTEM'
    data += sec
aud = struct.pack('<hh', 1000, -1000) * (588 * 450)
fps = 30 / 1.001
pcm = bytearray(struct.pack('<hh', 0, 0) * int(20 * (44100 / fps)))
for aba in range(850):
    pcm += struct.pack('<hh', (aba & 0x3FF) * 8, 0x2000) * 588
cue = 'FILE "DigitalAudio.img" BINARY\r\n  TRACK 01 MODE1/2352\r\n    INDEX 01 00:00:00\r\nFILE "Track02.bin" BINARY\r\n  TRACK 02 AUDIO\r\n    INDEX 01 00:00:00\r\n'
mi = {"name": "Harness Disc", "system": "MegaLD", "media": [{"name": "Side A", "type": "LD", "format": "CAV", "sideNo": 1, "streams": [
    {"name": "DigitalAudio", "type": "Redbook", "file": "Disc1Side1/DigitalAudio.cue"},
    {"name": "AnalogAudio", "type": "RawAudio", "file": "Disc1Side1/AnalogAudio.pcm"},
    {"name": "AnalogVideo", "type": "RawVideo", "file": "Disc1Side1/AnalogVideo.qon",
     "framesInLeadInRegion": 20, "framesInActiveRegion": 200, "framesInLeadOutRegion": 20}]}]}
o = zipfile.ZipFile(out, 'w', zipfile.ZIP_STORED, allowZip64=True)
o.writestr('MediaInfo.json', json.dumps(mi))
o.writestr('Disc1Side1/DigitalAudio.cue', cue)
o.writestr('Disc1Side1/DigitalAudio.img', bytes(data))
o.writestr('Disc1Side1/Track02.bin', aud)
o.writestr('Disc1Side1/AnalogAudio.pcm', bytes(pcm))
o.writestr('Disc1Side1/AnalogVideo.dscv', bytes(dscv))
dm = json.loads(z.read('DSCMMI.json')); dm['video'][0]['entry'] = 'Disc1Side1/AnalogVideo.dscv'
o.writestr('DSCMMI.json', json.dumps(dm))
o.close()
