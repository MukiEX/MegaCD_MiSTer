// engine.js - the converter's logic, shared by the browser page (web/
// mmidsc.html), its workers, and the Node test (web/test_node.mjs).
// It is a port of src/main.c's convert command and src/mmi.c / src/zipw.c;
// the per-frame work (QON decode, VBI, DSC encode, CRC) runs in
// mmidsc.wasm, built from the same C sources. The output is byte for byte
// what `mmidsc convert` writes (apart from the "tool" line in DSCMMI.json).
//
// Everything is plain functions on globalThis.MMIDSC so the page can inline
// this file into itself and into its workers.
// SPDX-License-Identifier: MIT
(function (G) {
	'use strict';

	const TOOL = 'mmidsc-web 1.1';
	const DSCV_HEADER_SIZE = 512, DSCV_VBI_ENTRY = 16;

	// ------------------------------------------------------------ bytes
	const rd16 = (b, o) => b[o] | (b[o + 1] << 8);
	const rd32 = (b, o) => (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0;
	const rd64 = (b, o) => rd32(b, o) + rd32(b, o + 4) * 4294967296;   // exact below 2^53
	function wr16(b, o, v) { b[o] = v & 255; b[o + 1] = (v >>> 8) & 255; }
	function wr32(b, o, v) { b[o] = v & 255; b[o + 1] = (v >>> 8) & 255; b[o + 2] = (v >>> 16) & 255; b[o + 3] = (v >>> 24) & 255; }
	function wr64(b, o, v) { wr32(b, o, v % 4294967296); wr32(b, o + 4, Math.floor(v / 4294967296)); }
	const utf8 = new TextEncoder();
	const fromUtf8 = new TextDecoder();

	// ---------------------------------------------------- zip (src/mmi.c)
	// read(offset, length) -> Promise<Uint8Array>
	async function zipOpen(read, size) {
		const tail = Math.min(size, 65557);
		const buf = await read(size - tail, tail);
		let eocd = -1;
		for (let i = tail - 22; i >= 0; i--) if (rd32(buf, i) === 0x06054b50) { eocd = i; break; }
		if (eocd < 0) throw new Error('This is not an .mmi file (it isn’t a ZIP archive).');
		let entries = rd16(buf, eocd + 10), cdSize = rd32(buf, eocd + 12), cdOff = rd32(buf, eocd + 16);
		const eocdAbs = size - tail + eocd;
		if (eocdAbs >= 20) {
			const loc = await read(eocdAbs - 20, 20);
			if (rd32(loc, 0) === 0x07064b50) {
				const rec = await read(rd64(loc, 8), 56);
				if (rd32(rec, 0) === 0x06064b50) {
					entries = rd64(rec, 32); cdSize = rd64(rec, 40); cdOff = rd64(rec, 48);
				}
			}
		}
		if (cdOff + cdSize > size || entries > 65536) throw new Error('The file’s ZIP directory is damaged.');
		const cd = await read(cdOff, cdSize);
		const list = [];
		let p = 0;
		for (let k = 0; k < entries; k++) {
			if (p + 46 > cdSize || rd32(cd, p) !== 0x02014b50) break;
			const nlen = rd16(cd, p + 28), xlen = rd16(cd, p + 30), clen = rd16(cd, p + 32);
			if (p + 46 + nlen + xlen + clen > cdSize) break;
			const e = {
				flags: rd16(cd, p + 8), method: rd16(cd, p + 10), mtime: rd16(cd, p + 12), mdate: rd16(cd, p + 14),
				crc: rd32(cd, p + 16), compSize: rd32(cd, p + 20), uncompSize: rd32(cd, p + 24), localOff: rd32(cd, p + 42),
				nameBytes: cd.slice(p + 46, p + 46 + Math.min(nlen, 511)),
			};
			e.name = fromUtf8.decode(e.nameBytes);
			let x = p + 46 + nlen;
			const xe = x + xlen;
			while (x + 4 <= xe) {
				const id = rd16(cd, x), sz = rd16(cd, x + 2), d = x + 4;
				if (d + sz > xe) break;
				if (id === 0x0001) {
					let q = d;
					if (e.uncompSize === 0xffffffff && q + 8 <= d + sz) { e.uncompSize = rd64(cd, q); q += 8; }
					if (e.compSize === 0xffffffff && q + 8 <= d + sz) { e.compSize = rd64(cd, q); q += 8; }
					if (e.localOff === 0xffffffff && q + 8 <= d + sz) { e.localOff = rd64(cd, q); q += 8; }
				}
				x = d + sz;
			}
			const lh = await read(e.localOff, 30);
			if (rd32(lh, 0) === 0x04034b50) {
				e.dataOff = e.localOff + 30 + rd16(lh, 26) + rd16(lh, 28);
				list.push(e);
			}
			p += 46 + nlen + xlen + clen;
		}
		return list;
	}

	async function isQon(read, e) {
		if (e.method !== 0 || e.compSize < 24) return false;
		const m = await read(e.dataOff, 4);
		return m[0] === 0x71 && m[1] === 0x6f && m[2] === 0x6e && m[3] === 0x31;   // "qon1"
	}

	// QON header and frame index. off[i] = absolute offset of frame i's
	// 4-byte size prefix; off[fc] = end of the last frame.
	async function qonOpen(read, e) {
		const h = await read(e.dataOff, 24);
		if (rd32(h, 0) !== 0x316e6f71) throw new Error('Bad QON header in ' + e.name + '.');
		const hdr = {
			width: rd32(h, 4), height: rd32(h, 8), channels: h[12], colorspace: h[13], flags: rd16(h, 14),
			frameCount: rd32(h, 16), frameDuration: rd32(h, 20),
		};
		if (hdr.flags & 1) throw new Error(e.name + ' uses inter-frame compression, which ares doesn’t support either.');
		if (hdr.channels !== 3 && hdr.channels !== 4) throw new Error(e.name + ' has an unexpected channel count.');
		const fc = hdr.frameCount;
		const idx = await read(e.dataOff + 24, fc * 8);
		const dataBase = e.dataOff + 24 + fc * 8;
		const off = new Float64Array(fc + 1);
		for (let i = 0; i < fc; i++) off[i] = dataBase + rd32(idx, i * 8) + (rd16(idx, i * 8 + 4)) * 4294967296;
		const last = await read(off[fc - 1], 4);
		off[fc] = off[fc - 1] + 4 + rd32(last, 0);
		let maxFrame = 0;
		for (let i = 0; i < fc; i++) maxFrame = Math.max(maxFrame, off[i + 1] - off[i]);
		return { hdr, off, end: e.dataOff + e.compSize, maxFrame };
	}

	const fpsOf = hdr => hdr.frameDuration ? 1e6 / hdr.frameDuration : 30000 / 1001;

	// ---------------------------------------------- wasm frame coder
	// One instance per worker (or one in the Node test).
	async function makeCoder(module) {
		const inst = await WebAssembly.instantiate(module, {});
		const X = inst.exports;
		const mem = () => new Uint8Array(X.memory.buffer);
		const u32 = () => new Uint32Array(X.memory.buffer);
		return {
			// returns the stream's geometry and DSC settings
			setup(hdr, bpp16, noIch, maxIn) {
				if (X.setup(hdr.width, hdr.height, hdr.channels, hdr.colorspace, bpp16, noIch ? 1 : 0, maxIn) !== 0) {
					const m = mem(), p = X.err_ptr();
					let s = '';
					for (let i = p; m[i]; i++) s += String.fromCharCode(m[i]);
					throw new Error('DSC setup: ' + s);
				}
				const ip = X.info_ptr() >>> 2, I = u32();
				return {
					cropX: I[ip], width: I[ip + 1], height: I[ip + 2], f0: I[ip + 3], f0n: I[ip + 4], f1: I[ip + 5], f1n: I[ip + 6],
					frameBytes: I[ip + 7], stride: I[ip + 8], slices: I[ip + 9],
					pps: mem().slice(X.info_ptr() + 128, X.info_ptr() + 256),
				};
			},
			// frame: the compressed frame without its size prefix, or null if unreadable
			encode(frame) {
				let len = -1;
				if (frame) { mem().set(frame, X.in_ptr()); len = frame.length; }
				const flags = X.encode_frame(len);
				const ip = X.info_ptr() >>> 2, I = u32();
				const stride = I[ip + 8];
				return {
					flags, crc: I[ip + 14], vbi: [I[ip + 10], I[ip + 11], I[ip + 12], I[ip + 13]],
					out: mem().slice(X.out_ptr(), X.out_ptr() + stride),
				};
			},
			crc32(data) {
				const p = X.scratch(data.length);
				mem().set(data, p);
				const c = X.crc32(0, p, data.length) >>> 0;
				X.heap_reset();
				return c;
			},
			combine(a, b, len2) { return X.crc32_combine(a, b, len2) >>> 0; },
		};
	}

	// ------------------------------------------- zip writer (src/zipw.c)
	// write(position, Uint8Array) -> Promise
	function zipWriter(write) {
		const ents = [];
		let pos = 0;
		return {
			get pos() { return pos; },
			// returns the absolute offset for the entry's data
			async begin(nameBytes, method, flags, mtime, mdate, compSize, uncompSize, align) {
				const e = { nameBytes, method, flags: flags & ~0x0008, mtime, mdate, comp: compSize, uncomp: uncompSize, localOff: pos, crc: 0 };
				ents.push(e);
				const nlen = nameBytes.length;
				const base = pos + 30 + nlen + 20;
				let padExtra = 0;
				if (align > 1 && base % align) padExtra = 4 + (align - (base + 4) % align) % align;
				const xlen = 20 + padExtra, hlen = 30 + nlen + xlen;
				const h = new Uint8Array(hlen);
				wr32(h, 0, 0x04034b50); wr16(h, 4, 45); wr16(h, 6, e.flags); wr16(h, 8, method);
				wr16(h, 10, mtime); wr16(h, 12, mdate); wr32(h, 14, 0);
				wr32(h, 18, 0xffffffff); wr32(h, 22, 0xffffffff); wr16(h, 26, nlen); wr16(h, 28, xlen);
				h.set(nameBytes, 30);
				const x = 30 + nlen;
				wr16(h, x, 0x0001); wr16(h, x + 2, 16); wr64(h, x + 4, uncompSize); wr64(h, x + 12, compSize);
				if (padExtra) { wr16(h, x + 20, 0xD935); wr16(h, x + 22, padExtra - 4); }
				await write(pos, h);
				const data = pos + hlen;
				pos = data + compSize;
				return data;
			},
			async end(crc) {
				const e = ents[ents.length - 1];
				e.crc = crc >>> 0;
				const c = new Uint8Array(4);
				wr32(c, 0, e.crc);
				await write(e.localOff + 14, c);
			},
			async finish() {
				const cdOff = pos;
				const parts = [];
				let cdLen = 0;
				for (const e of ents) {
					const nlen = e.nameBytes.length;
					const h = new Uint8Array(46 + nlen + 28);
					wr32(h, 0, 0x02014b50); wr16(h, 4, 45); wr16(h, 6, 45); wr16(h, 8, e.flags); wr16(h, 10, e.method);
					wr16(h, 12, e.mtime); wr16(h, 14, e.mdate); wr32(h, 16, e.crc); wr32(h, 20, 0xffffffff); wr32(h, 24, 0xffffffff);
					wr16(h, 28, nlen); wr16(h, 30, 28); wr32(h, 42, 0xffffffff);
					h.set(e.nameBytes, 46);
					const x = 46 + nlen;
					wr16(h, x, 0x0001); wr16(h, x + 2, 24); wr64(h, x + 4, e.uncomp); wr64(h, x + 12, e.comp); wr64(h, x + 20, e.localOff);
					parts.push(h);
					cdLen += h.length;
				}
				const r = new Uint8Array(56 + 20 + 22);
				const eocd64 = cdOff + cdLen;
				wr32(r, 0, 0x06064b50); wr64(r, 4, 44); wr16(r, 12, 45); wr16(r, 14, 45);
				wr64(r, 24, ents.length); wr64(r, 32, ents.length); wr64(r, 40, cdLen); wr64(r, 48, cdOff);
				wr32(r, 56, 0x07064b50); wr64(r, 64, eocd64); wr32(r, 72, 1);
				wr32(r, 76, 0x06054b50); wr16(r, 84, 0xffff); wr16(r, 86, 0xffff); wr32(r, 88, 0xffffffff); wr32(r, 92, 0xffffffff);
				parts.push(r);
				const all = new Uint8Array(cdLen + r.length);
				let o = 0;
				for (const q of parts) { all.set(q, o); o += q.length; }
				await write(cdOff, all);
				pos = cdOff + all.length;
			},
		};
	}

	// --------------------------------------------------- DSCV header
	function dscvPack(h) {
		const o = new Uint8Array(DSCV_HEADER_SIZE);
		o.set([0x44, 0x53, 0x43, 0x56], 0);   // "DSCV"
		wr16(o, 4, 1); wr16(o, 6, DSCV_HEADER_SIZE);
		wr16(o, 8, h.srcWidth); wr16(o, 10, h.srcHeight); wr16(o, 12, h.cropX); wr16(o, 14, 0);
		wr16(o, 16, h.width); wr16(o, 18, h.height);
		wr16(o, 20, h.f0); wr16(o, 22, h.f0n); wr16(o, 24, h.f1); wr16(o, 26, h.f1n);
		wr32(o, 28, h.frameCount); wr32(o, 32, h.frameDuration); wr32(o, 36, h.qonFlags);
		wr32(o, 40, h.frameBytes); wr32(o, 44, h.stride);
		wr64(o, 48, h.framesOffset); wr64(o, 56, DSCV_HEADER_SIZE);
		wr32(o, 64, DSCV_VBI_ENTRY); wr32(o, 68, h.slices);
		o.set(h.pps, 128);
		return o;
	}

	// ------------------------------------------------- whole conversion
	// Plan the output: which entries are copied and which are converted.
	async function inspect(read, size) {
		const entries = await zipOpen(read, size);
		const out = [];
		for (const e of entries) {
			const qon = await isQon(read, e);
			const item = { entry: e, qon: null };
			if (qon) item.qon = await qonOpen(read, e);
			out.push(item);
		}
		if (!out.some(i => i.qon)) throw new Error('No LaserDisc video (a stored .qon stream) was found in this file.');
		return out;
	}

	function rateLabel(bpp16) { return bpp16 % 16 === 0 ? (bpp16 / 16) + ' bpp' : (bpp16 / 16).toFixed(2) + ' bpp'; }

	// Size of the finished file, for the progress display and free-space check.
	function estimateSize(items, geomOf) {
		let total = 0;
		for (const it of items) {
			const nl = it.entry.nameBytes.length;
			total += 30 + nl + 20 + 4100;
			if (!it.qon) { total += it.entry.compSize; continue; }
			const g = geomOf(it.qon.hdr);
			const fc = it.qon.hdr.frameCount;
			const framesOff = Math.ceil((DSCV_HEADER_SIZE + fc * DSCV_VBI_ENTRY) / 4096) * 4096;
			total += framesOff + fc * g.stride;
		}
		return total + 4096;
	}

	// opts: { read, size, sourceName, write, bpp16, noIch, coder (for setup and CRCs),
	//         encodeFrames(qonItem, settings, onResult) -> Promise (runs the frames,
	//         calls onResult(i, result) for each, possibly out of order and possibly
	//         returning a Promise), onStatus(text), onProgress(done, total), tool }
	async function convert(o) {
		const items = await inspect(o.read, o.size);
		const zw = zipWriter(o.write);
		let json = '{\n  "format": "DSCMMI",\n  "version": 1,\n  "tool": "' + (o.tool || TOOL) + '",\n' +
			'  "source": "' + o.sourceName + '",\n  "video": [';
		let nvid = 0;
		const report = { failures: 0, fallbacks: 0, readFailures: 0, firstFailure: -1, videos: [] };
		const CHUNK = 8 << 20;
		for (const it of items) {
			const e = it.entry;
			if (!it.qon) {
				o.onStatus && o.onStatus('Copying ' + e.name);
				const at = await zw.begin(e.nameBytes, e.method, e.flags, e.mtime, e.mdate, e.compSize, e.uncompSize, 4096);
				for (let done = 0; done < e.compSize;) {
					const n = Math.min(CHUNK, e.compSize - done);
					await o.write(at + done, await o.read(e.dataOff + done, n));
					done += n;
					o.onCopy && o.onCopy(n);
				}
				await zw.end(e.crc);
				continue;
			}
			const q = it.qon, hdr = q.hdr, fc = hdr.frameCount;
			let name = e.name;
			if (name.length > 4 && name.endsWith('.qon')) name = name.slice(0, -4);
			name += '.dscv';
			const nameBytes = utf8.encode(name);
			const g = o.coder.setup(hdr, o.bpp16, o.noIch, q.maxFrame);
			const framesOff = Math.ceil((DSCV_HEADER_SIZE + fc * DSCV_VBI_ENTRY) / 4096) * 4096;
			const total = framesOff + fc * g.stride;
			o.onStatus && o.onStatus('Converting ' + e.name + ' (' + fc.toLocaleString('en-US') + ' frames)');
			const at = await zw.begin(nameBytes, 0, 0, e.mtime, e.mdate, total, total, 4096);
			const crcs = new Uint32Array(fc);
			const vbi = new Uint8Array(fc * DSCV_VBI_ENTRY);
			let done = 0;
			await o.encodeFrames(it, { bpp16: o.bpp16, noIch: o.noIch, maxIn: q.maxFrame }, async (i, r) => {
				crcs[i] = r.crc;
				for (let k = 0; k < 4; k++) wr32(vbi, i * 16 + k * 4, r.vbi[k]);
				if (r.flags & 1) { report.failures++; if (report.firstFailure < 0 || i < report.firstFailure) report.firstFailure = i; }
				else if (r.flags & 2) report.fallbacks++;
				if (r.flags & 4) report.readFailures++;
				await o.write(at + framesOff + i * g.stride, r.out);
				done++;
				o.onProgress && o.onProgress(done, fc);
			});
			// header, VBI table and padding, then the frames' CRCs in order
			const head = new Uint8Array(framesOff);
			head.set(dscvPack({
				srcWidth: hdr.width, srcHeight: hdr.height, cropX: g.cropX, width: g.width, height: g.height,
				f0: g.f0, f0n: g.f0n, f1: g.f1, f1n: g.f1n, frameCount: fc, frameDuration: hdr.frameDuration, qonFlags: hdr.flags,
				frameBytes: g.frameBytes, stride: g.stride, framesOffset: framesOff, slices: g.slices, pps: g.pps,
			}), 0);
			head.set(vbi, DSCV_HEADER_SIZE);
			await o.write(at, head);
			let crc = o.coder.crc32(head);
			for (let f = 0; f < fc; f++) crc = o.coder.combine(crc, crcs[f], g.stride);
			await zw.end(crc);
			const fps = fpsOf(hdr);
			json += (nvid ? ',' : '') + '\n    {\n      "source_entry": "' + e.name + '",\n      "entry": "' + name + '",\n' +
				'      "codec": "VESA DSC 1.1, RGB 4:4:4, 8 bpc",\n      "bits_per_pixel": ' + (o.bpp16 / 16).toFixed(4) + ',\n' +
				'      "width": ' + g.width + ',\n      "height": ' + g.height + ',\n      "crop_x": ' + g.cropX + ',\n      "frame_count": ' + fc + ',\n' +
				'      "frame_bytes": ' + g.frameBytes + ',\n      "frame_stride": ' + g.stride + ',\n      "bandwidth_bytes_per_second": ' + (g.stride * fps).toFixed(0) + '\n    }';
			nvid++;
			report.videos.push({ name, frames: fc, bytesPerSecond: g.stride * fps, width: g.width, height: g.height });
		}
		json += '\n  ]\n}\n';
		const jb = utf8.encode(json);
		const at = await zw.begin(utf8.encode('DSCMMI.json'), 0, 0, 0, 0x21, jb.length, jb.length, 1);
		await o.write(at, jb);
		await zw.end(o.coder.crc32(jb));
		await zw.finish();
		report.size = zw.pos;
		return report;
	}

	G.MMIDSC = { TOOL, zipOpen, isQon, qonOpen, inspect, makeCoder, convert, estimateSize, rateLabel, fpsOf };
})(typeof globalThis !== 'undefined' ? globalThis : self);
