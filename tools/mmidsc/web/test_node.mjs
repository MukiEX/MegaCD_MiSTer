// Node test of the browser converter's engine: converts an .mmi with
// engine.js + mmidsc.wasm and compares the result with `mmidsc convert`.
//   node test_node.mjs IN.mmi OUT.dscmmi [bpp] [REFERENCE.dscmmi]
// With a reference, the tool line in DSCMMI.json is written as the native
// tool's, so the files must be identical byte for byte.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const here = path.dirname(fileURLToPath(import.meta.url));
await import(path.join(here, 'engine.js'));
const M = globalThis.MMIDSC;

const [inPath, outPath, bppArg, refPath] = process.argv.slice(2);
const bpp16 = Math.round(parseFloat(bppArg || '8') * 16);
const fin = fs.openSync(inPath, 'r');
const size = fs.fstatSync(fin).size;
const read = async (off, len) => {
	const b = Buffer.alloc(len);
	let got = 0;
	while (got < len) {
		const n = fs.readSync(fin, b, got, len - got, off + got);
		if (n <= 0) throw new Error('read past end');
		got += n;
	}
	return new Uint8Array(b.buffer, b.byteOffset, len);
};
const fout = fs.openSync(outPath, 'w');
const write = async (pos, data) => { fs.writeSync(fout, data, 0, data.length, pos); };
const module = await WebAssembly.compile(fs.readFileSync(path.join(here, 'wasm', 'mmidsc.wasm')));
const coder = await M.makeCoder(module);
const worker = await M.makeCoder(module);
const t0 = Date.now();
const report = await M.convert({
	read, size, sourceName: path.basename(inPath), write, bpp16, noIch: false, coder,
	tool: refPath ? 'mmidsc 1.1' : undefined,
	onStatus: s => console.log(s),
	async encodeFrames(item, s, onResult) {
		const q = item.qon;
		worker.setup(q.hdr, s.bpp16, s.noIch, s.maxIn);
		// out of order, like the browser's workers
		const order = [...Array(q.hdr.frameCount).keys()].sort((a, b) => ((a * 7919) % 97) - ((b * 7919) % 97) || a - b);
		for (const i of order) {
			const len = (await read(q.off[i], 4)).reduce((a, v, k) => a + v * 2 ** (8 * k), 0);
			const frame = q.off[i] + 4 + len > q.end ? null : await read(q.off[i] + 4, len);
			await onResult(i, worker.encode(frame));
		}
	},
});
fs.closeSync(fout);
console.log('wrote', outPath, report.size, 'bytes in', ((Date.now() - t0) / 1000).toFixed(1), 's;',
	'failures', report.failures, 'fallbacks', report.fallbacks, 'unreadable', report.readFailures);
if (refPath) {
	const a = fs.readFileSync(outPath), b = fs.readFileSync(refPath);
	let first = -1;
	if (a.length === b.length) { for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) { first = i; break; } }
	console.log(a.length === b.length && first < 0 ? 'IDENTICAL to ' + refPath : 'DIFFERENT from ' + refPath + ' (sizes ' + a.length + ' / ' + b.length + ', first difference at ' + first + ')');
	process.exit(a.length === b.length && first < 0 ? 0 : 1);
}
