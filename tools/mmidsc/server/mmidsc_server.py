#!/usr/bin/env python3
"""mmidsc server: convert LaserActive .mmi disc images to .dscmmi from a web page.

Runs on the machine that holds the discs (a NAS, a desktop, a server) and
drives the native mmidsc converter there, so conversion runs at full native
speed and nothing travels through the browser. Python 3.8 or newer, standard
library only: no venv, no pip.

    python3 mmidsc_server.py /path/to/discs
    python3 mmidsc_server.py /path/to/discs --host 0.0.0.0 --port 8642

Then open http://localhost:8642 (or the machine's address with --host 0.0.0.0).

The page lists every .mmi under the folder, converts one or more of them at
the chosen quality, and queues the jobs (one at a time: each uses every CPU
core). Output goes next to the source as NAME.8bpp.dscmmi (or 9bpp, 10bpp), or
into --out DIR. A file is written as NAME.part and renamed when it's complete.

It finds mmidsc next to this script (mmidsc, mmidsc-linux-x86_64,
mmidsc-windows-x86_64.exe), then on PATH, or wherever --mmidsc points.

There is no login. By default it only listens on this machine (127.0.0.1);
use --host 0.0.0.0 only on a network you trust.
"""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

VERSION = "1.0"
RATES = {8: 12.49, 9: 14.06, 10: 15.62}    # MB/s of a 29.97 fps disc side, as mmidsc writes it
PROGRESS = re.compile(rb"(\d+) / (\d+) frames, ([\d.]+) frames/s, ([\d.]+) s left")
INFO_VIDEO = re.compile(r"(\d+)x(\d+), (\d+) frames, ([\d.]+) fps")


# ------------------------------------------------------------------ setup

def find_mmidsc(explicit):
    if explicit:
        return os.path.abspath(explicit) if os.path.exists(explicit) else shutil.which(explicit)
    here = os.path.dirname(os.path.abspath(__file__))
    names = ["mmidsc-windows-x86_64.exe", "mmidsc.exe"] if os.name == "nt" else ["mmidsc", "mmidsc-linux-x86_64"]
    for d in (here, os.path.join(here, ".."), os.path.join(here, "..", "tools")):
        for n in names:
            p = os.path.join(d, n)
            if os.path.isfile(p) and os.access(p, os.X_OK):
                return os.path.abspath(p)
    for n in names:
        p = shutil.which(n)
        if p:
            return p
    return None


class Config:
    root = "."
    out_dir = None
    mmidsc = None


def inside_root(rel):
    """Absolute path of rel under the disc folder, or None if it points outside it."""
    root = os.path.realpath(Config.root)
    p = os.path.realpath(os.path.join(root, rel))
    try:
        if os.path.commonpath([root, p]) != root:
            return None
    except ValueError:      # different drives on Windows
        return None
    return p


def output_path(src, bpp):
    stem = os.path.basename(src)
    if stem.lower().endswith(".mmi"):
        stem = stem[:-4]
    folder = Config.out_dir or os.path.dirname(src)
    return os.path.join(folder, "%s.%dbpp.dscmmi" % (stem, bpp))


def list_discs():
    root = os.path.realpath(Config.root)
    found = []
    for d, dirs, files in os.walk(root, followlinks=False):
        dirs[:] = sorted(x for x in dirs if not x.startswith("."))
        for f in sorted(files):
            if not f.lower().endswith(".mmi") or f.startswith("."):
                continue
            p = os.path.join(d, f)
            try:
                size = os.path.getsize(p)
            except OSError:
                continue
            outs = {}
            for bpp in RATES:
                o = output_path(p, bpp)
                if os.path.exists(o):
                    outs[bpp] = os.path.getsize(o)
            found.append({"path": os.path.relpath(p, root).replace(os.sep, "/"), "size": size, "done": outs})
    return found


_info_cache = {}

def disc_info(path):
    """Video streams of a disc, from `mmidsc info` (cached by path, size and time)."""
    st = os.stat(path)
    key = (path, st.st_size, st.st_mtime)
    if key in _info_cache:
        return _info_cache[key]
    r = subprocess.run([Config.mmidsc, "info", path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
    text = r.stdout.decode("utf-8", "replace")
    videos = []
    for m in INFO_VIDEO.finditer(text):
        w, h, frames, fps = int(m.group(1)), int(m.group(2)), int(m.group(3)), float(m.group(4))
        videos.append({"width": w, "height": h, "frames": frames, "fps": fps})
    frames = sum(v["frames"] for v in videos)
    secs = sum(v["frames"] / v["fps"] for v in videos if v["fps"])
    info = {"ok": r.returncode == 0 and bool(videos), "videos": videos, "frames": frames, "seconds": secs,
            "sizes": {b: int(rate * 1e6 * secs) for b, rate in RATES.items()},
            "error": None if r.returncode == 0 and videos else (text.strip().splitlines() or ["not a LaserActive disc image"])[-1]}
    _info_cache[key] = info
    return info


# ------------------------------------------------------------------ jobs

class Job:
    _next = 1

    def __init__(self, src_rel, src, bpp, overwrite):
        self.id = Job._next
        Job._next += 1
        self.src_rel, self.src, self.bpp, self.overwrite = src_rel, src, bpp, overwrite
        self.out = output_path(src, bpp)
        self.state = "queued"          # queued, running, done, failed, cancelled
        self.done = self.total = 0
        self.fps = self.left = 0.0
        self.started = self.finished = None
        self.message = ""
        self.log = []
        self.proc = None
        self.cancel = False

    def view(self):
        return {"id": self.id, "src": self.src_rel, "out": os.path.basename(self.out), "bpp": self.bpp,
                "state": self.state, "done": self.done, "total": self.total, "fps": self.fps, "left": self.left,
                "elapsed": (self.finished or time.time()) - self.started if self.started else 0,
                "message": self.message, "log": self.log[-12:]}


jobs = []
jobs_lock = threading.Lock()
wake = threading.Event()


def run_job(j):
    part = j.out + ".part"
    if os.path.exists(j.out) and not j.overwrite:
        j.state, j.message = "failed", "%s already exists" % os.path.basename(j.out)
        return
    os.makedirs(os.path.dirname(j.out) or ".", exist_ok=True)
    cmd = [Config.mmidsc, "convert", j.src, part, "--bpp", str(j.bpp)]
    j.state, j.started = "running", time.time()
    try:
        kw = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if os.name == "nt" else {"start_new_session": True}
        j.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0, **kw)
    except OSError as e:
        j.state, j.message, j.finished = "failed", "couldn't start mmidsc: %s" % e, time.time()
        return
    buf = b""
    while True:
        chunk = j.proc.stdout.read(4096)
        if not chunk:
            break
        buf += chunk
        parts = re.split(rb"[\r\n]", buf)
        buf = parts.pop()
        for line in parts:
            m = PROGRESS.search(line)
            if m:
                j.done, j.total = int(m.group(1)), int(m.group(2))
                j.fps, j.left = float(m.group(3)), float(m.group(4))
            elif line.strip():
                j.log.append(line.decode("utf-8", "replace").strip())
                del j.log[:-200]
    rc = j.proc.wait()
    j.finished = time.time()
    if j.cancel:
        j.state, j.message = "cancelled", "stopped; nothing was saved"
    elif rc == 0 and os.path.exists(part):
        os.replace(part, j.out)
        j.state = "done"
        j.message = "%s, %.1f GB" % (os.path.basename(j.out), os.path.getsize(j.out) / 1e9)
    else:
        j.state = "failed"
        j.message = (j.log[-1] if j.log else "mmidsc exited with code %d" % rc)
    if j.state != "done":
        try:
            os.remove(part)
        except OSError:
            pass


def worker():
    while True:
        wake.wait()
        with jobs_lock:
            j = next((x for x in jobs if x.state == "queued"), None)
            if not j:
                wake.clear()
                continue
        run_job(j)


def stop_job(j):
    j.cancel = True
    if j.state == "queued":
        j.state, j.message = "cancelled", "removed from the queue"
    elif j.proc and j.proc.poll() is None:
        try:
            if os.name == "nt":
                j.proc.terminate()
            else:
                os.killpg(j.proc.pid, signal.SIGTERM)
        except OSError:
            pass


# ------------------------------------------------------------------ http

class Handler(BaseHTTPRequestHandler):
    server_version = "mmidsc-server/" + VERSION

    def log_message(self, fmt, *args):
        pass

    def send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > 1 << 20:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return {}

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            return self.send(200, PAGE.encode(), "text/html; charset=utf-8")
        if path == "/api/discs":
            return self.send(200, {"root": os.path.realpath(Config.root), "out": Config.out_dir,
                                   "rates": RATES, "discs": list_discs()})
        if path == "/api/jobs":
            with jobs_lock:
                return self.send(200, [j.view() for j in jobs])
        self.send(404, {"error": "not found"})

    def do_POST(self):
        # a page from another site can't post JSON here: require the header a form can't set
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return self.send(415, {"error": "JSON only"})
        path, b = urlparse(self.path).path, self.body()
        if path == "/api/info":
            p = inside_root(str(b.get("path", "")))
            if not p or not os.path.isfile(p):
                return self.send(404, {"error": "no such disc image"})
            try:
                return self.send(200, disc_info(p))
            except (OSError, subprocess.SubprocessError) as e:
                return self.send(500, {"error": str(e)})
        if path == "/api/convert":
            try:
                bpp = int(b.get("bpp", 8))
            except (TypeError, ValueError):
                bpp = 0
            if bpp not in RATES:
                return self.send(400, {"error": "quality must be 8, 9 or 10 bpp"})
            added = []
            with jobs_lock:
                for rel in b.get("paths", []):
                    p = inside_root(str(rel))
                    if not p or not os.path.isfile(p):
                        continue
                    if any(x.src == p and x.bpp == bpp and x.state in ("queued", "running") for x in jobs):
                        continue
                    j = Job(str(rel), p, bpp, bool(b.get("overwrite")))
                    jobs.append(j)
                    added.append(j.id)
            wake.set()
            return self.send(200, {"added": added})
        if path == "/api/cancel":
            with jobs_lock:
                for j in jobs:
                    if j.id == b.get("id") and j.state in ("queued", "running"):
                        stop_job(j)
            return self.send(200, {"ok": True})
        if path == "/api/clear":
            with jobs_lock:
                jobs[:] = [j for j in jobs if j.state in ("queued", "running")]
            return self.send(200, {"ok": True})
        self.send(404, {"error": "not found"})


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>mmidsc Server</title>
<style>
:root {
	--bg: #f1f3f6; --panel: #fbfcfd; --ink: #18202c; --muted: #5a6577; --rule: #cdd3dc;
	--accent: #2c56c9; --accent-ink: #ffffff; --good: #1d7a46; --bad: #b3261e; --track: #dde2ea;
	--font: "Segoe UI", system-ui, -apple-system, Roboto, "Helvetica Neue", sans-serif;
	--mono: ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) {
	:root { --bg: #12151b; --panel: #1a1e26; --ink: #e3e7ee; --muted: #9aa4b4; --rule: #333a47;
		--accent: #8aa6ff; --accent-ink: #10131a; --good: #5fcf8f; --bad: #ff8a80; --track: #2a303b; color-scheme: dark; }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink); font: 15px/1.5 var(--font); }
main { max-width: 980px; margin: 0 auto; padding: 28px 16px 60px; display: grid; gap: 20px; }
h1 { font-size: 26px; margin: 0; letter-spacing: -0.01em; }
h2 { font-size: 16px; margin: 0 0 10px; }
.sub { color: var(--muted); margin: 2px 0 0; }
.panel { background: var(--panel); border: 1px solid var(--rule); border-radius: 10px; padding: 16px; min-width: 0; }
.bar { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; margin-bottom: 10px; }
input[type=search] { flex: 1 1 220px; min-width: 0; padding: 7px 10px; border: 1px solid var(--rule); border-radius: 7px; background: var(--bg); color: var(--ink); font: inherit; }
.list { max-height: 420px; overflow: auto; border: 1px solid var(--rule); border-radius: 8px; }
.row { display: grid; grid-template-columns: auto minmax(0, 1fr) auto; gap: 10px; align-items: center; padding: 8px 10px; border-bottom: 1px solid var(--rule); cursor: pointer; }
.row:last-child { border-bottom: 0; }
.row:hover { background: var(--bg); }
.row .name { overflow-wrap: anywhere; }
.row .meta { color: var(--muted); font-size: 13px; white-space: nowrap; font-variant-numeric: tabular-nums; }
.tag { display: inline-block; font-size: 12px; padding: 0 6px; border-radius: 9px; border: 1px solid var(--good); color: var(--good); margin-left: 6px; }
.rates { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0; }
.rates label { flex: 1 1 200px; border: 1px solid var(--rule); border-radius: 8px; padding: 9px 12px; cursor: pointer; }
.rates label:has(input:checked) { border-color: var(--accent); box-shadow: 0 0 0 1px var(--accent); }
.rates b { display: block; }
.rates span { color: var(--muted); font-size: 13px; }
button { font: inherit; border-radius: 7px; padding: 7px 14px; border: 1px solid var(--rule); background: var(--panel); color: var(--ink); cursor: pointer; }
button.primary { background: var(--accent); border-color: var(--accent); color: var(--accent-ink); font-weight: 600; }
button:disabled { opacity: .5; cursor: default; }
button:focus-visible, input:focus-visible, .row:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.muted { color: var(--muted); }
.job { border-top: 1px solid var(--rule); padding: 12px 0; display: grid; gap: 6px; }
.job:first-of-type { border-top: 0; padding-top: 0; }
.job .head { display: flex; flex-wrap: wrap; gap: 8px; justify-content: space-between; align-items: baseline; }
.job .head b { overflow-wrap: anywhere; }
.state { font-size: 12px; padding: 1px 8px; border-radius: 9px; border: 1px solid var(--rule); color: var(--muted); white-space: nowrap; }
.state.running { border-color: var(--accent); color: var(--accent); }
.state.done { border-color: var(--good); color: var(--good); }
.state.failed { border-color: var(--bad); color: var(--bad); }
.track { height: 8px; background: var(--track); border-radius: 4px; overflow: hidden; }
.fill { height: 100%; background: var(--accent); width: 0; transition: width .4s; }
.stats { display: flex; flex-wrap: wrap; gap: 4px 16px; color: var(--muted); font-size: 13px; font-variant-numeric: tabular-nums; }
pre { margin: 0; font: 12px/1.45 var(--mono); color: var(--muted); white-space: pre-wrap; overflow-wrap: anywhere; max-height: 7.5em; overflow: auto; }
.err { color: var(--bad); }
@media (prefers-reduced-motion: reduce) { .fill { transition: none; } }
</style>
</head>
<body>
<main>
<header>
	<h1>mmidsc Server</h1>
	<p class="sub" id="where">Loading…</p>
</header>

<section class="panel">
	<h2>Disc images</h2>
	<div class="bar">
		<input type="search" id="filter" placeholder="Filter by name" aria-label="Filter by name">
		<button id="all">Select all shown</button>
		<button id="none">Clear</button>
		<button id="refresh">Refresh</button>
	</div>
	<div class="list" id="list" role="listbox" aria-multiselectable="true"></div>
	<p class="muted" id="picked"></p>

	<div class="rates" id="rates">
		<label><input type="radio" name="bpp" value="8" checked> <b>Standard, 8 bpp</b><span>12.5 MB/s. Plays from SD cards and USB drives alike.</span></label>
		<label><input type="radio" name="bpp" value="9"> <b>High, 9 bpp</b><span>14.1 MB/s.</span></label>
		<label><input type="radio" name="bpp" value="10"> <b>Highest, 10 bpp</b><span>15.6 MB/s. Needs a fast SD card.</span></label>
	</div>
	<div class="bar">
		<button class="primary" id="go" disabled>Convert</button>
		<label><input type="checkbox" id="overwrite"> Replace existing files</label>
	</div>
</section>

<section class="panel">
	<div class="bar" style="justify-content: space-between; margin-bottom: 4px">
		<h2 style="margin: 0">Queue</h2>
		<button id="clear">Clear finished</button>
	</div>
	<p class="muted" id="nojobs">Nothing converting.</p>
	<div id="jobs"></div>
</section>
</main>

<script>
'use strict';
const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const gb = n => (n / 1e9).toFixed(2) + ' GB';
const dur = s => { s = Math.round(s); const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, x = s % 60; return h ? h + ' h ' + m + ' min' : m ? m + ' min ' + x + ' s' : x + ' s'; };
const post = (url, body) => fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }).then(r => r.json());
let discs = [], picked = new Set(), info = {};

async function loadDiscs() {
	const r = await (await fetch('/api/discs')).json();
	discs = r.discs;
	$('where').textContent = discs.length + ' disc image' + (discs.length === 1 ? '' : 's') + ' in ' + r.root + (r.out ? ', saving to ' + r.out : ', saving next to each source');
	for (const p of [...picked]) if (!discs.some(d => d.path === p)) picked.delete(p);
	drawList();
}

function drawList() {
	const f = $('filter').value.trim().toLowerCase();
	const shown = discs.filter(d => !f || d.path.toLowerCase().includes(f));
	$('list').innerHTML = shown.length ? shown.map(d => {
		const done = Object.keys(d.done).map(b => '<span class="tag">' + b + ' bpp done</span>').join('');
		const i = info[d.path];
		const meta = i ? (i.ok ? dur(i.seconds) + ' · ' + gb(d.size) : '<span class="err">' + esc(i.error) + '</span>') : gb(d.size);
		return '<div class="row" role="option" tabindex="0" aria-selected="' + picked.has(d.path) + '" data-p="' + esc(d.path) + '">' +
			'<input type="checkbox" tabindex="-1" ' + (picked.has(d.path) ? 'checked' : '') + '>' +
			'<span class="name">' + esc(d.path) + done + '</span><span class="meta">' + meta + '</span></div>';
	}).join('') : '<p class="muted" style="padding: 10px; margin: 0">No .mmi files' + (f ? ' match' : ' here') + '.</p>';
	const p = [...picked];
	const secs = p.reduce((a, x) => a + (info[x] && info[x].ok ? info[x].seconds : 0), 0);
	const bpp = +document.querySelector('input[name=bpp]:checked').value;
	const rate = { 8: 12.49, 9: 14.06, 10: 15.62 }[bpp];
	$('picked').textContent = p.length ? p.length + ' selected' + (secs ? ', about ' + gb(rate * 1e6 * secs) + ' of output at ' + bpp + ' bpp' : '') : 'Pick one or more disc images.';
	$('go').disabled = !p.length;
}

function toggle(path) {
	if (picked.has(path)) picked.delete(path); else picked.add(path);
	if (!info[path]) post('/api/info', { path }).then(i => { info[path] = i.error && !i.videos ? { ok: false, error: i.error } : i; drawList(); });
	drawList();
}

$('list').addEventListener('click', e => { const r = e.target.closest('.row'); if (r) toggle(r.dataset.p); });
$('list').addEventListener('keydown', e => { const r = e.target.closest('.row'); if (r && (e.key === ' ' || e.key === 'Enter')) { e.preventDefault(); toggle(r.dataset.p); } });
$('filter').addEventListener('input', drawList);
$('rates').addEventListener('change', drawList);
$('all').onclick = () => { $('list').querySelectorAll('.row').forEach(r => { if (!picked.has(r.dataset.p)) toggle(r.dataset.p); }); };
$('none').onclick = () => { picked.clear(); drawList(); };
$('refresh').onclick = loadDiscs;
$('go').onclick = async () => {
	const bpp = +document.querySelector('input[name=bpp]:checked').value;
	await post('/api/convert', { paths: [...picked], bpp, overwrite: $('overwrite').checked });
	picked.clear(); drawList(); pollJobs();
};
$('clear').onclick = () => post('/api/clear', {}).then(pollJobs);

const label = { queued: 'Queued', running: 'Converting', done: 'Done', failed: 'Failed', cancelled: 'Stopped' };
let lastDone = 0;
async function pollJobs() {
	let js;
	try { js = await (await fetch('/api/jobs')).json(); } catch (e) { $('nojobs').hidden = false; $('nojobs').textContent = 'Lost contact with the server.'; return; }
	$('nojobs').hidden = js.length > 0;
	$('nojobs').textContent = 'Nothing converting.';
	$('jobs').innerHTML = js.slice().reverse().map(j => {
		const pct = j.total ? 100 * j.done / j.total : (j.state === 'done' ? 100 : 0);
		const stats = j.state === 'running' && j.total
			? '<span>' + j.done.toLocaleString('en-US') + ' of ' + j.total.toLocaleString('en-US') + ' frames</span><span>' + j.fps.toFixed(1) + ' frames/s</span><span>' + dur(j.left) + ' left</span>'
			: j.elapsed ? '<span>' + dur(j.elapsed) + '</span>' : '';
		const stop = j.state === 'queued' || j.state === 'running' ? '<button data-stop="' + j.id + '">Stop</button>' : '';
		const msg = j.message ? '<div class="' + (j.state === 'failed' ? 'err' : 'muted') + '">' + esc(j.message) + '</div>' : '';
		const log = j.state === 'failed' && j.log.length ? '<pre>' + esc(j.log.join('\n')) + '</pre>' : '';
		return '<div class="job"><div class="head"><b>' + esc(j.src) + ' → ' + esc(j.out) + '</b><span><span class="state ' + j.state + '">' + label[j.state] + '</span> ' + stop + '</span></div>' +
			(j.state === 'running' || j.state === 'done' ? '<div class="track"><div class="fill" style="width:' + pct.toFixed(2) + '%"></div></div>' : '') +
			'<div class="stats">' + stats + '</div>' + msg + log + '</div>';
	}).join('');
	const nDone = js.filter(j => j.state === 'done').length;
	if (nDone !== lastDone) { lastDone = nDone; loadDiscs(); }
}
$('jobs').addEventListener('click', e => { const b = e.target.closest('[data-stop]'); if (b) post('/api/cancel', { id: +b.dataset.stop }).then(pollJobs); });

loadDiscs();
pollJobs();
setInterval(pollJobs, 1000);
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description="Convert LaserActive .mmi disc images to .dscmmi from a web page.")
    ap.add_argument("root", nargs="?", default=".", help="folder to look for .mmi files in (with subfolders)")
    ap.add_argument("--out", help="save converted files here instead of next to each source")
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (0.0.0.0 for the whole network; there is no login)")
    ap.add_argument("--port", type=int, default=8642)
    ap.add_argument("--mmidsc", help="path to the mmidsc converter")
    a = ap.parse_args()

    Config.root = os.path.abspath(a.root)
    Config.out_dir = os.path.abspath(a.out) if a.out else None
    Config.mmidsc = find_mmidsc(a.mmidsc)
    if not os.path.isdir(Config.root):
        sys.exit("error: %s is not a folder" % Config.root)
    if not Config.mmidsc:
        sys.exit("error: can't find mmidsc. Put mmidsc (or mmidsc-linux-x86_64) next to this script, add it to PATH, or use --mmidsc PATH.")

    threading.Thread(target=worker, daemon=True).start()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    shown = "localhost" if a.host in ("127.0.0.1", "0.0.0.0", "::") else a.host
    print("mmidsc server %s: discs in %s, converter %s" % (VERSION, Config.root, Config.mmidsc))
    print("open http://%s:%d   (Ctrl+C to stop)" % (shown, a.port))
    if a.host not in ("127.0.0.1", "localhost", "::1"):
        print("note: listening on %s with no login; anyone who can reach this port can start conversions" % a.host)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
        with jobs_lock:
            for j in jobs:
                if j.state in ("queued", "running"):
                    stop_job(j)
        for j in jobs:
            if j.proc:
                try:
                    j.proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    j.proc.kill()


if __name__ == "__main__":
    main()
