# mmidsc in the browser

`mmidsc.html` is the `.mmi` → `.dscmmi` converter as a single web page, for
people who'd rather not use a command line. Open it, pick the `.mmi`, pick a
quality, press **Convert and save**. Nothing is uploaded: the page reads the
file and writes the result on the same computer.

- **Same output:** the per-frame work (QON decoding, VBI codes, DSC
  encoding, CRCs) is `src/` compiled to WebAssembly, and the archive logic is
  a line-by-line port of `src/main.c`, `src/mmi.c` and `src/zipw.c`. The
  files match `mmidsc convert` byte for byte, except the `"tool"` line in
  `DSCMMI.json` (`mmidsc-web 1.1`).
- **Speed:** one worker per CPU core, about as fast as the native tool.
- **Browsers:**
  - Chrome and Edge write the file straight to where you save it, even from
    a page opened from disk.
  - Firefox and Safari can't do that. They keep the file in the browser's own
    storage and then offer it as a download, so they need free space for two
    copies. That storage also only works when the page comes from a website
    (for example GitHub Pages), not from a file on disk.

## Files

| File | What |
|---|---|
| `mmidsc.html` | The built page: everything inlined (engine, worker, wasm). Share this one. |
| `page.html` | The page's source (layout, UI, workers). |
| `engine.js` | Archive reading and writing, the conversion plan, the wasm wrapper. Also used by the Node test. |
| `wasm/mmidsc_wasm.c`, `wasm/libc.c`, `wasm/shim/` | The wasm build: the C sources plus a tiny freestanding libc. |
| `build_html.py` | Inlines `engine.js` and the wasm into `page.html` → `mmidsc.html`. |
| `test_node.mjs` | Converts with the engine under Node and compares with a native result. |

## Building

Needs clang with the wasm32 target and wasm-ld (no Emscripten or WASI SDK):

```
web/wasm/build.sh
python3 web/build_html.py
```

## Testing

```
mmidsc convert test.mmi ref.dscmmi --bpp 8
node web/test_node.mjs test.mmi web.dscmmi 8 ref.dscmmi    # must print IDENTICAL
```

Done here on a 240-frame test disc: the engine's output was identical to
`mmidsc convert` at 8 and 10 bpp. The page itself, driven in headless
Chromium, produced the same file through both save paths (direct save and
browser storage), and stopping a conversion halfway left nothing behind.
