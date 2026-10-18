#!/usr/bin/env python3
"""Build mmidsc.html: page.html with engine.js and wasm/mmidsc.wasm inlined,
so the converter is one file that works offline (fonts aside)."""
import base64, os, sys
here = os.path.dirname(os.path.abspath(__file__))
page = open(os.path.join(here, 'page.html'), encoding='utf-8').read()
engine = open(os.path.join(here, 'engine.js'), encoding='utf-8').read()
wasm = base64.b64encode(open(os.path.join(here, 'wasm', 'mmidsc.wasm'), 'rb').read()).decode()
assert '</script' not in engine
assert page.count('/*ENGINE*/') == 1 and page.count("'/*WASM*/'") == 1
page = page.replace('/*ENGINE*/', engine).replace("'/*WASM*/'", "'" + wasm + "'")
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'mmidsc.html')
open(out, 'w', encoding='utf-8').write(page)
print('wrote', out, len(page), 'bytes')
