# LaserActive tools

| Folder | What |
|---|---|
| `mmidsc/` | The `.mmi` → `.dscmmi` converter: the command-line tool (`src/`, `make`), the same converter as a single web page (`web/mmidsc.html`, no install needed), the DSC codec tests, and the Verilator testbenches for the core's LaserDisc video RTL (`rtl/tb/`, the RTL is in `../rtl/LD/`). |
| `ldtest/` | Host test harness for the Main_MiSTer side of the LaserActive player (`support/megacd/megald*.cpp`), run against a model of the core. Needs a Main_MiSTer checkout with the LaserActive branch. |

See `../LaserActive.md` for how it all fits together.
