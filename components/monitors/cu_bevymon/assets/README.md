# Bundled Fonts

`cu_bevymon` keeps a small generated CopperMono set for the Bevy windowed Ratatui backend:

- `CopperMono-Light.ttf`
- `CopperMono-SemiBold.ttf`
- `CopperMono-LightItalic.ttf`

These files are generated from local system JetBrains Mono Nerd Font Mono faces and patched with
monitor icon glyphs from local Iosevka Fixed SS10 faces so the Bevy monitor does not depend on
runtime font fallback.

The 😼 Copper mascot uses a monochrome outline from DejaVu Sans, fitted to the
software backend's glyph cell. Its license is included in
[fonts/DejaVu-LICENSE.txt](./fonts/DejaVu-LICENSE.txt).

The generator also needs Python `fontTools` and a local `DejaVuSans.ttf`. Use
`python3 fonts/generate_copper_mono.py --mascot-only` from this directory to patch
the mascot into the existing bundled fonts without regenerating the other glyphs.

The generator lives in [generate_copper_mono.py](./fonts/generate_copper_mono.py).
