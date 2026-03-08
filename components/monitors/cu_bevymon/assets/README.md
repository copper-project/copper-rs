# Bundled Fonts

`cu_bevymon` keeps a small generated CopperMono set for the Bevy windowed Ratatui backend:

- `CopperMono-Light.ttf`
- `CopperMono-SemiBold.ttf`
- `CopperMono-LightItalic.ttf`

These files are generated from local system JetBrains Mono Nerd Font Mono faces and patched with
monitor icon glyphs from local Iosevka Fixed SS10 faces so the Bevy monitor does not depend on
runtime font fallback.

The generator lives in [generate_copper_mono.py](./fonts/generate_copper_mono.py).
