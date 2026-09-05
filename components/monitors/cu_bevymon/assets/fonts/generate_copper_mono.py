import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
SYSTEM_FONTS = pathlib.Path("/usr/share/fonts")
IOSEVKA_FIXED_INDEX = 2
PATCH_CODEPOINTS = (
    0x2699,
    0x274C,
    0x29D6,
    0x29D7,
    0x2B06,
    0x2B07,
    0x2B73,
    0x21C6,
)


def resolve_font_path(font_name: str) -> pathlib.Path:
    for path in SYSTEM_FONTS.rglob(font_name):
        if path.is_file():
            return path
    raise FileNotFoundError(f"unable to find system font {font_name!r} under {SYSTEM_FONTS}")


def open_font(path: pathlib.Path, index: int | None = None):
    import fontforge

    if index is None:
        return fontforge.open(str(path))
    return fontforge.open(f"{path}({index})")


def patch_font(
    *,
    base_font_name: str,
    source_font_name: str,
    output_font_name: str,
    family_name: str,
    full_name: str,
):
    import psMat

    base_path = resolve_font_path(base_font_name)
    source_path = resolve_font_path(source_font_name)
    output_path = ROOT / output_font_name

    base = open_font(base_path)
    source = open_font(source_path, IOSEVKA_FIXED_INDEX)

    cell_width = base[ord("M")].width

    for codepoint in PATCH_CODEPOINTS:
        source.selection.select(codepoint)
        source.copy()
        base.selection.select(codepoint)
        base.paste()

        glyph = base[codepoint]
        delta = cell_width - glyph.width
        if delta:
            glyph.transform(psMat.translate(delta / 2.0, 0))
            glyph.width = cell_width

    base.familyname = family_name
    base.fontname = family_name.replace(" ", "") + "-" + full_name.split()[-1]
    base.fullname = full_name
    base.generate(str(output_path))


def patch_mascot():
    from fontTools.pens.boundsPen import BoundsPen
    from fontTools.pens.transformPen import TransformPen
    from fontTools.pens.ttGlyphPen import TTGlyphPen
    from fontTools.ttLib import TTFont

    # The software backend uses outline fonts with no system emoji fallback.
    with TTFont(resolve_font_path("DejaVuSans.ttf")) as source:
        source_glyph = source.getGlyphSet()[source.getBestCmap()[0x1F63C]]
        bounds = BoundsPen(source.getGlyphSet())
        source_glyph.draw(bounds)
        xmin, ymin, xmax, ymax = bounds.bounds
        for path in sorted(ROOT.glob("CopperMono-*.ttf")):
            with TTFont(path, recalcTimestamp=False) as font:
                cell_width = font["hmtx"][font.getBestCmap()[ord("M")]][0]
                # Fit a square cat inside the backend's single-cell glyph clip.
                scale = cell_width * 0.9 / max(xmax - xmin, ymax - ymin)
                x = (cell_width - (xmax - xmin) * scale) / 2 - xmin * scale
                y = (font["OS/2"].sCapHeight - (ymax - ymin) * scale) / 2 - ymin * scale
                pen = TTGlyphPen(None)
                source_glyph.draw(TransformPen(pen, (scale, 0, 0, scale, x, y)))
                name = "copperMascot"
                if name not in font.getGlyphOrder():
                    font.setGlyphOrder([*font.getGlyphOrder(), name])
                font["glyf"][name] = pen.glyph()
                font["hmtx"][name] = (cell_width, round(cell_width * 0.05))
                for table in font["cmap"].tables:
                    if table.isUnicode() and table.format in (12, 13):
                        table.cmap[0x1F63C] = name
                font.save(path)


def main() -> int:
    if sys.argv[1:] == ["--mascot-only"]:
        patch_mascot()
        return 0

    patch_font(
        base_font_name="JetBrainsMonoNerdFontMono-Light.ttf",
        source_font_name="IosevkaSS10-Light.ttc",
        output_font_name="CopperMono-Light.ttf",
        family_name="CopperMono",
        full_name="CopperMono Light",
    )
    patch_font(
        base_font_name="JetBrainsMonoNerdFontMono-SemiBold.ttf",
        source_font_name="IosevkaSS10-SemiBold.ttc",
        output_font_name="CopperMono-SemiBold.ttf",
        family_name="CopperMono",
        full_name="CopperMono SemiBold",
    )
    patch_font(
        base_font_name="JetBrainsMonoNerdFontMono-LightItalic.ttf",
        source_font_name="IosevkaSS10-Light.ttc",
        output_font_name="CopperMono-LightItalic.ttf",
        family_name="CopperMono",
        full_name="CopperMono LightItalic",
    )
    patch_mascot()
    return 0


if __name__ == "__main__":
    sys.exit(main())
