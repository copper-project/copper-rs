import pathlib
import sys

import fontforge
import psMat


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


def main() -> int:
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
