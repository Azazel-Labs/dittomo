"""Writes dittomo's logo files: the mark, the wordmark and their lockups, as SVG.

The mark is a ditto sign, 〃, drawn as two bones: the same stroke twice, the second a size bigger. Same motion, another
body. The wordmark is drawn from strokes, so it needs no font. Run `sh build.sh` to write these and render the PNGs.

    python -I brand.py OUT_DIR
"""

import sys
from pathlib import Path

CORAL = ("#FF8A5B", "#FF4F64")  # the source
TEAL = ("#41E8C3", "#2AA3E6")  # the copy
INK = "#10132A"
PAPER = "#F6F3EC"
NIGHT = ("#0B0E1F", "#171C3A")


def bone(top, bottom, shaft, joint, grad):
    (x0, y0), (x1, y1) = top, bottom
    return (
        f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="url(#{grad})" '
        f'stroke-width="{shaft}" stroke-linecap="round"/>'
        f'<circle cx="{x0:.1f}" cy="{y0:.1f}" r="{joint}" fill="url(#{grad})"/>'
        f'<circle cx="{x1:.1f}" cy="{y1:.1f}" r="{joint}" fill="url(#{grad})"/>'
    )


def gradients(prefix=""):
    def lin(name, a, b):
        return (
            f'<linearGradient id="{prefix}{name}" x1="0" y1="0" x2="0.35" y2="1">'
            f'<stop offset="0" stop-color="{a}"/><stop offset="1" stop-color="{b}"/></linearGradient>'
        )

    return lin("coral", *CORAL) + lin("teal", *TEAL)


def mark(prefix=""):
    """The two bones in a 512 box, both leaning the same way."""
    slope = -60 / 256
    a_top, a_bottom = (216, 128), (156, 384)
    b_top = (352, 104)
    b_bottom = (b_top[0] + slope * (408 - 104), 408)
    return bone(a_top, a_bottom, 40, 40, f"{prefix}coral") + bone(b_top, b_bottom, 46, 47, f"{prefix}teal")


# The wordmark, from round-capped strokes 24 wide: x-height 110, ascenders 160, baseline at y = 0.
SW = 24


def wordmark(color, dot):
    r = 43  # a bowl's radius, to the middle of its stroke
    parts = []
    circle = lambda cx: f'<circle cx="{cx}" cy="-55" r="{r}" fill="none" stroke="{color}" stroke-width="{SW}"/>'
    path = lambda d: (
        f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{SW}" stroke-linecap="round" stroke-linejoin="round"/>'
    )
    # d
    parts += [circle(55), path("M98,-148 V-12")]
    # i, its dot a joint
    parts += [path("M148,-98 V-12"), f'<circle cx="148" cy="-147" r="16" fill="{dot}"/>']
    # tt, one crossbar for the pair
    parts += [path("M212,-142 V-46 Q212,-12 246,-12"), path("M290,-142 V-46 Q290,-12 324,-12"), path("M192,-98 H330")]
    # o
    parts += [circle(411)]
    # m
    parts += [path("M504,-98 V-12 M504,-68 A30,30 0 0 1 564,-68 V-12 M564,-68 A30,30 0 0 1 624,-68 V-12")]
    # o
    parts += [circle(717)]
    return "".join(parts)


WORD_W, WORD_TOP = 772, -163


def svg(w, h, body, defs=""):
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}">'
        f"<defs>{gradients()}{defs}</defs>{body}</svg>\n"
    )


def lockup(color):
    """The mark beside the wordmark, the mark's middle level with the wordmark's."""
    # The mark spans x 116..399 and y 57..455 of its box.
    s, pad = 0.56, 30
    mark_x, mark_top = pad - 116 * s, pad - 57 * s
    word_x = pad + 283 * s + 52
    baseline = pad + 398 * s / 2 - WORD_TOP / 2
    body = (
        f'<g transform="translate({mark_x:.1f},{mark_top:.1f}) scale({s})">{mark()}</g>'
        f'<g transform="translate({word_x:.1f},{baseline:.1f})">{wordmark(color, CORAL[1])}</g>'
    )
    return svg(round(word_x + WORD_W + pad), round(2 * pad + 398 * s), body)


def icon():
    """The mark on a rounded square of night, for avatars and app icons."""
    defs = (
        f'<linearGradient id="night" x1="0" y1="0" x2="1" y2="1">'
        f'<stop offset="0" stop-color="{NIGHT[1]}"/><stop offset="1" stop-color="{NIGHT[0]}"/></linearGradient>'
        f'<radialGradient id="glow" cx="0.5" cy="0.42" r="0.6">'
        f'<stop offset="0" stop-color="#2A2F66" stop-opacity="0.75"/><stop offset="1" stop-color="#2A2F66" stop-opacity="0"/>'
        f"</radialGradient>"
    )
    body = (
        '<rect width="512" height="512" rx="116" fill="url(#night)"/>'
        '<rect width="512" height="512" rx="116" fill="url(#glow)"/>'
        f'<g transform="translate(256,256) scale(0.9) translate(-257.5,-256)">{mark()}</g>'
    )
    return svg(512, 512, body, defs)


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    files = {
        "dittomo-mark.svg": svg(512, 512, f'<g transform="translate(-1.5,0)">{mark()}</g>'),
        "dittomo-icon.svg": icon(),
        "dittomo-logo.svg": lockup(INK),
        "dittomo-logo-light.svg": lockup(PAPER),
        "dittomo-wordmark.svg": svg(
            WORD_W + 8, -WORD_TOP + 20, f'<g transform="translate(4,{-WORD_TOP + 4})">{wordmark(INK, CORAL[1])}</g>'
        ),
    }
    for name, text in files.items():
        (out / name).write_text(text, encoding="utf-8")
        print(out / name)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
