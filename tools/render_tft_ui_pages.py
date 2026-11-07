#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_ROOT = REPO_ROOT / ".venv"
PYTHON_LIB = VENV_ROOT / "lib"

if not sys.modules.get("PIL"):
    candidates = []
    if PYTHON_LIB.exists():
        for lib_dir in sorted(PYTHON_LIB.iterdir(), reverse=True):
            site_pkg = lib_dir / "site-packages"
            if site_pkg.exists():
                candidates.append(str(site_pkg))
    for candidate in candidates:
        if candidate not in sys.path:
            sys.path.insert(0, candidate)

try:
    from PIL import Image, ImageDraw, ImageFont
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Pillow is not installed. Activate the project venv or install it with: "
        "'python3 -m pip install pillow'"
    ) from exc

WIDTH = 320
HEIGHT = 240
OUT_DIR = REPO_ROOT / "docs" / "ui_pages"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def find_font(*candidates: str) -> str | None:
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            return str(path)
    return None


def load_font(size: int, preferred: tuple[str, ...], fallback: tuple[str, ...]):
    font_path = find_font(*preferred) or find_font(*fallback)
    if font_path is None:
        return None
    return ImageFont.truetype(font_path, size)


def rgb565_to_rgb(color: int):
    r = ((color >> 11) & 0x1F) * 255 // 31
    g = ((color >> 5) & 0x3F) * 255 // 63
    b = (color & 0x1F) * 255 // 31
    return (r, g, b)


menu_names = [
    "Pulse1", "Pulse2", "Interval", "Trigger", "Series",
    "CellV", "Balance", "ChargeI", "ChargeOn", "Bright",
    "Save", "Exit", "Home"
]


def draw_text_centered(draw, box_x, box_y, box_w, text, font, fill, bg=None):
    if box_w <= 0:
        return
    bbox = draw.textbbox((0, 0), text, font=font)
    text_w = bbox[2] - bbox[0]
    start_x = box_x + max(0, (box_w - text_w) // 2)
    if bg is not None:
        draw.rectangle((box_x, box_y, box_x + box_w - 1, box_y + 20), fill=bg)
    draw.text((start_x, box_y), text, font=font, fill=fill)


def draw_mixed_text_centered(draw, box_x, box_y, box_w, segments, fill):
    widths = [
        draw.textbbox((0, 0), text, font=font)[2]
        for text, font in segments
    ]
    start_x = box_x + max(0, (box_w - sum(widths)) // 2)
    for (text, font), text_w in zip(segments, widths):
        draw.text((start_x, box_y), text, font=font, fill=fill)
        start_x += text_w


def draw_text_left(draw, x, y, text, font, fill):
    draw.text((x, y), text, font=font, fill=fill)


def render_home(path: Path):
    image = Image.new("RGB", (WIDTH, HEIGHT), rgb565_to_rgb(0x0000))
    draw = ImageDraw.Draw(image)
    block_colors = [
        rgb565_to_rgb(0xFC15),
        rgb565_to_rgb(0x67FB),
        rgb565_to_rgb(0xB45F),
        rgb565_to_rgb(0xFA8A),
        rgb565_to_rgb(0x445F),
        rgb565_to_rgb(0xFEA8),
    ]
    cn_font = load_font(
        14,
        (
            "/System/Library/Fonts/STHeiti Medium.ttc",
            "/System/Library/Fonts/Heiti.ttc",
            "/Library/Fonts/Heiti.ttc",
        ),
        (
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        ),
    ) or ImageFont.load_default()
    en_font = load_font(
        18,
        (
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Monaco.ttf",
            "/Library/Fonts/Menlo.ttc",
        ),
        (
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
        ),
    ) or ImageFont.load_default()

    labels = ["脉冲1", "脉冲2", "间隔", "等待", "总次数", "次数"]
    values = ["10ms", "15ms", "20ms", "3.0s", "0", "0"]
    card_width = 100
    card_height = 81
    card_gap = 2
    card_margin_x = 8
    card_top = 4
    for i in range(6):
        row = i // 3
        col = i % 3
        box_x = card_margin_x + col * (card_width + card_gap)
        box_y = card_top + row * (card_height + card_gap)
        draw.rounded_rectangle(
            (box_x, box_y, box_x + card_width - 1, box_y + card_height - 1),
            radius=8,
            fill=block_colors[i],
        )

        draw_text_centered(draw, box_x + 4, box_y + 17, card_width - 8, labels[i], cn_font, (0, 0, 0))
        draw_text_centered(draw, box_x + 4, box_y + 49, card_width - 8, values[i], en_font, (0, 0, 0))

    line1 = "C1 2.70V  C2 2.73V"
    line2 = "C3 2.76V  C4 2.79V"
    draw_text_centered(draw, 0, 168, WIDTH, line1, en_font, (255, 255, 255))
    draw_text_centered(draw, 0, 189, WIDTH, line2, en_font, (255, 255, 255))
    draw_mixed_text_centered(
        draw, 0, 212, WIDTH,
        [("总压", cn_font), ("10.80V ", en_font), ("功率", cn_font),
         ("0.0W ", en_font), ("电流", cn_font), ("0.0A", en_font)],
        (255, 255, 255),
    )

    image.save(path)


def render_menu(path: Path, *, edit_mode: bool = False, selected_index: int = 0, message: str | None = None):
    image = Image.new("RGB", (WIDTH, HEIGHT), (0, 0, 0))
    draw = ImageDraw.Draw(image)
    blue = (0, 122, 255)
    white = (255, 255, 255)
    black = (0, 0, 0)
    grey = (235, 235, 235)
    green = (0, 200, 120)

    cn_font = load_font(
        18,
        (
            "/System/Library/Fonts/STHeiti Medium.ttc",
            "/System/Library/Fonts/Heiti.ttc",
            "/Library/Fonts/Heiti.ttc",
        ),
        (
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        ),
    ) or ImageFont.load_default()
    title_font = cn_font
    value_font = load_font(
        20,
        (
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Monaco.ttf",
            "/Library/Fonts/Menlo.ttc",
        ),
        (
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
        ),
    ) or ImageFont.load_default()
    msg_font = load_font(
        16,
        (
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/Monaco.ttf",
            "/Library/Fonts/Menlo.ttc",
        ),
        (
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
        ),
    ) or ImageFont.load_default()

    draw.text((0, 8), "参数菜单", font=title_font, fill=white, anchor="ma")
    # panel area
    panel_x0, panel_y0 = 10, 30
    panel_x1, panel_y1 = WIDTH - 10, HEIGHT - 12
    draw.rectangle((panel_x0, panel_y0, panel_x1, panel_y1), fill=grey)

    labels = ["脉冲1", "脉冲2", "脉冲间隔", "触发时间"]
    values = ["5 ms", "5 ms", "3 ms", "1.0 s"]
    row_h = (panel_y1 - panel_y0) // 4
    for i, label in enumerate(labels):
        row_top = panel_y0 + i * row_h
        row_bottom = row_top + row_h
        row_fg = white if i == selected_index else black
        row_bg = blue if i == selected_index else white
        draw.rectangle((panel_x0, row_top, panel_x1, row_bottom), fill=row_bg)
        draw.text((18, row_top + 10), label, font=cn_font, fill=row_fg)
        value = values[i]
        value_w = draw.textbbox((0, 0), value, font=value_font)[2]
        draw.text((WIDTH - 18 - value_w, row_top + 10), value, font=value_font, fill=row_fg)

    if message:
        draw.rectangle((0, HEIGHT - 20, WIDTH - 1, HEIGHT - 1), fill=black)
        draw.text((10, HEIGHT - 18), message, font=msg_font, fill=green)

    image.save(path)


def render_all():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    render_home(OUT_DIR / "home_page.png")
    render_menu(OUT_DIR / "menu_page_idle.png", selected_index=0, edit_mode=False)
    render_menu(OUT_DIR / "menu_page_editing.png", selected_index=0, edit_mode=True)
    render_menu(OUT_DIR / "menu_page_message.png", selected_index=0, edit_mode=False, message="Saved")

    # montage for quick viewing
    pages = [
        Image.open(OUT_DIR / "home_page.png").convert("RGB"),
        Image.open(OUT_DIR / "menu_page_idle.png").convert("RGB"),
        Image.open(OUT_DIR / "menu_page_editing.png").convert("RGB"),
        Image.open(OUT_DIR / "menu_page_message.png").convert("RGB"),
    ]
    collage_w = 2 * WIDTH
    collage_h = 2 * HEIGHT
    collage = Image.new("RGB", (collage_w, collage_h), (0, 0, 0))
    for idx, page in enumerate(pages):
        row = idx // 2
        col = idx % 2
        x = col * WIDTH
        y = row * HEIGHT
        collage.paste(page, (x, y))
    collage.save(OUT_DIR / "all_pages.png")


if __name__ == "__main__":
    render_all()
    print(f"Saved rendered pages to {OUT_DIR}")
