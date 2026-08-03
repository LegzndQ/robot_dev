from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from .camera_calibration import make_charuco_board


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Generate the ChArUco board used by iPhone calibration."
    )
    parser.add_argument("--output", default="calibration/charuco_7x5_30mm.pdf")
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=30.0)
    parser.add_argument("--marker-mm", type=float, default=22.0)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--dpi", type=int, default=300)
    return parser.parse_args(args)


def main(args=None) -> None:
    options = parse_args(args)
    output = Path(options.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    board, _ = make_charuco_board(
        options.squares_x,
        options.squares_y,
        options.square_mm / 1000.0,
        options.marker_mm / 1000.0,
        options.dictionary,
    )

    pixels_per_mm = options.dpi / 25.4
    board_width = round(options.squares_x * options.square_mm * pixels_per_mm)
    board_height = round(options.squares_y * options.square_mm * pixels_per_mm)
    board_image = board.draw((board_width, board_height), marginSize=0)

    page_width = round(297.0 * pixels_per_mm)
    page_height = round(210.0 * pixels_per_mm)
    if board_width > page_width or board_height > page_height:
        raise ValueError("Board does not fit on an A4 landscape page")
    page = np.full((page_height, page_width), 255, dtype=np.uint8)
    x = (page_width - board_width) // 2
    y = (page_height - board_height) // 2
    page[y:y + board_height, x:x + board_width] = board_image
    image = Image.fromarray(page).convert("RGB")

    pdf_path = output.with_suffix(".pdf")
    png_path = output.with_suffix(".png")
    yaml_path = output.with_suffix(".yaml")
    image.save(pdf_path, "PDF", resolution=float(options.dpi))
    image.save(png_path, "PNG", dpi=(options.dpi, options.dpi))
    metadata = {
        "squares_x": options.squares_x,
        "squares_y": options.squares_y,
        "square_length_m": options.square_mm / 1000.0,
        "marker_length_m": options.marker_mm / 1000.0,
        "dictionary": options.dictionary,
        "print": "A4 landscape, actual size / 100%, no fit-to-page",
    }
    yaml_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    print(f"Generated {pdf_path}")
    print(f"Generated {png_path}")
    print(f"Generated {yaml_path}")


if __name__ == "__main__":
    main()
