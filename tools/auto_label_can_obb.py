#!/usr/bin/env python3
"""Create reviewed YOLO OBB bootstrap labels for the red can dataset."""

from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
import random
import shutil

import cv2
import numpy as np

from object_pose_dataset import atomic_write_json, file_sha256, load_json
from bootstrap_top_pen_obb_labels import (
    draw_review,
    label_text as pen_obb_label_text,
    order_box_points,
    write_review_sheets,
)


EXPECTED_CAN_ASPECT = 0.12244 / 0.053


def red_mask(image: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    hsv_red = (
        ((hue <= 14) | (hue >= 166))
        & (saturation >= 55)
        & (value >= 28)
    )
    blue, green, red = cv2.split(image)
    red_float = red.astype(np.float32)
    channel_red = (
        (red >= 38)
        & (red_float >= green.astype(np.float32) * 1.25)
        & (red_float >= blue.astype(np.float32) * 1.18)
    )
    mask = np.where(hsv_red | channel_red, 255, 0).astype(np.uint8)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
    )
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )


def detect_red_can_obb(image: np.ndarray) -> tuple[np.ndarray, dict]:
    mask = red_mask(image)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    candidates = []
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < 250:
            continue
        rectangle = cv2.minAreaRect(cv2.convexHull(contour))
        (_, _), (width, height), _ = rectangle
        short_side = min(float(width), float(height))
        long_side = max(float(width), float(height))
        if short_side < 8 or long_side < 24:
            continue
        aspect = long_side / short_side
        # A can placed at the front edge can span a little over 220 px in the
        # 640x480 capture.  Keep the upper bound below large table/floor blobs.
        if not 42.0 <= long_side <= 240.0:
            continue
        if not 14.0 <= short_side <= 120.0:
            continue
        if not 1.35 <= aspect <= 4.0:
            continue
        fill_ratio = contour_area / max(float(width * height), 1.0)
        # Oversized edge contours also occur where the reddish wood floor is
        # clipped by the image.  A close can remains a dense rectangular blob.
        if long_side > 190.0 and fill_ratio < 0.65:
            continue
        aspect_score = math.exp(-abs(math.log(aspect / EXPECTED_CAN_ASPECT)))
        score = contour_area * (0.5 + fill_ratio) * aspect_score
        candidates.append((score, contour_area, fill_ratio, rectangle))
    if not candidates:
        raise ValueError("no red can candidate")

    score, contour_area, fill_ratio, rectangle = max(
        candidates,
        key=lambda item: item[0],
    )
    center, (width, height), angle = rectangle
    if width >= height:
        expanded_size = (float(width) * 1.12, float(height) * 1.08)
    else:
        expanded_size = (float(width) * 1.08, float(height) * 1.12)
    expanded = (center, expanded_size, angle)
    corners = order_box_points(cv2.boxPoints(expanded).astype(np.float32))
    image_height, image_width = image.shape[:2]
    corners[:, 0] = np.clip(corners[:, 0], 0, image_width - 1)
    corners[:, 1] = np.clip(corners[:, 1], 0, image_height - 1)
    side_a = float(np.linalg.norm(corners[1] - corners[0]))
    side_b = float(np.linalg.norm(corners[2] - corners[1]))
    aspect = max(side_a, side_b) / max(min(side_a, side_b), 1e-6)
    touches_border = bool(
        np.any(corners[:, 0] <= 1)
        or np.any(corners[:, 0] >= image_width - 2)
        or np.any(corners[:, 1] <= 1)
        or np.any(corners[:, 1] >= image_height - 2)
    )
    metrics = {
        "score": score,
        "contour_area_px": contour_area,
        "fill_ratio": fill_ratio,
        "box_aspect_ratio": aspect,
        "touches_image_border": touches_border,
        "review_required": bool(
            contour_area < 500
            or fill_ratio < 0.42
            or not 1.55 <= aspect <= 3.35
            or touches_border
        ),
    }
    return corners, metrics


def yolo_obb_line(corners: np.ndarray, width: int, height: int) -> str:
    return pen_obb_label_text(corners, width, height)


def assign_splits(items: list[dict], train_fraction: float, seed: int) -> None:
    strata: dict[tuple[bool, str], list[dict]] = collections.defaultdict(list)
    for item in items:
        strata[(item["requires_can_obb"], item["lighting"])].append(item)
    generator = random.Random(seed)
    for values in strata.values():
        generator.shuffle(values)
        validation_count = max(1, round(len(values) * (1.0 - train_fraction)))
        validation_count = min(validation_count, len(values) - 1)
        for index, item in enumerate(values):
            item["split"] = "val" if index < validation_count else "train"


def contact_sheet(overlays: list[tuple[Path, str]], output: Path) -> None:
    tile_width, image_height, label_height, columns = 192, 144, 32, 10
    rows = math.ceil(len(overlays) / columns)
    sheet = np.full(
        (rows * (image_height + label_height), columns * tile_width, 3),
        245,
        dtype=np.uint8,
    )
    for index, (overlay_path, label) in enumerate(overlays):
        image = cv2.imread(str(overlay_path))
        if image is None:
            continue
        image = cv2.resize(image, (tile_width, image_height))
        row, column = divmod(index, columns)
        x = column * tile_width
        y = row * (image_height + label_height)
        sheet[y : y + image_height, x : x + tile_width] = image
        cv2.putText(
            sheet,
            label[:30],
            (x + 3, y + image_height + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.31,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            sheet,
            label[30:60],
            (x + 3, y + image_height + 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.31,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(str(output), sheet):
        raise RuntimeError(f"failed to write {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", required=True, type=Path)
    parser.add_argument("--source-images", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--train-fraction", default=0.8, type=float)
    parser.add_argument("--seed", default=20260815, type=int)
    parser.add_argument(
        "--write-labels",
        action="store_true",
        help="write train/val images and labels after proposal review",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.5 <= args.train_fraction < 1.0:
        raise ValueError("--train-fraction must be in [0.5, 1.0)")
    selection = load_json(args.selection.resolve())
    source_images = args.source_images.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    if args.write_labels:
        for split in ("train", "val"):
            (output_dir / "images" / split).mkdir(parents=True)
            (output_dir / "labels" / split).mkdir(parents=True)
    overlays_dir = output_dir / "overlays"
    overlays_dir.mkdir()

    items = [dict(item) for item in selection["images"]]
    assign_splits(items, args.train_fraction, args.seed)
    overlays = []
    review_images = []
    failures = []
    for item in items:
        image_path = source_images / item["selected_image"]
        if file_sha256(image_path) != item["image_sha256"]:
            raise ValueError(f"source image hash mismatch: {image_path}")
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f"failed to decode image: {image_path}")
        height, width = image.shape[:2]
        label_text = ""
        overlay = image.copy()
        if item["requires_can_obb"]:
            try:
                corners, metrics = detect_red_can_obb(image)
                label_text = yolo_obb_line(corners, width, height)
                cv2.polylines(
                    overlay,
                    [np.round(corners).astype(np.int32)],
                    True,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                item["auto_label_metrics"] = metrics
                proposal = {
                    "points": corners,
                    "score": metrics["score"],
                }
                overlay = draw_review(
                    image,
                    item["capture_id"],
                    proposal,
                )
                if metrics["review_required"]:
                    failures.append(item["capture_id"])
            except ValueError as error:
                item["auto_label_error"] = str(error)
                failures.append(item["capture_id"])
        else:
            cv2.putText(
                overlay,
                "NEGATIVE - NO CAN BOX",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

        if args.write_labels:
            split = item["split"]
            output_image = output_dir / "images" / split / image_path.name
            output_label = (
                output_dir / "labels" / split / f"{image_path.stem}.txt"
            )
            shutil.copy2(image_path, output_image)
            output_label.write_text(label_text, encoding="utf-8")
        overlay_path = overlays_dir / f"{item['capture_id']}.jpg"
        if not cv2.imwrite(str(overlay_path), overlay):
            raise RuntimeError(f"failed to write {overlay_path}")
        overlays.append((overlay_path, item["capture_id"]))
        review_images.append(overlay)

    if args.write_labels:
        dataset_yaml = (
            f"path: {output_dir}\n"
            "train: images/train\n"
            "val: images/val\n\n"
            "names:\n"
            "  0: can\n"
        )
        (output_dir / "can_obb.yaml").write_text(
            dataset_yaml,
            encoding="utf-8",
        )
    contact_sheet(overlays, output_dir / "autolabel_contact_sheet.jpg")
    write_review_sheets(review_images, output_dir / "review_sheets")
    atomic_write_json(
        output_dir / "autolabel_manifest.json",
        {
            "schema_version": 1,
            "class_names": ["can"],
            "seed": args.seed,
            "train_fraction": args.train_fraction,
            "labels_written": bool(args.write_labels),
            "image_count": len(items),
            "positive_count": sum(item["requires_can_obb"] for item in items),
            "negative_count": sum(not item["requires_can_obb"] for item in items),
            "train_count": sum(item["split"] == "train" for item in items),
            "val_count": sum(item["split"] == "val" for item in items),
            "review_required": sorted(set(failures)),
            "images": items,
        },
    )
    print(
        "CAN_OBB_AUTOLABEL_PASS "
        f"images={len(items)} labels_written={int(args.write_labels)} "
        f"review_required={len(set(failures))} "
        f"output={output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
