"""VMM-1 — Interactive ETS2 minimap ROI selector + HSV route-line tuner.

Workflow
--------
1.  Capture a frame from the ETS2 window (DXcam) or load a screenshot (-i).
2.  User selects the minimap ROI via cv2.selectROI().
3.  Six HSV trackbars let the user tune the mask until the red route line
    is cleanly isolated.
4.  Press S to save the result to truckpilot.toml as [vision.minimap].
5.  Press R to re-select the ROI, C to capture a fresh frame, Q to quit.

Output written to truckpilot.toml:
    [vision.minimap]
    x = <int>
    y = <int>
    w = <int>
    h = <int>
    hsv_lower = [H, S, V]
    hsv_upper = [H, S, V]
"""

from __future__ import annotations

import argparse
import logging
import sys
import textwrap
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger(__name__)

# ── constants ─────────────────────────────────────────────────────────────────

WINDOW_MAIN = "TruckPilot Minimap Calibration — press S=save R=roi C=capture Q=quit"
WINDOW_MASK = "HSV Mask Preview (live)"
WINDOW_HSV = "HSV Trackbars"

# Sensible defaults: red route line in ETS2 tends to be ~0-10 or 170-180 hue.
DEFAULT_HSV_LOWER = (0, 100, 100)
DEFAULT_HSV_UPPER = (10, 255, 255)

# Path to project root truckpilot.toml, relative to this file's repo location.
_THIS_FILE = Path(__file__).resolve()
_REPO_ROOT = _THIS_FILE.parents[4]  # tools/minimap_calibration/src/minimap_calibration/
TOML_PATH = _REPO_ROOT / "truckpilot.toml"


# ── screen capture ────────────────────────────────────────────────────────────

def _find_window_rect(title: str) -> tuple[int, int, int, int] | None:
    if sys.platform != "win32":
        return None
    try:
        import win32gui
    except ImportError:
        log.warning("pywin32 not installed; will capture full screen")
        return None

    title_l = title.lower()
    matches: list[int] = []

    def _cb(hwnd: int, _: object) -> bool:
        if win32gui.IsWindowVisible(hwnd):
            text = win32gui.GetWindowText(hwnd)
            if text and title_l in text.lower():
                matches.append(hwnd)
        return True

    win32gui.EnumWindows(_cb, None)
    if not matches:
        return None
    hwnd = matches[0]
    rect = win32gui.GetClientRect(hwnd)
    lt = win32gui.ClientToScreen(hwnd, (0, 0))
    rb = win32gui.ClientToScreen(hwnd, (rect[2], rect[3]))
    return (lt[0], lt[1], rb[0], rb[1])


def grab_frame(window_title: str = "Euro Truck Simulator 2") -> np.ndarray | None:
    """Grab one frame via DXcam. Returns BGR ndarray or None on failure."""
    try:
        import dxcam  # type: ignore[import-not-found]
    except ImportError:
        log.error("dxcam not installed. Install with: pip install dxcam")
        return None

    region = _find_window_rect(window_title)
    if region is None:
        log.warning("ETS2 window not found — capturing full primary display")

    cam = dxcam.create(output_color="BGR")
    if cam is None:
        log.error("dxcam.create() returned None — no DXGI device available")
        return None

    frame = cam.grab(region=region) if region else cam.grab()
    try:
        cam.release()
    except Exception:
        pass
    return frame  # may be None if dxcam returned no frame


# ── HSV mask application ──────────────────────────────────────────────────────

def apply_hsv_mask(roi_bgr: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Return a binary mask isolating pixels in [lower, upper] HSV range.

    Handles red hue wrap-around: if lower[0] > upper[0] the mask is split
    at hue=0/180 and OR-merged (e.g. H 170-180 ∪ H 0-10).
    """
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        # wrap-around (e.g. red: 170-180 + 0-10)
        mask_lo = cv2.inRange(hsv, np.array([0, lower[1], lower[2]]), upper)
        mask_hi = cv2.inRange(hsv, lower, np.array([179, upper[1], upper[2]]))
        mask = cv2.bitwise_or(mask_lo, mask_hi)

    # clean up noise
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


# ── TOML writer ───────────────────────────────────────────────────────────────

def _section_block(x: int, y: int, w: int, h: int,
                   lower: tuple[int, int, int], upper: tuple[int, int, int]) -> str:
    return textwrap.dedent(f"""\
        [vision.minimap]
        x = {x}
        y = {y}
        w = {w}
        h = {h}
        hsv_lower = [{lower[0]}, {lower[1]}, {lower[2]}]
        hsv_upper = [{upper[0]}, {upper[1]}, {upper[2]}]
    """)


def save_to_toml(toml_path: Path, x: int, y: int, w: int, h: int,
                 lower: tuple[int, int, int], upper: tuple[int, int, int]) -> None:
    """Insert or replace [vision.minimap] section in truckpilot.toml."""
    new_block = _section_block(x, y, w, h, lower, upper)

    if not toml_path.exists():
        toml_path.write_text(new_block, encoding="utf-8")
        log.info("created %s with [vision.minimap]", toml_path)
        return

    text = toml_path.read_text(encoding="utf-8")

    # Remove existing [vision.minimap] block if present.
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    inside = False
    for line in lines:
        stripped = line.strip()
        if stripped == "[vision.minimap]":
            inside = True
            continue
        if inside:
            # next section header ends the block
            if stripped.startswith("[") and not stripped.startswith("[vision.minimap"):
                inside = False
            else:
                continue
        out.append(line)

    # Ensure trailing newline before appending.
    result = "".join(out).rstrip("\n") + "\n\n" + new_block
    toml_path.write_text(result, encoding="utf-8")
    log.info("updated %s — [vision.minimap] written", toml_path)


# ── interactive calibration loop ──────────────────────────────────────────────

def _make_trackbar_window(lower: tuple[int, int, int], upper: tuple[int, int, int]) -> None:
    cv2.namedWindow(WINDOW_HSV, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_HSV, 400, 250)
    cv2.createTrackbar("H_min", WINDOW_HSV, lower[0], 179, lambda _: None)
    cv2.createTrackbar("H_max", WINDOW_HSV, upper[0], 179, lambda _: None)
    cv2.createTrackbar("S_min", WINDOW_HSV, lower[1], 255, lambda _: None)
    cv2.createTrackbar("S_max", WINDOW_HSV, upper[1], 255, lambda _: None)
    cv2.createTrackbar("V_min", WINDOW_HSV, lower[2], 255, lambda _: None)
    cv2.createTrackbar("V_max", WINDOW_HSV, upper[2], 255, lambda _: None)


def _read_trackbars() -> tuple[np.ndarray, np.ndarray]:
    h_min = cv2.getTrackbarPos("H_min", WINDOW_HSV)
    h_max = cv2.getTrackbarPos("H_max", WINDOW_HSV)
    s_min = cv2.getTrackbarPos("S_min", WINDOW_HSV)
    s_max = cv2.getTrackbarPos("S_max", WINDOW_HSV)
    v_min = cv2.getTrackbarPos("V_min", WINDOW_HSV)
    v_max = cv2.getTrackbarPos("V_max", WINDOW_HSV)
    lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
    upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
    return lower, upper


def select_roi(frame: np.ndarray) -> tuple[int, int, int, int] | None:
    """Open selectROI dialog. Returns (x, y, w, h) in frame coords or None."""
    print("\n>>> Drag to select the minimap region, then press ENTER or SPACE. ESC to cancel.")
    r = cv2.selectROI("Select Minimap ROI", frame, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Select Minimap ROI")
    x, y, w, h = int(r[0]), int(r[1]), int(r[2]), int(r[3])
    if w < 10 or h < 10:
        print("Selection too small or cancelled.")
        return None
    return x, y, w, h


def run_calibration(
    frame: np.ndarray,
    toml_path: Path,
    initial_roi: tuple[int, int, int, int] | None = None,
) -> bool:
    """Main interactive calibration loop. Returns True if user saved."""

    roi = initial_roi
    saved = False

    # First ROI selection if not pre-set.
    if roi is None:
        roi = select_roi(frame)
        if roi is None:
            print("No ROI selected. Exiting.")
            return False

    _make_trackbar_window(DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)

    print("\nControls:")
    print("  S — save to truckpilot.toml")
    print("  R — re-select ROI")
    print("  C — capture a new frame from ETS2")
    print("  Q — quit without saving")

    while True:
        x, y, w, h = roi
        roi_bgr = frame[y:y + h, x:x + w].copy()

        lower, upper = _read_trackbars()
        mask = apply_hsv_mask(roi_bgr, lower, upper)

        # Overlay: green pixels = detected route line.
        overlay = roi_bgr.copy()
        overlay[mask > 0] = (0, 255, 0)

        # Draw ROI rect on full frame preview.
        preview = frame.copy()
        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 255), 2)
        # Downscale preview to fit screen.
        ph, pw = preview.shape[:2]
        scale = min(1280 / pw, 720 / ph, 1.0)
        if scale < 1.0:
            preview = cv2.resize(preview, (int(pw * scale), int(ph * scale)), interpolation=cv2.INTER_AREA)

        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        side_by_side = np.hstack([overlay, mask_rgb])

        # Status text.
        status = (f"ROI ({x},{y}) {w}x{h}  |  "
                  f"H [{lower[0]}-{upper[0]}]  "
                  f"S [{lower[1]}-{upper[1]}]  "
                  f"V [{lower[2]}-{upper[2]}]  |  "
                  f"matched px: {int(mask.sum() // 255)}")
        cv2.putText(preview, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow(WINDOW_MAIN, preview)
        cv2.imshow(WINDOW_MASK, side_by_side)

        # Trackbar window needs a dummy image to stay visible.
        dummy = np.zeros((10, 400, 3), dtype=np.uint8)
        cv2.imshow(WINDOW_HSV, dummy)

        key = cv2.waitKey(30) & 0xFF

        if key in (ord("q"), ord("Q"), 27):  # Q or ESC
            print("Quit without saving.")
            break

        elif key in (ord("s"), ord("S")):
            lo = (int(lower[0]), int(lower[1]), int(lower[2]))
            hi = (int(upper[0]), int(upper[1]), int(upper[2]))
            save_to_toml(toml_path, x, y, w, h, lo, hi)
            print(f"\nSaved to {toml_path}")
            print(_section_block(x, y, w, h, lo, hi))
            saved = True
            break

        elif key in (ord("r"), ord("R")):
            new_roi = select_roi(frame)
            if new_roi is not None:
                roi = new_roi

        elif key in (ord("c"), ord("C")):
            print("Capturing new frame from ETS2...")
            new_frame = grab_frame()
            if new_frame is not None:
                frame = new_frame
                print("New frame captured.")
            else:
                print("Capture failed — keeping current frame.")

    cv2.destroyAllWindows()
    return saved


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    parser = argparse.ArgumentParser(
        description="TruckPilot VMM-1: Minimap ROI + HSV calibration tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              # Capture from running ETS2 and calibrate:
              python -m minimap_calibration

              # Load existing screenshot instead of capturing:
              python -m minimap_calibration -i screenshot.png

              # Write to a different toml file:
              python -m minimap_calibration --toml /path/to/truckpilot.toml
        """),
    )
    parser.add_argument(
        "-i", "--image",
        type=Path,
        default=None,
        help="Path to a screenshot PNG/JPG to use instead of live capture.",
    )
    parser.add_argument(
        "--toml",
        type=Path,
        default=TOML_PATH,
        help=f"Path to truckpilot.toml (default: {TOML_PATH})",
    )
    parser.add_argument(
        "--roi",
        type=str,
        default=None,
        metavar="X,Y,W,H",
        help="Pre-set ROI as comma-separated integers (skips interactive ROI selection).",
    )
    parser.add_argument(
        "--window",
        default="Euro Truck Simulator 2",
        help="Window title substring used to locate the ETS2 game window.",
    )
    args = parser.parse_args()

    # Load or capture frame.
    if args.image is not None:
        frame = cv2.imread(str(args.image))
        if frame is None:
            print(f"ERROR: cannot load image: {args.image}", file=sys.stderr)
            sys.exit(1)
        print(f"Loaded frame from {args.image} ({frame.shape[1]}x{frame.shape[0]})")
    else:
        print("Capturing frame from ETS2 (make sure the game is visible)...")
        frame = grab_frame(args.window)
        if frame is None:
            print(
                "ERROR: could not capture frame.\n"
                "  • Make sure ETS2 is running and visible.\n"
                "  • Or pass -i <screenshot.png> to load a static image.",
                file=sys.stderr,
            )
            sys.exit(1)
        print(f"Captured: {frame.shape[1]}x{frame.shape[0]}")

    # Parse pre-set ROI if given.
    initial_roi: tuple[int, int, int, int] | None = None
    if args.roi is not None:
        try:
            parts = [int(v.strip()) for v in args.roi.split(",")]
            if len(parts) != 4:
                raise ValueError
            initial_roi = (parts[0], parts[1], parts[2], parts[3])
        except ValueError:
            print("ERROR: --roi must be four integers: X,Y,W,H", file=sys.stderr)
            sys.exit(1)

    saved = run_calibration(frame, args.toml, initial_roi)
    sys.exit(0 if saved else 1)
