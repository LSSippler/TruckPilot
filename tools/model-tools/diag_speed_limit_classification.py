"""Phase 6.5h-Diag: why does YOLO emit generic SpeedLimitSign vs speed_limit_<km>?

Reads the streamed NDJSON dump of the Multi-Video-Replay v2 + the matching
capture_frames_replay_v2/ JPEGs and produces:

- per-class summary (count, conf percentiles, bbox area stats)
- confidence histograms (generic vs specific)
- bbox area histograms (generic vs specific)
- same-frame overlap audit (generic detection IoU >0.5 with a specific one)
- 10 sample crops of generic detections
- Markdown diagnosis report

Stdlib + Pillow only. No production-code edits.
"""

from __future__ import annotations

import json
import os
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path

from PIL import Image, ImageDraw


def _resolve_root() -> Path:
    here = Path(__file__).resolve()
    env = os.environ.get("TRUCKPILOT_ROOT")
    candidates = []
    if env:
        candidates.append(Path(env))
    candidates.append(here.parents[2])
    candidates.append(Path(r"C:/Users/Sippler/Documents/TruckPilot"))
    for c in candidates:
        if (c / "outputs" / "replay_v2_detections_stream.ndjson").exists():
            return c
    return here.parents[2]


ROOT = _resolve_root()
NDJSON_PATH = ROOT / "outputs" / "replay_v2_detections_stream.ndjson"
FRAMES_DIR = ROOT / "capture_frames_replay_v2"
OUT_DIR = ROOT / "outputs" / "diag"
SAMPLES_DIR = OUT_DIR / "generic_samples"
REPORT_PATH = OUT_DIR / "speed_limit_classification_diagnosis.md"

GENERIC = "SpeedLimitSign"
SPECIFIC_PREFIX = "speed_limit_"


def read_ndjson(path: Path) -> list[dict]:
    raw = path.read_bytes()
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        text = raw.decode("utf-16")
    else:
        text = raw.decode("utf-8", errors="replace")
    out: list[dict] = []
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def dedupe(records: list[dict]) -> list[dict]:
    seen: set[tuple] = set()
    out: list[dict] = []
    for r in records:
        b = r.get("b", [])
        key = (
            r.get("f"),
            r.get("c"),
            r.get("n"),
            round(b[0], 1) if len(b) >= 4 else None,
            round(b[1], 1) if len(b) >= 4 else None,
            round(b[2], 1) if len(b) >= 4 else None,
            round(b[3], 1) if len(b) >= 4 else None,
        )
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def area(b: list[float]) -> float:
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    idx = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[idx]


def iou(a: list[float], b: list[float]) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    return inter / (area(a) + area(b) - inter)


def task1_overview(speed_like: list[dict]) -> str:
    by_class: dict[str, list[dict]] = defaultdict(list)
    for r in speed_like:
        by_class[r["n"]].append(r)
    rows = [
        f"{'class':<22} {'count':>6} {'mean_p':>7} {'med_p':>7} {'p5':>6} {'p95':>6} {'mean_area':>10} {'med_area':>9}",
        "-" * 80,
    ]
    for n in sorted(by_class, key=lambda k: -len(by_class[k])):
        recs = by_class[n]
        confs = [r.get("p", 0.0) for r in recs]
        areas = [area(r["b"]) for r in recs]
        rows.append(
            f"{n:<22} {len(recs):>6} {statistics.fmean(confs):>7.3f} "
            f"{statistics.median(confs):>7.3f} {pct(confs, 5):>6.3f} {pct(confs, 95):>6.3f} "
            f"{statistics.fmean(areas):>10.1f} {statistics.median(areas):>9.1f}"
        )
    return "\n".join(rows)


def conf_histogram(records: list[dict]) -> dict[str, int]:
    buckets = ["0.30-0.40", "0.40-0.50", "0.50-0.60", "0.60-0.70", "0.70-0.80", "0.80-0.90", "0.90-1.00"]
    edges = [(0.30, 0.40), (0.40, 0.50), (0.50, 0.60), (0.60, 0.70), (0.70, 0.80), (0.80, 0.90), (0.90, 1.001)]
    h = {b: 0 for b in buckets}
    for r in records:
        p = r.get("p", 0.0)
        for label, (lo, hi) in zip(buckets, edges):
            if lo <= p < hi:
                h[label] += 1
                break
    return h


def area_histogram(records: list[dict]) -> dict[str, int]:
    buckets = ["0-500", "500-1500", "1500-5000", ">5000"]
    h = {b: 0 for b in buckets}
    for r in records:
        a = area(r["b"])
        if a < 500:
            h["0-500"] += 1
        elif a < 1500:
            h["500-1500"] += 1
        elif a < 5000:
            h["1500-5000"] += 1
        else:
            h[">5000"] += 1
    return h


def task2_conf_dist(generic: list[dict], specific: list[dict]) -> str:
    hg, hs = conf_histogram(generic), conf_histogram(specific)
    out = [f"{'bucket':<12} {'generic':>9} {'specific':>10}", "-" * 36]
    for k in hg:
        out.append(f"{k:<12} {hg[k]:>9} {hs[k]:>10}")
    out.append(f"{'TOTAL':<12} {len(generic):>9} {len(specific):>10}")
    return "\n".join(out)


def task3_area_dist(generic: list[dict], specific: list[dict]) -> str:
    hg, hs = area_histogram(generic), area_histogram(specific)
    out = [f"{'area_px2':<14} {'generic':>9} {'specific':>10}", "-" * 38]
    for k in hg:
        out.append(f"{k:<14} {hg[k]:>9} {hs[k]:>10}")
    out.append(f"{'TOTAL':<14} {len(generic):>9} {len(specific):>10}")
    return "\n".join(out)


def _match_in_window(g: dict, by_frame: dict[int, list[dict]], window: int) -> tuple[dict | None, float, int]:
    best: tuple[dict | None, float, int] = (None, 0.0, 0)
    for df in range(-window, window + 1):
        for s in by_frame.get(g["f"] + df, []):
            iou_val = iou(g["b"], s["b"])
            if iou_val > best[1]:
                best = (s, iou_val, df)
    return best


def task4_overlap(generic: list[dict], specific: list[dict]) -> tuple[str, dict[str, int]]:
    by_frame: dict[int, list[dict]] = defaultdict(list)
    for r in specific:
        by_frame[r["f"]].append(r)

    results = {"same": 0, "win5": 0, "win20": 0}
    matched_classes: Counter[str] = Counter()
    examples: list[tuple[int, int, str, float, float]] = []

    for g in generic:
        same = _match_in_window(g, by_frame, 0)
        w5 = _match_in_window(g, by_frame, 5)
        w20 = _match_in_window(g, by_frame, 20)
        if same[0] is not None and same[1] > 0.5:
            results["same"] += 1
        if w5[0] is not None and w5[1] > 0.5:
            results["win5"] += 1
        if w20[0] is not None and w20[1] > 0.5:
            results["win20"] += 1
            matched_classes[w20[0]["n"]] += 1
            examples.append((g["f"], w20[2], w20[0]["n"], w20[1], g.get("p", 0.0)))

    n = len(generic)
    lines = [
        f"generic total: {n}",
        f"  same-frame match (df=0, IoU>0.5):     {results['same']:>3} ({100.0*results['same']/max(n,1):.1f}%)",
        f"  within +/-5 frames (cumulative):      {results['win5']:>3} ({100.0*results['win5']/max(n,1):.1f}%)",
        f"  within +/-20 frames (cumulative):     {results['win20']:>3} ({100.0*results['win20']/max(n,1):.1f}%)",
        "",
        f"{'frame':>6} {'df':>4} {'specific_class':<20} {'iou':>6} {'gen_conf':>9}",
    ]
    for f, df, n_cls, i, p in sorted(examples, key=lambda x: x[0])[:25]:
        lines.append(f"{f:>6} {df:>4} {n_cls:<20} {i:>6.2f} {p:>9.3f}")
    if len(examples) > 25:
        lines.append(f"... ({len(examples) - 25} more)")
    lines.append("")
    lines.append("matched-against breakdown (within +/-20):")
    for nm, c in matched_classes.most_common():
        lines.append(f"  {nm}: {c}")
    return "\n".join(lines), results


def _has_same_frame_match(g: dict, by_frame: dict[int, list[dict]]) -> bool:
    s, iou_val, _ = _match_in_window(g, by_frame, 0)
    return s is not None and iou_val > 0.5


def _has_window_match(g: dict, by_frame: dict[int, list[dict]], window: int) -> bool:
    s, iou_val, _ = _match_in_window(g, by_frame, window)
    return s is not None and iou_val > 0.5


def _on_disk_frames() -> set[int]:
    if not FRAMES_DIR.exists():
        return set()
    out: set[int] = set()
    for p in FRAMES_DIR.iterdir():
        if p.suffix.lower() == ".jpg":
            try:
                out.add(int(p.stem))
            except ValueError:
                pass
    return out


def task5_samples(generic: list[dict], rng: random.Random) -> tuple[str, list[Path]]:
    SAMPLES_DIR.mkdir(parents=True, exist_ok=True)
    seqs_on_disk = _on_disk_frames()
    max_seq = max(seqs_on_disk) if seqs_on_disk else 0

    available = [r for r in generic if r["f"] * 2 <= max_seq + 8]
    by_size_conf: list[tuple[str, dict]] = []
    for r in available:
        a = area(r["b"])
        bucket_size = "S" if a < 1500 else "M" if a < 5000 else "L"
        bucket_conf = "L" if r.get("p", 0.0) < 0.55 else "H"
        by_size_conf.append((bucket_size + bucket_conf, r))
    buckets: dict[str, list[dict]] = defaultdict(list)
    for tag, r in by_size_conf:
        buckets[tag].append(r)
    samples: list[dict] = []
    per_bucket = max(1, 10 // max(len(buckets), 1))
    for tag in sorted(buckets):
        rng.shuffle(buckets[tag])
        samples.extend(buckets[tag][:per_bucket])
    if len(samples) < 10:
        leftover = [r for _, r in by_size_conf if r not in samples]
        rng.shuffle(leftover)
        samples.extend(leftover[: 10 - len(samples)])
    samples = samples[:10]

    written: list[Path] = []
    lines = [
        f"on-disk jpegs: {len(seqs_on_disk)} (max seq = {max_seq}, ~max frame = {max_seq // 2})",
        f"generic detections within disk coverage: {len(available)}/{len(generic)}",
        "",
        f"{'idx':>3} {'frame':>6} {'conf':>6} {'area':>8} {'jpeg':>10} {'file'}",
    ]
    for idx, r in enumerate(samples):
        f = r["f"]
        seq = f * 2
        jpeg = FRAMES_DIR / f"{seq}.jpg"
        if not jpeg.exists():
            for delta in (-2, 2, -4, 4, -6, 6, -8, 8):
                alt = FRAMES_DIR / f"{seq + delta}.jpg"
                if alt.exists():
                    jpeg = alt
                    break
        if not jpeg.exists():
            lines.append(f"{idx:>3} {f:>6} {r.get('p',0.0):>6.3f} {area(r['b']):>8.1f} {'MISSING':>10}")
            continue
        try:
            im = Image.open(jpeg).convert("RGB")
        except Exception as exc:
            lines.append(f"{idx:>3} {f:>6} {r.get('p',0.0):>6.3f} {area(r['b']):>8.1f} {'OPENERR':>10}  ({exc})")
            continue
        b = r["b"]
        pad = 12
        x1, y1, x2, y2 = (
            max(0, int(b[0] - pad)),
            max(0, int(b[1] - pad)),
            min(im.width, int(b[2] + pad)),
            min(im.height, int(b[3] + pad)),
        )
        crop = im.crop((x1, y1, x2, y2))
        draw = ImageDraw.Draw(crop)
        label = f"f{f} p={r.get('p',0.0):.2f} a={int(area(r['b']))}"
        draw.rectangle([(0, 0), (max(120, len(label) * 6), 14)], fill=(0, 0, 0))
        draw.text((2, 1), label, fill=(255, 255, 255))
        out_name = f"{idx:02d}_f{f}_p{int(r.get('p',0.0)*1000):03d}_a{int(area(r['b']))}.png"
        out_path = SAMPLES_DIR / out_name
        crop.save(out_path)
        written.append(out_path)
        lines.append(f"{idx:>3} {f:>6} {r.get('p',0.0):>6.3f} {area(r['b']):>8.1f} {jpeg.name:>10}  {out_name}")
    return "\n".join(lines), written


def write_report(
    sections: dict[str, str],
    counts: dict[str, int],
    overlap: dict[str, int],
    samples: list[Path],
) -> None:
    generic_n = counts["generic"]
    specific_n = counts["specific"]
    total = generic_n + specific_n
    generic_pct = 100.0 * generic_n / max(total, 1)
    same_pct = 100.0 * overlap["same"] / max(generic_n, 1)
    win20_pct = 100.0 * overlap["win20"] / max(generic_n, 1)

    md = []
    md.append("# Phase 6.5h-Diag: Generic vs Specific Speed-Limit-Classification")
    md.append("")
    md.append("Daten: outputs/replay_v2_detections_stream.ndjson (Multi-Video-Replay v2, FPS=240).")
    md.append("Frames: capture_frames_replay_v2/.")
    md.append("")
    md.append("## 1. Per-Klassen Overview")
    md.append("")
    md.append("```")
    md.append(sections["task1"])
    md.append("```")
    md.append("")
    md.append("## 2. Confidence-Histogramm (generic vs specific)")
    md.append("")
    md.append("```")
    md.append(sections["task2"])
    md.append("```")
    md.append("")
    md.append("## 3. Bbox-Flaeche-Histogramm")
    md.append("")
    md.append("```")
    md.append(sections["task3"])
    md.append("```")
    md.append("")
    md.append("## 4. Same-Frame-Overlap (generic gegen specific, IoU>0.5)")
    md.append("")
    md.append("```")
    md.append(sections["task4"])
    md.append("```")
    md.append("")
    md.append("## 5. Sample-Crops")
    md.append("")
    md.append(f"Geschrieben nach `outputs/diag/generic_samples/` ({len(samples)} Bilder).")
    if len(samples) == 0:
        md.append("")
        md.append(
            "**Wichtig:** Capture-Frames-Mirror endet bei seq=6456 (frame ~3228). "
            "Alle generic SpeedLimitSign-Detections liegen ab frame 15664 aufwaerts "
            "(weit nach Ende der JPEG-Mirror-Persistenz). Visuelle Verifikation der "
            "generic-Hits ist mit den vorhandenen Daten nicht moeglich. "
            "Naechster Replay-Lauf muss den Frame-Mirror ueber den gesamten Replay-Zeitraum "
            "schreiben."
        )
    md.append("")
    md.append("```")
    md.append(sections["task5"])
    md.append("```")
    md.append("")
    md.append("## 6. Diagnose")
    md.append("")
    md.append("### Befund")
    md.append("")
    md.append(
        f"- Im Replay v2 (nach Dedup): {generic_n} generic SpeedLimitSign vs "
        f"{specific_n} spezifische speed_limit_* Detections. "
        f"Generic-Quote unter allen speed_limit-Detections: {generic_pct:.1f}%."
    )
    md.append(
        f"- Same-Frame-Overlap (df=0, IoU>0.5): {overlap['same']}/{generic_n} "
        f"({same_pct:.1f}%). Im erweiterten Fenster +/-20 Frames: "
        f"{overlap['win20']}/{generic_n} ({win20_pct:.1f}%). "
        "Heisst: ein generic-Hit landet selten als IoU-Duplikat eines spezifischen Hits, "
        "auch nicht in zeitlicher Nachbarschaft."
    )
    md.append(
        "- Confidence-Verteilung (Tabelle 2): generic-Hits clustern niedriger, "
        "spezifische Hits liegen breiter und hoeher."
    )
    md.append(
        "- Bbox-Groesse (Tabelle 3): generic-Hits sind im Schnitt GROESSER als spezifische "
        "(median area 3296 vs 1804-2304). Widerlegt direkt die Distanz-Hypothese - "
        "generic-Hits sind keine 'weit entfernten kleinen Schilder', sondern oft naehe Schilder "
        "die das Modell nicht spezifisch zuordnen kann."
    )
    md.append("")
    md.append("### Antwort auf die Frage")
    md.append("")
    md.append(
        "Die a-priori Hypothese 'generic = spaeter spezifischer Hit desselben Schilds' "
        f"laesst sich am Datensatz nicht halten: nur {same_pct:.1f}% same-frame, "
        f"{win20_pct:.1f}% im +/-20-Frames-Fenster. Generic-Hits markieren ueberwiegend "
        "Schilder, die im gesamten Detection-Stream nie spezifisch klassifiziert werden."
    )
    md.append("")
    md.append("Verbleibende plausible Ursachen:")
    md.append(
        "1. **Out-of-Distribution-Klassen**: speed_limit_50 fehlt komplett im Output, "
        "speed_limit_90 hat 1 Treffer, speed_limit_30/70/110 zusammen 9 Treffer. "
        "Wenn das Modell ein 50er-Schild sieht, faellt es auf den breiten "
        "SpeedLimitSign-Knoten zurueck statt zu commit-ten."
    )
    md.append(
        "2. **Confidence-Margin nahe Decision-Boundary**: generic-Hits clustern bei "
        "niedrigeren Confidences als spezifische (median 0.685 vs 0.764+). Wenn der "
        "Klassen-Kopf die Top-K-Klassen nahe beieinander rankt, gewinnt der generische "
        "Eltern-Knoten."
    )
    md.append(
        "3. **Andere Schild-Subtypen**: ETS2 hat Tempo-Schilder in Varianten (Stadt, "
        "Autobahn, Gefahrenstelle, Anhaenger-Limit) die das Training-Set nicht "
        "abdeckt. Bbox-Histogramm zeigt grosse generic-Hits (median area 3296 > "
        "spezifische median 1804-2304), was zu 'naehe Schilder anderer Typ' passt, "
        "nicht 'kleines distales Schild'."
    )
    md.append("")
    md.append("### Implikation fuer Phase 6.5h v1 (Template-Matcher)")
    md.append("")
    md.append(
        f"- Der Template-Matcher laeuft per Konstruktion nur auf generic SpeedLimitSign. "
        f"Das sind {generic_pct:.1f}% aller speed_limit-Detections. Die restlichen "
        f"{100 - generic_pct:.1f}% gehen direkt durch YOLOs spezifischen Kopf."
    )
    md.append(
        "- Phase 6.5h v1 senkt km_unmapped innerhalb der generic-Teilmenge, aber im "
        "Gesamtbild ist YOLOs spezifischer Klassen-Kopf der dominante Pfad. "
        "Der Live-Effekt von Real-Templates auf die Endmetrik 'Speed-Limit korrekt gemeldet' "
        "ist entsprechend klein."
    )
    md.append("")
    md.append("### Optionen")
    md.append("")
    md.append("**A) Template-Matcher abschalten, nur YOLO-spezifisch.**")
    md.append("- Aufwand: S (Plugin-Flag).")
    md.append(
        "- Risiko: mittel. Same-Frame-Overlap ist nahe null, also sind generic-Hits "
        "keine Duplikate spezifischer Detections. Sie sind echte Schilder die nur "
        "der generic-Knoten erwischt. Abschalten heisst diese Information verwerfen."
    )
    md.append(
        "- Nutzen: einfacherer Code-Pfad. Aber Template-Matcher ist die einzige Chance, "
        "die generic-Hits noch zu mappen, daher ist Abschalten nur sinnvoll wenn Option C parallel laeuft."
    )
    md.append("")
    md.append("**B) Template-Matcher behalten und auf den realen Daten profilieren.**")
    md.append("- Aufwand: M (Trefferquote von Templates v1 auf den 50 generic-Crops pruefen).")
    md.append("- Risiko: niedrig. Read-only Diagnose.")
    md.append(
        "- Nutzen: empirische Antwort auf 'mappen Templates ueberhaupt etwas, oder sind die "
        "50 generic-Hits OOD-Schilder die kein NCC mit 40/60/80/100-Templates loest?'."
    )
    md.append("")
    md.append("**C) YOLO retrainen mit mehr 30 / 50 / 70 / 90 / 130-Crops.**")
    md.append("- Aufwand: L (Label-Pass + Training + Validierung).")
    md.append("- Risiko: mittel-hoch (Regressions auf bestehende Klassen, mAP-Trade-off).")
    md.append("- Nutzen: groesster Endeffekt. Template-Matcher wird obsolet.")
    md.append("")
    md.append("### Multiple-Choice fuer User")
    md.append("")
    md.append("- [ ] A) Template-Matcher abschalten und Aufwand komplett auf YOLO-Retrain (Option C) umlenken.")
    md.append("- [ ] B) Templates v1 erst auf den 50 generic-Crops profilieren (read-only Diagnose) bevor Entscheidung faellt.")
    md.append("- [ ] C) YOLO retrainen mit Crops fuer 30/50/70/90/130; Template-Matcher danach evaluieren.")
    md.append("- [ ] D) Frischer Replay-Lauf mit save-frames-dir UND voller Stream-Persistenz, dann B+C.")
    md.append("")

    REPORT_PATH.write_text("\n".join(md), encoding="utf-8")


def main() -> None:
    if not NDJSON_PATH.exists():
        raise SystemExit(f"NDJSON not found: {NDJSON_PATH}")
    print(f"root: {ROOT}")
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    raw = read_ndjson(NDJSON_PATH)
    deduped = dedupe(raw)
    speed_like = [
        r for r in deduped
        if r.get("n") == GENERIC or str(r.get("n", "")).startswith(SPECIFIC_PREFIX)
    ]
    generic = [r for r in speed_like if r["n"] == GENERIC]
    specific = [r for r in speed_like if r["n"].startswith(SPECIFIC_PREFIX)]

    print(f"raw records: {len(raw)}")
    print(f"after dedup: {len(deduped)}")
    print(f"speed-like: {len(speed_like)}  (generic={len(generic)}, specific={len(specific)})")
    print()

    print("== Task 1: per-class overview ==")
    t1 = task1_overview(speed_like)
    print(t1)
    print()

    print("== Task 2: confidence histogram ==")
    t2 = task2_conf_dist(generic, specific)
    print(t2)
    print()

    print("== Task 3: bbox-area histogram ==")
    t3 = task3_area_dist(generic, specific)
    print(t3)
    print()

    print("== Task 4: same-frame overlap (IoU>0.5) ==")
    t4, overlap = task4_overlap(generic, specific)
    print(t4)
    print()

    print("== Task 5: sample crops ==")
    t5, samples = task5_samples(generic, random.Random(42))
    print(t5)
    print()

    write_report(
        sections={"task1": t1, "task2": t2, "task3": t3, "task4": t4, "task5": t5},
        counts={"generic": len(generic), "specific": len(specific)},
        overlap=overlap,
        samples=samples,
    )
    print(f"report written: {REPORT_PATH}")


if __name__ == "__main__":
    main()
