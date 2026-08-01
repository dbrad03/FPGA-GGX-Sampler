#!/usr/bin/env python3
"""Regression harness: one command per claim this project makes about itself.

Two claims, two subcommands.

    python sim/harness.py pnr [--note TEXT]
        Place-and-route the Lane and append one row to sim/timing_history.csv,
        keyed by commit SHA: WNS, TNS, failing endpoints, DSP, LUT, BRAM,
        CARRY4. Every timing number quoted in an issue, an ADR or a handoff note
        should be a row in that file rather than a figure transcribed by hand
        from a report that has since been overwritten.

    python sim/harness.py baseline capture|check
        Capture, or compare against, the exact Oct32 word test_ggx_control's
        Lane emits for its fixed stimulus. This is what gives "bit-identical"
        something to mean: a refactor that is supposed to preserve results
        exactly either reproduces every word or it does not.

Why resources are recorded next to slack: a build that quietly loses a source
file, or a $readmem ROM, still routes and still reports a perfectly plausible
WNS. Its DSP count is what gives it away -- which is why `pnr` refuses to record
a row whose DSP count moved from the previous row unless the run says why
(--note). See the ROM trap in docs/handoff.md.

The measurement flow itself lives in sim/harness_pnr.tcl. This file never
decides what a number means; it runs the tool, parses what the tool printed, and
guards what gets written down.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
from pathlib import Path
from typing import NamedTuple

SIM_DIR = Path(__file__).resolve().parent
PROJ_DIR = SIM_DIR.parent

HISTORY_PATH = SIM_DIR / "timing_history.csv"
BASELINE_PATH = SIM_DIR / "baselines" / "ggx_control_oct32.txt"
PNR_TCL = SIM_DIR / "harness_pnr.tcl"
RUNS_DIR = SIM_DIR / "harness_runs"

#: Printed by harness_pnr.tcl once the whole flow has run. Vivado exits 0 after
#: plenty of failures that matter, so this sentinel -- not the exit status -- is
#: what proves the run reached the end.
DONE_SENTINEL = "HARNESS_PNR_DONE"

#: Vivado's message when it cannot find a $readmem file. It is a warning, not an
#: error: the run continues, constant-folds the ROM away, and reports a
#: plausible WNS for a design missing a whole LUT. Never record such a run.
READMEM_FAILURE = "could not open $readmem data file"

#: The RTL $readmemh's its ROMs under bare filenames, which resolve against
#: whatever directory Vivado was launched in. Every .mem in sim/ is staged into
#: the run directory -- runs get their own directory so their reports stay with
#: them, which would otherwise put the ROMs out of reach. Which ROMs the build
#: actually requires is asserted by harness_pnr.tcl, where the CWD is; naming
#: them here as well would just be a second list to forget to update.
ROM_GLOB = "*.mem"

#: key -> parser. Order is the column order in the history file.
METRICS = {
    "wns": float,
    "tns": float,
    "failing_endpoints": int,
    "dsp": int,
    "lut": int,
    "bram": float,
    "carry4": int,
}


class HarnessError(Exception):
    """A run that must not be recorded, or a comparison that cannot be made."""


# ---------------------------------------------------------------- P&R metrics


def parse_metrics(log_text: str) -> dict:
    """Pull the `GGXMETRIC key value` lines out of a Vivado run's stdout.

    Raises rather than returning partial results: a half-parsed run recorded as
    a row is worse than no row, because the gap is invisible afterwards.
    """
    if READMEM_FAILURE in log_text:
        raise HarnessError(
            "the build could not read a $readmem ROM, so it synthesized an "
            "uninitialized ROM, constant-folded the logic behind it away, and "
            "measured a design that does not exist. Run: python sim/gen_roms.py"
        )
    if DONE_SENTINEL not in log_text:
        raise HarnessError(
            f"the P&R run never printed {DONE_SENTINEL} -- it died partway through "
            f"(Vivado exits 0 on many such failures; check the run log)"
        )

    found = {}
    for line in log_text.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == "GGXMETRIC":
            found[parts[1]] = parts[2]

    metrics = {}
    for key, cast in METRICS.items():
        if key not in found:
            raise HarnessError(f"the P&R run printed no value for '{key}'")
        try:
            metrics[key] = cast(found[key])
        except ValueError as e:
            raise HarnessError(f"'{key}' is not a number: {found[key]!r}") from e
    return metrics


@dataclass(frozen=True)
class Row:
    commit: str
    dirty: bool
    date: str
    wns: float
    tns: float
    failing_endpoints: int
    dsp: int
    lut: int
    bram: float
    carry4: int
    note: str = ""


_FIELD_CASTS = {"dirty": lambda s: s == "True", "note": str, "commit": str, "date": str}


def read_rows(path: Path) -> list[Row]:
    if not Path(path).exists():
        return []
    with open(path, newline="") as fh:
        out = []
        for raw in csv.DictReader(fh):
            kw = {}
            for f in fields(Row):
                cast = _FIELD_CASTS.get(f.name, METRICS.get(f.name, str))
                kw[f.name] = cast(raw[f.name])
            out.append(Row(**kw))
        return out


def dsp_deviation(prev: Row | None, new: Row) -> str | None:
    """The sentinel check. None when the DSP count held, a message when it moved."""
    if prev is None or prev.dsp == new.dsp:
        return None
    delta = new.dsp - prev.dsp
    return (
        f"DSP count moved {prev.dsp} -> {new.dsp} ({delta:+d}) since {prev.commit}. "
        f"An unintended move here usually means the build lost a source file or a "
        f"$readmem ROM -- in which case its WNS is meaningless, not good news."
    )


def append_row(path: Path, row: Row) -> str | None:
    """Append `row`, refusing an unexplained DSP change. Returns the deviation note."""
    path = Path(path)
    rows = read_rows(path)
    deviation = dsp_deviation(rows[-1] if rows else None, row)
    if deviation and not row.note:
        raise HarnessError(
            f"{deviation}\nRefusing to record it unexplained: re-run with "
            f"--note 'why the DSP count moved'."
        )

    new_file = not path.exists()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=[f.name for f in fields(Row)])
        if new_file:
            writer.writeheader()
        writer.writerow(asdict(row))
    return deviation


# ------------------------------------------------------------ Oct32 baseline

class Sample(NamedTuple):
    """One Sample as it left the Lane. The unit the baseline is a list of.

    `word` is the Oct32 output word verbatim, not a decoded direction: what
    "bit-identical" claims is that the same bits reached the stream.
    """

    burst: int
    idx: int
    word: int
    last: int


#: One Sample per line: burst index, index within the Burst, the Oct32 word as
#: emitted, TLAST. Nothing else -- no wall-clock time, no simulation time, no
#: path, no tool version. That is deliberate: it makes plain `diff` a complete
#: comparison, with nothing volatile to filter out first.
BASELINE_HEADER = """\
# test_ggx_control Oct32 output baseline.
#
# One line per Sample:  <burst> <sample_idx> <oct32_hex> <tlast>
# The Oct32 word is exactly what the Lane put on the output stream. ADR-0001
# fixes the encoding and its 2x16-bit width; the field ORDER is
# {field_w, field_u}, from hdl/axis_oct32_encode.sv.
#
# Captured and compared with:  python sim/harness.py baseline capture|check
# Contains no timestamp, no simulation time and no absolute path by design, so
# `diff` on this file is a complete comparison -- see docs/harness.md.
"""


def render_baseline(samples, header: bool = True) -> str:
    """The one place the baseline's line format is written. Both the file and
    test_ggx_control's dump go through here, so they cannot drift apart."""
    lines = [BASELINE_HEADER] if header else []
    for burst, idx, word, last in samples:
        lines.append(f"{burst} {idx} {word:08x} {last}\n")
    return "".join(lines)


def parse_baseline(text: str) -> list[Sample]:
    out = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 4:
            raise HarnessError(f"baseline line {lineno} is malformed: {line!r}")
        burst, idx, word, last = parts
        out.append(Sample(int(burst), int(idx), int(word, 16), int(last)))
    return out


def baseline_diff(expected, actual) -> list[str]:
    """Differences between two captures, most useful first.

    A length mismatch is reported alone: once the streams differ in length the
    per-Sample comparison is comparing different Samples, and pages of
    consequential mismatches would bury the one fact that matters.
    """
    if len(expected) != len(actual):
        return [
            f"Sample count differs: baseline has {len(expected)}, this run produced "
            f"{len(actual)} -- the output stream is truncated or over-long, so the "
            f"per-Sample comparison is not meaningful yet"
        ]

    out = []
    for i, (exp, got) in enumerate(zip(expected, actual)):
        e_burst, e_idx, e_word, e_last = exp
        g_burst, g_idx, g_word, g_last = got
        if (e_burst, e_idx) != (g_burst, g_idx):
            out.append(
                f"#{i}: position differs -- baseline burst={e_burst} idx={e_idx}, "
                f"this run burst={g_burst} idx={g_idx}"
            )
            continue
        if e_word != g_word:
            out.append(
                f"#{i} (burst={e_burst} idx={e_idx}): Oct32 word {e_word:08x} -> {g_word:08x}"
            )
        if e_last != g_last:
            out.append(f"#{i} (burst={e_burst} idx={e_idx}): TLAST {e_last} -> {g_last}")
    return out


# ------------------------------------------------------------------ plumbing


def git_state() -> tuple[str, bool]:
    def git(*args):
        return subprocess.run(
            ["git", *args], cwd=PROJ_DIR, capture_output=True, text=True, check=True
        ).stdout.strip()

    return git("rev-parse", "--short", "HEAD"), bool(git("status", "--porcelain"))


def _check_sources_current():
    r = subprocess.run(
        [sys.executable, str(SIM_DIR / "sources.py"), "--check-tcl"],
        cwd=PROJ_DIR, capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise HarnessError(
            "sim/rtl_sources.tcl is out of date with sim/sources.py, so the build "
            "would not be of this tree's RTL. Run: python sim/sources.py --emit-tcl\n"
            + r.stdout + r.stderr
        )


#: Run directories kept after a successful `pnr`. Each is only ~0.5 MB, so this
#: is about keeping the directory readable, not about disk. Enough to hold the
#: run you just did plus the couple you are comparing it against.
KEEP_RUNS = 5

#: What prune_runs is allowed to delete: the <UTC timestamp>-<short SHA> names
#: run_pnr creates, and nothing else. Deleting by pattern match is the one
#: genuinely destructive thing here, so the pattern is anchored and strict --
#: anything a person put in this directory by hand survives.
RUN_DIR_RE = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{7,40}$")


def prune_runs(runs_dir: Path, keep: int = KEEP_RUNS) -> list[Path]:
    """Delete all but the `keep` newest run directories. Returns what it removed.

    Newest by name, not by mtime: the names are UTC timestamps, so they sort
    chronologically, and unlike mtime they do not change when someone opens a
    report or the filesystem is restored from a backup.
    """
    if keep < 1:
        raise ValueError("keep must be at least 1 -- keep=0 would delete the run "
                         "whose row was just recorded, reports and all")
    runs_dir = Path(runs_dir)
    if not runs_dir.is_dir():
        return []

    runs = sorted(p for p in runs_dir.iterdir() if p.is_dir() and RUN_DIR_RE.match(p.name))
    doomed = runs[:-keep] if keep < len(runs) else []
    for path in doomed:
        shutil.rmtree(path)
    return doomed


def run_pnr(run_dir: Path) -> str:
    """Run the P&R flow with `run_dir` as CWD, so Vivado's reports land there."""
    run_dir.mkdir(parents=True, exist_ok=True)
    roms = sorted(SIM_DIR.glob(ROM_GLOB))
    if not roms:
        raise HarnessError(f"no {ROM_GLOB} in {SIM_DIR} -- run: python sim/gen_roms.py")
    for rom in roms:
        (run_dir / rom.name).write_bytes(rom.read_bytes())
    log_path = run_dir / "pnr.log"
    print(f"[harness] place-and-route running (~10 min); log: {log_path}")
    with open(log_path, "w") as log:
        subprocess.run(
            ["vivado", "-mode", "batch", "-notrace", "-source", str(PNR_TCL)],
            cwd=run_dir, stdout=log, stderr=subprocess.STDOUT, check=False,
        )
    return log_path.read_text(errors="replace")


def cmd_pnr(args) -> int:
    _check_sources_current()
    commit, dirty = git_state()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_text = run_pnr(RUNS_DIR / f"{stamp}-{commit}")

    metrics = parse_metrics(log_text)
    row = Row(
        commit=commit,
        dirty=dirty,
        date=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        note=args.note or "",
        **metrics,
    )

    deviation = append_row(HISTORY_PATH, row)

    print(f"\n[harness] recorded in {HISTORY_PATH.relative_to(PROJ_DIR)}:")
    print(f"  commit  {row.commit}{' (DIRTY TREE)' if row.dirty else ''}")
    print(f"  WNS     {row.wns:+.3f} ns")
    print(f"  TNS     {row.tns:+.3f} ns over {row.failing_endpoints} failing endpoints")
    print(f"  DSP {row.dsp}  LUT {row.lut}  BRAM {row.bram}  CARRY4 {row.carry4}")
    if row.dirty:
        print("\n[harness] WARNING: the tree was dirty, so this row is not reproducible "
              "from its commit SHA.")
    if deviation:
        print(f"\n[harness] *** {deviation}")

    # Only once the row is safely written. A run whose metrics were refused
    # keeps its reports, which are exactly what you need to see to find out why.
    pruned = prune_runs(RUNS_DIR, args.keep_runs)
    if pruned:
        print(f"\n[harness] pruned {len(pruned)} old run director"
              f"{'y' if len(pruned) == 1 else 'ies'}, keeping the newest "
              f"{args.keep_runs} (--keep-runs)")
    return 0


def capture_oct32(dump_path: Path) -> list[Sample]:
    """Run test_ggx_control and return the Oct32 stream it observed.

    The test writes the dump itself (GGX_OCT32_DUMP), because the output stream
    is only observable from inside the simulation -- and it writes it only after
    its own gates have passed, so a baseline can never be captured from a run
    that failed.
    """
    env = dict(os.environ, GGX_OCT32_DUMP=str(dump_path))
    print("[harness] running test_ggx_control (~2 min)")
    r = subprocess.run(
        [sys.executable, str(SIM_DIR / "test_ggx_control.py")],
        cwd=SIM_DIR, env=env, capture_output=True, text=True,
    )
    if r.returncode != 0 or not dump_path.exists():
        sys.stdout.write(r.stdout[-4000:])
        sys.stderr.write(r.stderr[-4000:])
        raise HarnessError(
            "test_ggx_control did not pass, so there is nothing to capture or compare. "
            "Fix the failure first -- a baseline taken from a failing run is a "
            "recorded bug."
        )
    return parse_baseline(dump_path.read_text())


def cmd_baseline(args) -> int:
    dump_path = SIM_DIR / "sim_build" / "oct32_dump.txt"
    dump_path.unlink(missing_ok=True)
    samples = capture_oct32(dump_path)

    if args.mode == "capture":
        BASELINE_PATH.parent.mkdir(parents=True, exist_ok=True)
        BASELINE_PATH.write_text(render_baseline(samples))
        print(f"[harness] wrote {len(samples)} Samples to "
              f"{BASELINE_PATH.relative_to(PROJ_DIR)} -- commit it.")
        return 0

    if not BASELINE_PATH.exists():
        raise HarnessError(
            f"no baseline at {BASELINE_PATH.relative_to(PROJ_DIR)} to check against; "
            f"run: python sim/harness.py baseline capture"
        )
    diffs = baseline_diff(parse_baseline(BASELINE_PATH.read_text()), samples)
    if not diffs:
        print(f"[harness] bit-identical to the baseline ({len(samples)} Samples).")
        return 0
    print(f"[harness] NOT bit-identical: {len(diffs)} difference(s) "
          f"(first 20 shown):", file=sys.stderr)
    for d in diffs[:20]:
        print(f"  {d}", file=sys.stderr)
    return 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    pnr = sub.add_parser("pnr", help="place-and-route and append a row to the history")
    pnr.add_argument("--note", default="", help="why this run's resources moved")
    pnr.add_argument("--keep-runs", type=int, default=KEEP_RUNS, metavar="N",
                     help=f"run directories to keep afterwards (default {KEEP_RUNS})")
    pnr.set_defaults(func=cmd_pnr)

    bl = sub.add_parser("baseline", help="capture or check the Oct32 output baseline")
    bl.add_argument("mode", choices=["capture", "check"])
    bl.set_defaults(func=cmd_baseline)

    args = p.parse_args(argv)
    try:
        return args.func(args)
    except HarnessError as e:
        print(f"\n[harness] {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
