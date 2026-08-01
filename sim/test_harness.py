#!/usr/bin/env python3
"""Unit tests for the regression harness's pure logic (sim/harness.py).

Run like every other test in this directory:  python test_harness.py

Deliberately no Vivado and no simulator here. What is worth testing about the
harness is the part that turns a tool's output into a recorded number and then
guards it -- parsing, the DSP-deviation refusal, and the baseline diff. Those
are the parts that can silently record a wrong row; the Tcl and the cocotb run
they wrap are exercised by actually running them.
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness import (  # noqa: E402
    HarnessError,
    Row,
    append_row,
    baseline_diff,
    dsp_deviation,
    parse_baseline,
    parse_metrics,
    Sample,
    read_rows,
    render_baseline,
)

GOOD_PNR_LOG = """
INFO: [Vivado 12-3661] blah blah
Phase 4 Post Router Timing
GGXMETRIC wns -1.330
GGXMETRIC tns -407.000
GGXMETRIC failing_endpoints 1852
GGXMETRIC dsp 79
GGXMETRIC lut 8947
GGXMETRIC bram 4.5
GGXMETRIC carry4 1302
HARNESS_PNR_DONE
INFO: [Common 17-206] Exiting Vivado
"""


def test_parse_metrics_reads_every_recorded_quantity():
    m = parse_metrics(GOOD_PNR_LOG)
    assert m == {
        "wns": -1.330,
        "tns": -407.0,
        "failing_endpoints": 1852,
        "dsp": 79,
        "lut": 8947,
        "bram": 4.5,
        "carry4": 1302,
    }, m


def test_parse_metrics_rejects_a_run_that_never_finished():
    # Vivado exits 0 after plenty of fatal-in-practice failures, so the absence
    # of the sentinel is the only reliable evidence the flow ran to completion.
    truncated = GOOD_PNR_LOG.replace("HARNESS_PNR_DONE", "")
    try:
        parse_metrics(truncated)
    except HarnessError as e:
        assert "HARNESS_PNR_DONE" in str(e), e
    else:
        raise AssertionError("a run with no completion sentinel must not be recorded")


def test_parse_metrics_rejects_a_missing_metric():
    partial = GOOD_PNR_LOG.replace("GGXMETRIC carry4 1302\n", "")
    try:
        parse_metrics(partial)
    except HarnessError as e:
        assert "carry4" in str(e), e
    else:
        raise AssertionError("a row missing a column must not be recorded")


def test_parse_metrics_rejects_a_non_numeric_metric():
    bad = GOOD_PNR_LOG.replace("GGXMETRIC dsp 79", "GGXMETRIC dsp none")
    try:
        parse_metrics(bad)
    except HarnessError as e:
        assert "dsp" in str(e), e
    else:
        raise AssertionError("a non-numeric metric must not be recorded")


def test_parse_metrics_rejects_a_build_that_could_not_read_a_rom():
    # The trap this harness exists for: Vivado only WARNS on a missing $readmem
    # file, folds the ROM away, and reports a plausible WNS for a design ~24
    # DSPs lighter than the real one. The DSP-delta guard cannot catch it on a
    # first row, so the log itself has to be read.
    degenerate = GOOD_PNR_LOG.replace(
        "Phase 4 Post Router Timing",
        "CRITICAL WARNING: [Synth 8-4445] could not open $readmem data file "
        "'ggx_trig_rom.mem'; please make sure the file is added to project, ignoring",
    )
    try:
        parse_metrics(degenerate)
    except HarnessError as e:
        assert "readmem" in str(e), e
    else:
        raise AssertionError("a build with an unreadable ROM must not be recorded")


def _row(**kw):
    base = dict(
        commit="abc1234",
        dirty=False,
        date="2026-08-01T00:00:00Z",
        wns=-1.330,
        tns=-407.0,
        failing_endpoints=1852,
        dsp=79,
        lut=8947,
        bram=4.5,
        carry4=1302,
        note="",
    )
    base.update(kw)
    return Row(**base)


def test_dsp_deviation_is_silent_when_the_count_holds():
    assert dsp_deviation(_row(), _row(lut=9001)) is None


def test_dsp_deviation_reports_direction_and_size():
    # The whole point of recording DSP alongside slack: a build that lost a
    # source file or a $readmem ROM reports a plausible WNS and is only caught
    # here. See the ROM trap in docs/handoff.md.
    msg = dsp_deviation(_row(dsp=79), _row(dsp=67))
    assert msg is not None
    assert "79" in msg and "67" in msg and "-12" in msg, msg


def test_dsp_deviation_against_no_previous_row_is_silent():
    assert dsp_deviation(None, _row()) is None


def test_append_row_refuses_a_dsp_change_without_a_note():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "timing_history.csv"
        append_row(path, _row(commit="aaa", dsp=79))
        try:
            append_row(path, _row(commit="bbb", dsp=67))
        except HarnessError as e:
            assert "DSP" in str(e), e
        else:
            raise AssertionError("an unexplained DSP change must not be recorded")
        assert len(read_rows(path)) == 1


def test_append_row_records_a_dsp_change_that_carries_a_note():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "timing_history.csv"
        append_row(path, _row(commit="aaa", dsp=79))
        append_row(path, _row(commit="bbb", dsp=67, note="folded the inv_sqrt"))
        rows = read_rows(path)
        assert len(rows) == 2
        assert rows[1].dsp == 67
        assert rows[1].note == "folded the inv_sqrt"


def test_rows_survive_a_write_read_round_trip():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "timing_history.csv"
        original = _row(dirty=True, note="a note, with a comma")
        append_row(path, original)
        assert read_rows(path) == [original]


def test_read_rows_of_a_missing_file_is_empty():
    with tempfile.TemporaryDirectory() as d:
        assert read_rows(Path(d) / "nope.csv") == []


BASELINE_SAMPLES = [
    Sample(0, 0, 0x1234ABCD, 0),
    Sample(0, 1, 0x0000FFFF, 1),
    Sample(1, 0, 0xDEADBEEF, 1),
]


def test_a_dump_without_a_header_still_parses():
    # test_ggx_control writes its dump through render_baseline(header=False);
    # `baseline check` parses it with the same parser that reads the committed
    # file. If those two ever disagree the harness compares nothing.
    assert parse_baseline(render_baseline(BASELINE_SAMPLES, header=False)) == BASELINE_SAMPLES


def test_baseline_round_trips_through_its_own_format():
    assert parse_baseline(render_baseline(BASELINE_SAMPLES)) == BASELINE_SAMPLES


def test_rendered_baseline_carries_no_timestamp_or_absolute_path():
    # This is what makes "diff the two files" a sufficient comparison: there is
    # nothing volatile in the file to filter out in the first place.
    text = render_baseline(BASELINE_SAMPLES)
    assert "/home/" not in text
    assert "ns" not in text.replace("# ", "").split("\n")[-2]


def test_baseline_diff_of_an_identical_capture_is_empty():
    assert baseline_diff(BASELINE_SAMPLES, list(BASELINE_SAMPLES)) == []


def test_baseline_diff_reports_a_changed_oct32_word():
    changed = list(BASELINE_SAMPLES)
    changed[1] = (0, 1, 0x0000FFFE, 1)
    out = baseline_diff(BASELINE_SAMPLES, changed)
    assert len(out) == 1
    assert "0000ffff" in out[0].lower() and "0000fffe" in out[0].lower(), out


def test_baseline_diff_reports_a_changed_tlast():
    changed = list(BASELINE_SAMPLES)
    changed[2] = (1, 0, 0xDEADBEEF, 0)
    out = baseline_diff(BASELINE_SAMPLES, changed)
    assert len(out) == 1 and "TLAST" in out[0], out


def test_baseline_diff_reports_a_truncated_stream():
    out = baseline_diff(BASELINE_SAMPLES, BASELINE_SAMPLES[:2])
    assert len(out) == 1 and "3" in out[0] and "2" in out[0], out


def test_baseline_diff_reports_an_over_long_stream():
    out = baseline_diff(BASELINE_SAMPLES, BASELINE_SAMPLES + [(1, 1, 0x1, 1)])
    assert len(out) == 1 and "4" in out[0], out


def main():
    tests = [(n, o) for n, o in sorted(globals().items()) if n.startswith("test_")]
    failures = 0
    for name, fn in tests:
        try:
            fn()
        except Exception as e:  # noqa: BLE001 -- a test runner reports, it does not raise
            failures += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
        else:
            print(f"pass {name}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
