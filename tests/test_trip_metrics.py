"""Tests for the tripinfo-derived metrics (audit M1, M3, M6, M7, M8).

These call the real functions in `metrics/trip_metrics.py` against small XML fixtures
written to a temporary directory, so they check the metric definitions themselves rather
than re-implementing them (audit finding R8).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from metrics.trip_metrics import (
    aggregate,
    load_summary,
    load_summary_timeseries,
    load_tripinfo,
    reconcile_reroutes,
)

TRIPINFO_XML = """<?xml version="1.0" encoding="UTF-8"?>
<tripinfos>
    <tripinfo id="normal_1" depart="10.00" departDelay="5.00" arrival="110.00"
              duration="100.00" routeLength="1000.00" waitingTime="20.00" waitingCount="2"
              stopTime="0.00" timeLoss="30.00" rerouteNo="1" vType="normal_passenger"/>
    <tripinfo id="normal_2" depart="20.00" departDelay="0.00" arrival="120.00"
              duration="100.00" routeLength="2000.00" waitingTime="0.00" waitingCount="0"
              stopTime="0.00" timeLoss="10.00" rerouteNo="0" vType="normal_passenger"/>
    <tripinfo id="amb_1" depart="30.00" departDelay="1.00" arrival="90.00"
              duration="60.00" routeLength="1500.00" waitingTime="4.00" waitingCount="1"
              stopTime="0.00" timeLoss="8.00" rerouteNo="2" vType="ambulance"/>
</tripinfos>
"""

SUMMARY_XML = """<?xml version="1.0" encoding="UTF-8"?>
<summary>
    <step time="0.00" loaded="10" inserted="1" running="1" waiting="9" ended="0"
          meanSpeed="0.00" meanWaitingTime="0.00" meanTravelTime="-1.00" halting="1" teleports="0"/>
    <step time="1.00" loaded="10" inserted="8" running="5" waiting="2" ended="3"
          meanSpeed="9.30" meanWaitingTime="3.82" meanTravelTime="138.07" halting="2" teleports="4"/>
</summary>
"""


@pytest.fixture
def run_dir(tmp_path):
    (tmp_path / "tripinfo.xml").write_text(TRIPINFO_XML, encoding="utf-8")
    (tmp_path / "summary.xml").write_text(SUMMARY_XML, encoding="utf-8")
    return tmp_path


def test_travel_time_includes_insertion_delay(run_dir):
    """M7: contract travel time is arrival minus *scheduled* departure."""
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    assert trips["normal_1"]["scheduled_travel_time"] == 105.0  # 100 duration + 5 delay
    assert trips["normal_2"]["scheduled_travel_time"] == 100.0


def test_waiting_time_is_read_once_not_accumulated(run_dir):
    """M1: the value is SUMO's accumulated waiting time, taken as-is."""
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    assert trips["normal_1"]["waitingTime"] == 20.0
    report = aggregate(trips)
    assert report["mean_waiting_time_s"] == 8.0  # (20 + 0 + 4) / 3


def test_congestion_is_time_loss_not_a_speed_proxy(run_dir):
    """M3, M4: contract decision 13.4 defines congestion as mean time loss in seconds."""
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    report = aggregate(trips)
    assert report["mean_time_loss_s"] == 16.0  # (30 + 10 + 8) / 3
    assert report["median_time_loss_s"] == 10.0
    assert "congestion_speed_proxy" not in report


def test_teleports_are_the_cumulative_total(run_dir):
    """M6: the total from summary.xml, not a running maximum of simultaneous teleports."""
    summary = load_summary(run_dir / "summary.xml")
    assert summary["teleports"] == 4.0
    report = aggregate(load_tripinfo(run_dir / "tripinfo.xml"), summary)
    assert report["teleports_total"] == 4


def test_incomplete_trips_are_reported_not_counted_as_arrivals(run_dir):
    """M8: vehicles that never arrived must not be treated as completed."""
    summary = load_summary(run_dir / "summary.xml")
    report = aggregate(load_tripinfo(run_dir / "tripinfo.xml"), summary)
    assert report["loaded_vehicles"] == 10
    assert report["inserted_vehicles"] == 8
    assert report["never_inserted_vehicles"] == 2
    assert report["incomplete_trips"] == 5  # 8 inserted - 3 ended
    assert report["completed_trips"] == 3   # only what tripinfo contains


def test_emergency_vehicles_are_split_out(run_dir):
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    report = aggregate(trips, emergency_types=("ambulance",))
    assert report["completed_emergency_trips"] == 1
    assert report["completed_normal_trips"] == 2
    assert report["mean_emergency_time_loss_s"] == 8.0
    assert report["mean_normal_time_loss_s"] == 20.0


def test_reroute_reconciliation_reports_both_counts(run_dir):
    trips = load_tripinfo(run_dir / "tripinfo.xml")
    result = reconcile_reroutes(trips, applied_reroutes=7)
    assert result["sumo_reroute_no_over_completed_trips"] == 3
    assert result["reroutes_applied_by_runner"] == 7


def test_summary_timeseries_is_read_from_sumo(run_dir):
    series = load_summary_timeseries(run_dir / "summary.xml")
    assert len(series) == 2
    assert series[1]["meanSpeed"] == 9.3
    assert series[1]["halting"] == 2.0


def test_quarantined_baseline_module_cannot_be_imported():
    """M8, H1: the old module launched a simulation on import and mis-recorded arrivals."""
    with pytest.raises(ImportError):
        import metrics.metrics_baseline  # noqa: F401
