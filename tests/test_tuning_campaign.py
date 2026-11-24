"""The persistent, resumable tuning campaign (2026-09-13), without SUMO.

A fake evaluator stands in for the simulation: it writes a trial folder and returns a
deterministic objective of the weights, which is what the real one does (proved by the tuner
smoke test's `repeat` check). These tests pin:

* a campaign run in chunks, with a hard-killed trial in the middle, evaluates exactly the same
  weights in the same order, with the same values, as one uninterrupted run;
* an interrupted trial is marked FAIL, not counted, and its weights are evaluated again;
  a rejected trial counts and is not repeated; an error stops the campaign;
* trial folders are never reused, each carries a state file mirroring the database, and
  best_e3_weights.json appears only when the target is reached;
* --new never reuses a campaign, --resume never creates one, a resume with changed settings or
  code is refused, and a second process is locked out.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import optuna
import pytest

import scripts.tune_e3_weights as tuner
from scripts.tune_e3_weights import (
    CampaignLock,
    EvaluationResult,
    TrialRejected,
    evaluated_trials,
    reconcile_interrupted,
    run_campaign,
    storage_url,
    trial_folder,
    write_campaign_outputs,
)

optuna.logging.set_verbosity(optuna.logging.WARNING)
SEED = 20260721


def fake_objective(weights):
    return sum((value - 0.2) ** 2 for value in weights.values())


@pytest.fixture
def fake_sumo(monkeypatch):
    """Replace the SUMO run. `behaviour[evaluation index]` can make a trial raise."""
    behaviour = {}
    calls = []

    def evaluate(weights, sumo_cfg, sim_time, seed, trial_dir=None, reference=None):
        index = len(calls)
        calls.append(dict(weights))
        trial_dir.mkdir(parents=True, exist_ok=True)
        (trial_dir / "trial.json").write_text(json.dumps({"weights_requested": weights}), encoding="utf-8")
        action = behaviour.get(index)
        if action is not None:
            action(trial_dir)
        return EvaluationResult(objective=fake_objective(weights), relative={}, metrics={}, reroutes=0)

    monkeypatch.setattr(tuner, "evaluate_weights", evaluate)
    return behaviour, calls


def settings(target=5, startup=1):
    return {"seed": SEED, "trials_target": target, "startup_trials": startup, "code_fingerprint": {"x.py": "1"},
            "frozen_demand": "f", "evaluator": "e", "objective_weights": {}, "road_permissions": "r",
            "reference": "ref", "sampler": "s", "sumo_cfg": "c", "sim_time": 60}


def args_for(folder, stop_after=None):
    folder.mkdir(parents=True, exist_ok=True)
    return argparse.Namespace(output_dir=folder, stop_after=stop_after, sumo_cfg=Path("c"), sim_time=60,
                              seed=SEED, reference_data={})


def new_study(folder, name="study"):
    return optuna.create_study(storage=storage_url(folder), study_name=name, direction="minimize")


def reopen(folder, name="study"):
    """What a new process sees: the study as the database holds it."""
    return optuna.load_study(study_name=name, storage=storage_url(folder))


def history(study):
    return [(tuple(round(t.params[n], 12) for n in tuner.STICK_PARAMS), t.value) for t in evaluated_trials(study)]


def test_a_chunked_and_interrupted_campaign_equals_an_uninterrupted_one(tmp_path, fake_sumo):
    s = settings(target=5)
    continuous_dir = tmp_path / "continuous"
    continuous = new_study(args_for(continuous_dir).output_dir)
    assert run_campaign(continuous, args_for(continuous_dir), s) == "target reached"

    folder = tmp_path / "chunked"
    study = new_study(args_for(folder).output_dir)
    assert run_campaign(study, args_for(folder, stop_after=2), s) == "--stop-after 2 reached"
    assert not (folder / "best_e3_weights.json").exists()       # nothing loadable mid-campaign

    # A process that died mid-trial: the sampler proposed, the trial started, nothing was told.
    study = reopen(folder)
    study.sampler = optuna.samplers.TPESampler(seed=SEED + 2, n_startup_trials=1)
    dead = study.ask()
    dead_weights = [dead.suggest_float(n, 0.0, 1.0) for n in tuner.STICK_PARAMS]
    trial_folder(folder, dead.number).mkdir(parents=True)
    (trial_folder(folder, dead.number) / "tripinfo.xml").write_text("<partial", encoding="utf-8")
    assert reopen(folder).trials[dead.number].state.name == "RUNNING"     # what Optuna leaves

    study = reopen(folder)
    assert reconcile_interrupted(study, folder) == [dead.number]
    assert run_campaign(study, args_for(folder), s) == "target reached"
    write_campaign_outputs(study, args_for(folder), s)

    study = reopen(folder)
    assert history(study) == history(continuous)                  # same weights, order, values
    retry = study.trials[dead.number + 1]
    assert [retry.params[n] for n in tuner.STICK_PARAMS] == dead_weights   # evaluated again
    assert [t.state.name for t in study.trials] == ["COMPLETE", "COMPLETE", "FAIL", "COMPLETE",
                                                    "COMPLETE", "COMPLETE"]
    state = json.loads((trial_folder(folder, dead.number) / "TRIAL_STATE.json").read_text(encoding="utf-8"))
    assert state["state"] == "FAIL" and state["fail_reason"] == "interrupted" and not state["counts_as_a_result"]
    assert (trial_folder(folder, dead.number) / "tripinfo.xml").read_text(encoding="utf-8") == "<partial"
    for trial in evaluated_trials(study):
        marker = json.loads((trial_folder(folder, trial.number) / "TRIAL_STATE.json").read_text(encoding="utf-8"))
        assert marker["state"] == "COMPLETE" and marker["value"] == trial.value
    best = json.loads((folder / "best_e3_weights.json").read_text(encoding="utf-8"))
    assert best["campaign_complete"] is True and best["trials_evaluated"] == 5


def test_a_rejected_trial_counts_and_is_not_repeated(tmp_path, fake_sumo):
    behaviour, calls = fake_sumo

    def reject(trial_dir):
        raise TrialRejected("1 exception(s) logged")
    behaviour[1] = reject
    folder = tmp_path / "c"
    study = new_study(args_for(folder).output_dir)
    run_campaign(study, args_for(folder), settings(target=3))
    study = reopen(folder)
    assert [t.state.name for t in study.trials] == ["COMPLETE", "FAIL", "COMPLETE"]
    assert study.trials[1].user_attrs["fail_reason"] == "rejected"
    assert len(evaluated_trials(study)) == 3
    assert calls[2] != calls[1]                                   # the next seed, new weights


def test_an_error_stops_the_campaign_and_its_weights_come_round_again(tmp_path, fake_sumo):
    behaviour, calls = fake_sumo

    def crash(trial_dir):
        raise RuntimeError("SUMO died")
    behaviour[1] = crash
    folder = tmp_path / "c"
    study = new_study(args_for(folder).output_dir)
    with pytest.raises(RuntimeError):
        run_campaign(study, args_for(folder), settings(target=3))
    study = reopen(folder)
    assert study.trials[1].state.name == "FAIL" and study.trials[1].user_attrs["fail_reason"] == "error"
    behaviour.clear()
    run_campaign(study, args_for(folder), settings(target=3))
    assert calls[2] == calls[1]                                   # not evaluated, so tried again
    assert len(evaluated_trials(reopen(folder))) == 3


def test_a_stop_file_stops_after_the_trial_in_progress(tmp_path, fake_sumo):
    behaviour, _ = fake_sumo
    folder = tmp_path / "c"
    behaviour[0] = lambda trial_dir: (folder / "STOP").write_text("", encoding="utf-8")
    study = new_study(args_for(folder).output_dir)
    assert run_campaign(study, args_for(folder), settings(target=3)) == "STOP file found"
    assert [t.state.name for t in reopen(folder).trials] == ["COMPLETE"]


def test_a_trial_folder_is_never_reused(tmp_path, fake_sumo):
    folder = tmp_path / "c"
    trial_folder(folder, 0).mkdir(parents=True)
    (trial_folder(folder, 0) / "tripinfo.xml").write_text("evidence", encoding="utf-8")
    study = new_study(args_for(folder).output_dir)
    with pytest.raises(RuntimeError, match="never reused"):
        run_campaign(study, args_for(folder), settings(target=2))
    assert (trial_folder(folder, 0) / "tripinfo.xml").read_text(encoding="utf-8") == "evidence"


def test_a_second_process_is_locked_out(tmp_path):
    holder = subprocess.Popen([sys.executable, "-c", (
        "import sys, time; sys.path.insert(0, r'%s'); from scripts.tune_e3_weights import CampaignLock; "
        "lock = CampaignLock(r'%s').__enter__(); print('held', flush=True); time.sleep(30)")
        % (Path(tuner.REPO_ROOT), tmp_path)], stdout=subprocess.PIPE, text=True)
    try:
        assert holder.stdout.readline().strip() == "held"
        with pytest.raises(SystemExit, match="another tuner process"):
            with CampaignLock(tmp_path):
                pass
    finally:
        holder.kill()
        holder.wait()
    with CampaignLock(tmp_path):                                  # released with its process
        pass


def run_main(monkeypatch, *argv):
    monkeypatch.setattr(sys, "argv", ["tune_e3_weights.py", *argv])
    return tuner.main()


def test_resume_never_creates_a_campaign(tmp_path, monkeypatch):
    with pytest.raises(SystemExit):
        run_main(monkeypatch, "--resume", "--output-dir", str(tmp_path / "nothing"))
    assert not (tmp_path / "nothing" / tuner.STORAGE_FILE).exists()


def test_new_never_reuses_a_campaign_folder(tmp_path, monkeypatch):
    new_study(tmp_path)
    before = (tmp_path / tuner.STORAGE_FILE).read_bytes()
    with pytest.raises(SystemExit):
        run_main(monkeypatch, "--new", "--output-dir", str(tmp_path))
    assert (tmp_path / tuner.STORAGE_FILE).read_bytes() == before


def test_a_mode_must_be_chosen(tmp_path, monkeypatch):
    with pytest.raises(SystemExit):
        run_main(monkeypatch, "--output-dir", str(tmp_path))


def resume_args(folder, **given):
    base = dict(new=False, resume=True, output_dir=folder, study_name=None, seed=None, sim_time=None,
                trials=None, startup_trials=None, sumo_cfg=None, reference=None, accept_code_change=False)
    base.update(given)
    return argparse.Namespace(**base)


@pytest.fixture
def stored_campaign(tmp_path, monkeypatch):
    stored = settings()
    study = new_study(tmp_path)
    study.set_user_attr("campaign", stored)
    monkeypatch.setattr(tuner, "load_reference", lambda path, seed, sim_time: {})
    current = dict(stored)
    monkeypatch.setattr(tuner, "campaign_settings", lambda *a, **k: dict(current))
    return tmp_path, current


def test_a_resume_continues_the_campaign_as_created(stored_campaign):
    folder, _ = stored_campaign
    study, loaded = tuner._open_campaign(resume_args(folder), argparse.ArgumentParser())
    assert loaded["trials_target"] == 5


def test_a_resume_with_different_settings_is_refused(stored_campaign):
    folder, _ = stored_campaign
    with pytest.raises(SystemExit):
        tuner._open_campaign(resume_args(folder, sim_time=3600), argparse.ArgumentParser())


def test_a_resume_after_a_code_change_is_refused_unless_accepted(stored_campaign):
    folder, current = stored_campaign
    current["code_fingerprint"] = {"x.py": "2"}
    with pytest.raises(SystemExit):
        tuner._open_campaign(resume_args(folder), argparse.ArgumentParser())
    study, loaded = tuner._open_campaign(resume_args(folder, accept_code_change=True), argparse.ArgumentParser())
    assert loaded["code_fingerprint"] == {"x.py": "2"}
    assert reopen(folder).user_attrs["code_changes"][0]["files"] == ["x.py"]
