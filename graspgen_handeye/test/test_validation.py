import numpy as np
from graspgen_handeye.validate import drift_report
from graspgen_handeye.core import matrix


def sample(x):return {'base_target':matrix(np.eye(3),[x,0,.4]).tolist()}


def test_report_metric_units_and_minimum_views():
    ref=matrix(np.eye(3),[0,0,.4])
    report=drift_report([sample(0),sample(.002)],ref)
    assert not report['accepted'] and not report['enough_samples']
    report=drift_report([sample(0),sample(.002),sample(.004)],ref)
    assert report['accepted'] and abs(report['max_pairwise_translation_mm']-4)<1e-8
    assert len(report['positions_base_m'])==3


def test_drift_and_moved_target_do_not_pass():
    ref=matrix(np.eye(3),[0,0,.4])
    assert not drift_report([sample(0),sample(.002),sample(.02)],ref)['accepted']
    assert not drift_report([sample(.04),sample(.041),sample(.042)],ref)['accepted']
