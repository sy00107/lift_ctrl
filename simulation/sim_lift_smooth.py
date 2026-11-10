#!/usr/bin/env python3
"""Compare hard speed gating with continuous approach feedback, offline only."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

from sim_lift_pd import Simulator, Plant, fw

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_APPROACH = fw.PositionController.approach_duty


class RecordedSimulator(Simulator):
    def __init__(self, *args, **kwargs):
        self.samples = []
        super().__init__(*args, **kwargs)

    def step(self, elapsed, heartbeat=True):
        super().step(elapsed, heartbeat)
        self.samples.append(dict(time_s=self.ms / 1000, position_mm=self.x,
                                 target_mm=self.target, velocity_mm_s=self.v,
                                 estimated_mm=self.c.position_mm, pwm=self.duty,
                                 state=self.c.mode, fault=self.c.fault))


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    plants = [Plant('light', friction=.06), Plant('observed_friction_assumption', friction=.18),
              Plant('slow', friction=.18, tau=.25, coast_tau=.1, latency_ms=50),
              Plant('inertial', friction=.08, tau=.35, coast_tau=.15, latency_ms=80),
              Plant('unmeasured_high_friction', friction=.22)]
    moves = [(10, 100), (100, 200), (200, 100), (10, 600), (600, 10)]
    rows, examples = [], {}
    try:
        for version in ('before', 'after'):
            def approach(self, duty, remaining, tolerance=fw.TOLERANCE_MM,
                         floor=fw.MIN_POSITION_DUTY, smooth=False):
                return ORIGINAL_APPROACH(self, duty, remaining, tolerance, floor,
                                         smooth=smooth and version == 'after')
            fw.PositionController.approach_duty = approach
            for plant in plants:
                for resolution in (.1, .20018, .21574):
                    for start, target in moves:
                        p = replace(plant, resolution=resolution)
                        sim = RecordedSimulator(p, start)
                        result = sim.move(target, .04, .001, 1.)
                        tail = [x for x in sim.samples if x['state'] == 'moving'
                                and abs(x['target_mm'] - x['position_mm']) < 10]
                        off = [x['pwm'] == 0 for x in tail]
                        interruptions = sum(off[i] and not off[i-1] for i in range(1, len(off)))
                        rows.append(dict(version=version, model=p.name, resolution=resolution,
                                         zero_output_samples=sum(off),
                                         drive_to_coast_transitions=interruptions, **result))
                        if resolution == .21574 and start == 100 and target == 200:
                            examples[version + '_' + p.name] = sim.samples
    finally:
        fw.PositionController.approach_duty = ORIGINAL_APPROACH
    summary = {}
    for version in ('before', 'after'):
        selected = [r for r in rows if r['version'] == version]
        summary[version] = dict(trials=len(selected), passed=sum(r['passed'] for r in selected),
                               events=dict(Counter(r['event'] for r in selected)),
                               contacts=sum(r['contact'] for r in selected),
                               near_target_zero_samples=sum(r['zero_output_samples'] for r in selected),
                               drive_to_coast_transitions=sum(r['drive_to_coast_transitions'] for r in selected),
                               max_overshoot_mm=max(r['overshoot_mm'] for r in selected),
                               mean_arrival_s=sum(r['arrival_s'] for r in selected)/len(selected))
    injections = []
    for injection in ('jam', 'encoder_loss', 'encoder_reverse', 'comms_loss'):
        sim = Simulator(Plant(injection, injection=injection, resolution=.21574), 10)
        injections.append(dict(injection=injection, **sim.move(200, .04, .001, 1)))
    payload = dict(summary=summary, trials=rows, examples=examples,
                   fault_injection=injections, models=[asdict(p) for p in plants],
                   firmware_sha256=hashlib.sha256((ROOT/'pico/main.py').read_bytes()).hexdigest(),
                   limitations='Assumed mechanics, not hardware qualification; nominal 38.1 mm/s maximum; no load identification.')
    (output/'results.json').write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    print('fault_injection:', [(r['injection'], r['fault']) for r in injections])
    return payload


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/lift-smooth-approach-20261110')
    run(parser.parse_args().output)
