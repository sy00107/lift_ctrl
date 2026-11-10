#!/usr/bin/env python3
"""Reproducible software-in-loop lift experiments. No serial/GPIO/network access.

Runs the actual PositionController with modeled mechanics, never a hardware rating.
Usage: python3 scripts/sim_lift_pd.py [--workers 6]
"""
import argparse
import csv
from dataclasses import asdict, dataclass, replace
from collections import deque, Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/lift-pd-hardening-20261109/simulation'
sys.modules.setdefault('machine', types.ModuleType('machine'))
sys.modules.setdefault('micropython', types.ModuleType('micropython'))
sys.modules.setdefault('utime', types.SimpleNamespace(ticks_diff=lambda a,b: (a-b+(1<<29))%(1<<30)-(1<<29)))
spec = importlib.util.spec_from_file_location('sim_firmware', ROOT/'pico/main.py')
fw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fw)
DT = .01

@dataclass(frozen=True)
class Plant:
    name: str = 'nominal'
    load_fraction: float = .5  # fraction of dual-column desk rating, NOT single-column kg
    supply: float = 1.
    friction: float = .06
    tau: float = .12
    coast_tau: float = .06
    latency_ms: int = 20
    resolution: float = .1
    scale_error: float = 0.
    down_gain: float = 1.
    backdrive: float = 0.
    injection: str = ''

class Simulator:
    def __init__(self, plant, start=0., trace=False):
        self.p = plant
        self.ms = 400
        self.x = start
        self.v = 0.
        self.raw = round(start/plant.resolution)
        self.initial_raw = self.raw
        self.pulses = 0
        self.direction = 0
        self.duty = 0.
        self.queue = deque([(0,0.)]*(plant.latency_ms//10+1))
        self.contact = False
        self.peak_speed = 0.
        self.trace_enabled = trace
        self.trace = []
        self.target = start
        self.c = fw.PositionController(self.drive, lambda:self.ms, self.encoder,
            mm_per_count=plant.resolution*(1+plant.scale_error), ab_verified=True)
        # Benchmark preposition: an already correctly homed encoder at start.
        # No new zero is performed between moves in a sequence.
        self.c.origin_count = -self.raw
        self.command('arm')

    def drive(self, direction, duty):
        self.direction,self.duty = direction,duty

    def encoder(self):
        return self.raw,self.pulses

    def command(self, cmd, **values):
        self.c.command(dict(cmd=cmd,**values))

    def step(self, elapsed, heartbeat=True):
        p=self.p
        self.queue.append((self.direction,self.duty))
        direction,duty=self.queue.popleft()
        active=elapsed>=1.
        velocity_target=0.
        if direction and duty>p.friction:
            # 38.1 is a maximum, not a promise at rated load. Load response unknown.
            gain=p.supply*(1-.30*p.load_fraction)
            if direction<0: gain*=p.down_gain
            velocity_target=direction*min(38.1,38.1*gain*(duty-p.friction)/(1-p.friction))
        elif not direction:
            velocity_target=p.backdrive
        tau=p.tau if direction else p.coast_tau
        self.v+=(velocity_target-self.v)*(1-math.exp(-DT/tau))
        if p.injection=='jam' and active:self.v=0.
        next_x=self.x+self.v*DT
        if next_x<0 or next_x>630:
            self.contact=True
            next_x=min(630.,max(0.,next_x));self.v=0.
        self.x=next_x
        self.peak_speed=max(self.peak_speed,abs(self.v))
        raw=round(self.x/p.resolution)
        if p.injection=='encoder_reverse':raw=2*self.initial_raw-raw
        if p.injection=='encoder_loss' and active:raw=self.raw
        self.pulses+=abs(raw-self.raw)
        self.raw=raw
        self.ms+=10
        self.c.update()
        if heartbeat and not(p.injection=='comms_loss' and active) and self.ms%50==0:
            self.command('heartbeat')
        if self.trace_enabled and self.ms%50==0:
            self.trace.append(dict(time_s=self.ms/1000,actual_mm=self.x,
                estimated_mm=self.c.position_mm,target_mm=self.target,velocity_mm_s=self.v,
                estimated_velocity_mm_s=self.c.velocity,pwm=self.duty*self.direction,
                state=self.c.mode,fault=self.c.fault))

    def move(self,target,kp,kd,duty,hold=1.):
        start=self.x;begin=self.ms;self.target=target
        self.contact=False;self.peak_speed=0.
        self.command('move_to',target_mm=target,max_duty=duty,kp=kp,kd=kd)
        max_over=0.
        for i in range(12500):
            self.step((self.ms-begin)/1000)
            max_over=max(max_over, (self.x-target)*(1 if target>start else -1))
            if self.c.mode=='idle':break
        arrival=(self.ms-begin)/1000
        event=self.c.event;fault=self.c.fault
        stop_x=self.x
        for i in range(round(hold/DT)):
            self.step((self.ms-begin)/1000)
            max_over=max(max_over,(self.x-target)*(1 if target>start else -1))
        estimated=self.c.position_mm
        passed=(not fault and not self.c.fault and event=='target_reached'
            and abs(self.x-target)<=1.25 and not self.contact)
        return dict(start_mm=start,target_mm=target,kp=kp,kd=kd,max_duty=duty,
            arrival_s=arrival,actual_mm=self.x,actual_error_mm=self.x-target,
            estimated_error_mm=None if estimated is None else estimated-target,
            post_stop_drift_mm=self.x-stop_x,overshoot_mm=max_over,
            peak_speed_mm_s=self.peak_speed,contact=self.contact,
            event=event,fault=fault or self.c.fault,passed=passed)

def trial(job):
    p,kp,kd,start,target,duty=job
    result=Simulator(p,start).move(target,kp,kd,duty)
    return dict(model=p.name,**result)

def write_csv(path, rows):
    if not rows:return
    with path.open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workers',type=int,default=6)
    args=parser.parse_args();OUT.mkdir(parents=True,exist_ok=True);began=time.time()
    # Training: fixed envelope; held-out draws below are never used to select gains.
    plants=[Plant(),Plant('light_fast',0,1,.035,.06,.03,0,.05),
        Plant('loaded',1,.9,.085,.25,.1,50,.2,down_gain=1.12),
        Plant('high_inertia',.75,.95,.07,.35,.15,80,.2,down_gain=1.1)]
    moves=[(0,10,.25),(0,100,.6),(0,620,1.),(620,10,1.),(300,200,.25),(200,300,1.)]
    gains=[(kp,kd) for kp in [.002,.003,.004,.006,.008,.01,.015,.02,.03,.04,.05,.06,.08,.1]
        for kd in [0,.0005,.001,.002,.003,.005,.006,.008,.01,.015,.02,.025,.03,.04,.06,.08]]
    jobs=[(p,kp,kd,*move) for kp,kd in gains for p in plants for move in moves]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results=list(pool.map(trial,jobs,chunksize=12))
    leaderboard=[]
    for kp,kd in gains:
        rows=[r for r in results if r['kp']==kp and r['kd']==kd]
        good=[r for r in rows if r['passed']]
        leaderboard.append(dict(kp=kp,kd=kd,trials=len(rows),passed=len(good),
            failures=len(rows)-len(good),contacts=sum(r['contact'] for r in rows),
            mean_arrival_s=sum(r['arrival_s'] for r in rows)/len(rows),
            worst_actual_error_mm=max(abs(r['actual_error_mm']) for r in rows)))
    leaderboard.sort(key=lambda r:(r['failures'],r['contacts'],r['mean_arrival_s'],r['worst_actual_error_mm']))
    best=leaderboard[0];kp,kd=best['kp'],best['kd']
    rng=random.Random(20261109)
    held_plants=[Plant(name=f'holdout_{i:03}',load_fraction=rng.random(),
        supply=rng.uniform(.85,1),friction=rng.uniform(.03,.09),tau=rng.uniform(.05,.35),
        coast_tau=rng.uniform(.025,.15),latency_ms=rng.choice([0,20,50,80,100]),
        resolution=rng.choice([.05,.1,.2,.4]),down_gain=rng.uniform(.9,1.15)) for i in range(240)]
    held_jobs=[]
    for p in held_plants:
        for start,target in [(0,rng.choice([10,20,100,300,620])),(630,rng.choice([10,100,300,600,620]))]:
            held_jobs.append((p,kp,kd,start,target,rng.choice([.25,.4,.6,.8,1.])))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        held=list(pool.map(trial,held_jobs,chunksize=8))
    cycles=[];cycle_traces=[]
    for p in plants:
        cycle_sim=Simulator(p,trace=True)
        for cycle in range(20):
            for target in [200,300,620,400,100,10]:
                r=cycle_sim.move(target,kp,kd,1.)
                cycles.append(dict(model=p.name,cycle=cycle+1,**r))
                if cycle_sim.c.fault:break
            if cycle_sim.c.fault:break
        cycle_traces.extend(dict(model=p.name,**r) for r in cycle_sim.trace)
    duty_tests=[]
    for p in plants[:3]:
        for duty in [.1,.15,.2,.25,.5,1.]:
            for start,target in [(0,620),(630,10)]:
                duty_tests.append(trial((p,kp,kd,start,target,duty)))
    stress=[];stress_traces=[]
    stress_plants=[Plant('high_static_friction',friction=.18,load_fraction=1),
        Plant('backdrivable_no_hold',backdrive=-.8),
        Plant('calibration_plus_3pct',scale_error=.03),
        Plant('calibration_minus_3pct',scale_error=-.03),
        Plant('encoder_reverse',injection='encoder_reverse'),
        Plant('encoder_loss',injection='encoder_loss'),Plant('jam',injection='jam'),
        Plant('comms_loss',injection='comms_loss'),
        Plant('very_slow_coast',tau=.5,coast_tau=.6,latency_ms=150),
        Plant('calibration_minus_3pct_at_top',scale_error=-.03)]
    for p in stress_plants:
        sim=Simulator(p,start=100,trace=True)
        r=sim.move(620 if p.name.endswith('_at_top') else 500,kp,kd,1.,hold=5.)
        r['output_off_after_test']=sim.duty==0
        r['homed_after_test']=sim.c.homed
        stress.append(dict(model=p.name,**r));stress_traces.extend(dict(model=p.name,**t) for t in sim.trace)
    representative=[];traces=[]
    for label,pg,pd in [('existing',.006,.001),('P_only',kp,0.),('selected',kp,kd)]:
        sim=Simulator(Plant(),trace=True)
        for target in [200,300,620,300,10]:
            r=sim.move(target,pg,pd,1.)
            representative.append(dict(controller=label,**r))
            if sim.c.fault:break
        traces.extend(dict(controller=label,**t) for t in sim.trace)
    write_csv(OUT/'gain_sweep.csv',results);write_csv(OUT/'leaderboard.csv',leaderboard)
    write_csv(OUT/'holdout.csv',held);write_csv(OUT/'cycles.csv',cycles)
    write_csv(OUT/'stress.csv',stress);write_csv(OUT/'stress_traces.csv',stress_traces)
    write_csv(OUT/'comparison.csv',representative);write_csv(OUT/'comparison_traces.csv',traces)
    write_csv(OUT/'cycle_traces.csv',cycle_traces)
    write_csv(OUT/'duty_limits.csv',duty_tests)
    summary=dict(seed=20261109,dt_s=DT,hold_s=1.,total_trials=len(results)+len(held)+len(cycles)+len(duty_tests)+len(stress)+len(representative),training_trials=len(results),
        selected=best,holdout_trials=len(held),holdout_passed=sum(r['passed'] for r in held),
        holdout_faults=dict(Counter(r['fault'] or ('physical_error_or_contact' if not r['passed'] else 'pass') for r in held)),
        duty_trials=len(duty_tests),duty_passed=sum(r['passed'] for r in duty_tests),
        gain_combinations=len(gains),cycle_moves=len(cycles),cycle_passed=sum(r['passed'] for r in cycles),
        cycle_max_error_mm=max(abs(r['actual_error_mm']) for r in cycles),
        stress=stress,training_plants=[asdict(p) for p in plants],
        heldout_plants=[asdict(p) for p in held_plants],
        elapsed_s=time.time()-began,firmware_sha256=hashlib.sha256((ROOT/'pico/main.py').read_bytes()).hexdigest(),
        specs=dict(stroke_mm=630,max_speed_mm_s=38.1,dual_column_desk_capacity_kg=125),
        limitations=['Assumed plant, not identified from real motor data','125 kg is the complete two-motor desk rating',
            'No current, force, temperature, structural, dual-column synchronization or real-time HAL validation',
            'Encoder resolution, load response, friction, latency and self locking are assumptions',
            'One second after-stop observation; five seconds in stress tests; no lifetime durability claim',
            'No physical end-limit input or automatic homing; good calibration and manual zero required'])
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ['heldout_plants','training_plants','stress','limitations']},indent=2))

if __name__=='__main__':main()
