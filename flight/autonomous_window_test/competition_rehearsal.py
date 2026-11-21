"""Offline competition workflow, with NO ROS, serial, PWM or flight transport.

Inputs are explicit simulated observations, not live detector messages. Identical
rings do not identify a numbered target: the future localization adapter must
establish target identity. This module must not be connected to an actuator.
"""
import math
import random
from dataclasses import dataclass


DEFAULT_TIMING = dict(observation_timeout_s=.35, ack_timeout_s=2.,
                      stable_time_s=1., light_time_s=3., acquire_timeout_s=10.,
                      move_timeout_s=30., handover_timeout_s=4.)
TERMINAL = ('DONE', 'ABORT')


def select_targets(sequence=None, random_two=False, seed=None):
    if type(random_two) is not bool:
        raise ValueError('random_two must be boolean')
    if random_two:
        if sequence is not None:
            raise ValueError('choose an explicit sequence OR random selection')
        return tuple(random.Random(seed).sample((1, 2, 3), 2))
    selected = (1,) if sequence is None else tuple(sequence)
    if (not 1 <= len(selected) <= 3 or
            any(type(n) is not int or n not in (1, 2, 3) for n in selected) or
            len(set(selected)) != len(selected)):
        raise ValueError('sequence must contain unique target IDs from 1, 2, 3')
    return selected


@dataclass(frozen=True)
class Sample:
    # All times use the same synthetic monotonic clock. Not ROS header epochs.
    time: float
    pose_ok: bool = True
    hover_ready: bool = False
    pilot_override: bool = False
    observed_mode: str = 'OFFBOARD'
    target_id: object = None
    observation_id: object = None
    observation_time: object = None
    target_valid: bool = False
    centred: bool = False
    stable: bool = False
    at_requested_target: bool = False
    # Explicit simulated physical feedback; an ACK alone is not hit evidence.
    spot_inside: bool = False
    ack_token: object = None
    ack_on: object = None
    ack_time: object = None


class Rehearsal:
    """One-shot workflow; output contains intents only, never hardware commands."""
    def __init__(self, sequence=(1,), timing=None):
        self.sequence = select_targets(sequence)
        supplied = timing or {}
        if set(supplied) - set(DEFAULT_TIMING):
            raise ValueError('unknown timing option')
        self.timing = dict(DEFAULT_TIMING, **supplied)
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0
               for v in self.timing.values()):
            raise ValueError('timing must be positive finite numbers')
        self.phase = 'WAIT_HOVER'
        self.reason = 'await_simulated_hover'
        self.index = 0
        self.current_location = 1
        self.requested_target = None
        self.started = self.entered = self.last_time = None
        self.stable_since = self.light_since = None
        self.last_observation_id = self.last_observation_time = None
        self.token = 0
        self.light_intent = None
        self.light_request_time = None
        self.off_confirmed = False
        self.completed = []
        self.events = []

    @property
    def target(self):
        return self.sequence[self.index]

    def _transition(self, phase, reason, time):
        self.events.append(dict(time=time, previous=self.phase, phase=phase, reason=reason))
        self.phase, self.reason, self.entered = phase, reason, time
        self.stable_since = None
        self.requested_target = None

    def _light(self, on, time):
        self.token += 1
        self.light_intent = bool(on)
        self.light_request_time = time
        self.off_confirmed = False

    def _abort(self, reason, time):
        if self.phase in TERMINAL:
            return self.output()
        self._light(False, time)
        self._transition('ABORT', reason, time)
        return self.output()

    def output(self):
        return dict(execution='offline_rehearsal_only', phase=self.phase,
                    reason=self.reason, selected_targets=list(self.sequence),
                    target_id=self.target, completed_targets=list(self.completed),
                    simulated_light_request=self.light_intent,
                    simulated_light_request_token=self.token,
                    simulated_light_off_confirmed=self.off_confirmed,
                    simulated_move_to_target=self.requested_target,
                    simulated_handover_request=self.phase == 'HANDOVER',
                    hardware_commands_sent=False, physical_hit_verified=False)

    def _ack_matches(self, sample, on):
        return (type(sample.ack_token) is int and sample.ack_token == self.token and
                sample.ack_on is on and type(sample.ack_time) in (int, float) and
                math.isfinite(sample.ack_time) and
                self.light_request_time <= sample.ack_time <= sample.time and
                sample.time - sample.ack_time <= self.timing['ack_timeout_s'])

    def _observation(self, sample):
        stamp = sample.observation_time
        valid = (type(sample.observation_id) is int and
                 type(stamp) in (int, float) and math.isfinite(stamp) and
                 0 <= sample.time - stamp <= self.timing['observation_timeout_s'] and
                 sample.target_valid is True and type(sample.target_id) is int and
                 sample.target_id == self.target)
        if not valid:
            return False, False
        new = (self.last_observation_id is None or
               (sample.observation_id > self.last_observation_id and stamp > self.last_observation_time))
        same = (sample.observation_id == self.last_observation_id and stamp == self.last_observation_time)
        if not new and not same:
            return False, False
        if new:
            self.last_observation_id, self.last_observation_time = sample.observation_id, stamp
        return True, new

    def tick(self, sample):
        if self.phase in TERMINAL:
            return self.output()
        now = sample.time
        if type(now) not in (int, float) or not math.isfinite(now):
            return self._abort('invalid_clock', self.last_time or 0.)
        if self.started is None:
            self.started = self.entered = now
            self._light(False, now)
        if self.last_time is not None:
            gap = now - self.last_time
            if gap < 0 or gap > self.timing['observation_timeout_s']:
                return self._abort('clock_jump_or_tick_gap', now)
        self.last_time = now
        bool_fields = ('pose_ok', 'hover_ready', 'pilot_override', 'target_valid',
                       'centred', 'stable', 'at_requested_target', 'spot_inside')
        if any(type(getattr(sample, name)) is not bool for name in bool_fields):
            return self._abort('malformed_boolean_input', now)
        if sample.pilot_override:
            return self._abort('pilot_override_no_reacquisition', now)
        if not sample.pose_ok:
            return self._abort('pose_unavailable', now)
        if self.phase != 'WAIT_HOVER' and not (
                sample.observed_mode == 'OFFBOARD' or
                self.phase == 'HANDOVER' and sample.observed_mode == 'POSCTL'):
            return self._abort('mode_changed_no_reacquisition', now)
        elapsed = now - self.entered
        if self.phase == 'WAIT_HOVER':
            self.off_confirmed = self._ack_matches(sample, False)
            if not self.off_confirmed and now - self.light_request_time > self.timing['ack_timeout_s']:
                return self._abort('startup_off_not_confirmed', now)
            if sample.hover_ready and self.off_confirmed and sample.observed_mode == 'OFFBOARD':
                if self.target == self.current_location:
                    self._transition('ACQUIRE', 'hover_ready_acquire_target', now)
                else:
                    self._transition('MOVE', 'move_to_first_selected_target_light_off', now)
                    self.requested_target = self.target
            elif elapsed > self.timing['move_timeout_s']:
                return self._abort('hover_timeout', now)
        elif self.phase == 'ACQUIRE':
            valid, new = self._observation(sample)
            if not valid or not sample.centred or not sample.stable:
                self.stable_since = None
                self.reason = 'await_unique_centred_stable_target'
            elif new:
                if self.stable_since is None:
                    self.stable_since = sample.observation_time
                if sample.observation_time - self.stable_since >= self.timing['stable_time_s']:
                    self._light(True, now)
                    self._transition('ON_WAIT', 'await_matching_simulated_light_ack', now)
            if self.phase == 'ACQUIRE' and elapsed > self.timing['acquire_timeout_s']:
                return self._abort('target_acquisition_timeout', now)
        elif self.phase in ('ON_WAIT', 'LIGHTING'):
            valid, new = self._observation(sample)
            if not valid or not sample.centred or not sample.stable:
                return self._abort('target_or_stability_lost', now)
            if self.phase == 'ON_WAIT':
                if self._ack_matches(sample, True):
                    self.light_since = sample.ack_time
                    self._transition('LIGHTING', 'await_continuous_simulated_spot_evidence', now)
                elif elapsed > self.timing['ack_timeout_s']:
                    return self._abort('light_on_ack_timeout', now)
            else:
                if not self._ack_matches(sample, True):
                    return self._abort('light_status_lost_or_changed', now)
                if not sample.spot_inside:
                    return self._abort('spot_outside_or_missing', now)
                if new and sample.observation_time >= self.light_since:
                    if self.stable_since is None:
                        self.stable_since = sample.observation_time
                    if sample.observation_time - self.stable_since >= self.timing['light_time_s']:
                        self._light(False, now)
                        self._transition('OFF_WAIT', 'await_matching_off_before_completion_or_move', now)
                if self.phase == 'LIGHTING' and now - self.light_since > self.timing['light_time_s'] + self.timing['ack_timeout_s']:
                    return self._abort('spot_evidence_timeout', now)
        elif self.phase == 'OFF_WAIT':
            if self._ack_matches(sample, False):
                self.off_confirmed = True
                self.completed.append(self.target)
                if self.index + 1 == len(self.sequence):
                    self._transition('HANDOVER', 'await_POSCTL_readback', now)
                else:
                    self.index += 1
                    self._transition('MOVE', 'previous_target_complete_light_off', now)
                    self.requested_target = self.target
            elif elapsed > self.timing['ack_timeout_s']:
                return self._abort('off_not_confirmed_no_movement', now)
        elif self.phase == 'MOVE':
            if not self.off_confirmed or self.light_intent is not False or not self._ack_matches(sample, False):
                return self._abort('movement_requires_confirmed_off', now)
            self.requested_target = self.target
            if (sample.at_requested_target and sample.stable and
                    type(sample.target_id) is int and sample.target_id == self.target):
                self.current_location = self.target
                self.last_observation_id = self.last_observation_time = None
                self._transition('ACQUIRE', 'arrival_requires_new_visual_acquisition', now)
            elif elapsed > self.timing['move_timeout_s']:
                return self._abort('move_timeout', now)
        elif self.phase == 'HANDOVER':
            if not self._ack_matches(sample, False):
                return self._abort('off_status_lost_during_handover', now)
            if sample.observed_mode == 'POSCTL':
                self._transition('DONE', 'pilot_control_observed_pilot_lands', now)
            elif elapsed > self.timing['handover_timeout_s']:
                return self._abort('handover_not_observed', now)
        return self.output()
