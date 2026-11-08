"""Transport-independent, fail-closed authorization for a single pilot task.

All times passed as ``now`` are monotonic seconds; source timestamps are PX4
DDS synchronized Unix microseconds. No commands or ROS dependencies here.
"""
import math


OFFBOARD = 14
TOKEN_KEYS = ("instance_id", "reset_counter", "reference_epoch", "clock_epoch",
              "px4_xy_reset_counter", "px4_z_reset_counter", "px4_heading_reset_counter")


class PilotAuthority:
    # Observed PX4 DDS cadence: status 2Hz, RC 5Hz, switches/land 1Hz.
    # Margins are the complete loss budget (not an additional retry window).
    LIMITS = {"status": 1.2, "rc": .7, "switches": 1.7,
              "local": .3, "health": .6, "land": 1.6}
    SAMPLE_LIMITS = {"rc": .8, "switches": 1.7, "local": .3}

    def __init__(self, offboard_slot=0, three_position=False):
        if offboard_slot not in range(7):
            raise ValueError("RC Offboard slot must be 0 (dedicated switch) or 1..6")
        self.offboard_slot = offboard_slot
        if three_position and offboard_slot:
            raise ValueError("three-position learning and fixed Offboard slot are mutually exclusive")
        self.three_position = three_position
        self.manual_slots = {}
        self.manual_candidate = None
        self.manual_candidate_since = 0.
        self.selector_fault = ""
        self.data = {}
        self.received = {}
        self.source = {}
        self.samples = {}
        self.publication_age = {}
        self.sample_age = {}
        self.started = False
        self.stopped = False
        self.reason = "waiting_for_pilot"
        self.token = None
        self.saw_manual = False
        self.saw_selector_off = False
        self.local_fault = ""

    def selector(self, switches):
        if self.three_position:
            slot = switches.get("mode_slot", 0)
            return (set(self.manual_slots) == {1, 2} and slot in range(1, 7)
                    and slot not in self.manual_slots.values() and not self.selector_fault)
        # An explicitly configured slot must agree with the physical slot.
        if self.offboard_slot:
            return switches.get("mode_slot") == self.offboard_slot
        return switches.get("offboard_switch") == 1

    def learn_manual_selector(self, now):
        """Bind the two manual detents before authorizing the third detent.

        Stable pairing avoids learning a new slot under a delayed old status
        message during an ordinary selector -> commander mode transition.
        """
        if not self.three_position or self.started or self.stopped:
            return
        if any(self.freshness_reason(k, now) for k in ("status", "rc", "switches")):
            self.manual_candidate = None
            return
        status, rc, switches = (self.data[k] for k in ("status", "rc", "switches"))
        mode, slot = status.get("nav_state"), switches.get("mode_slot", 0)
        if (mode not in (1, 2) or status.get("intended_mode") != mode
                or status.get("failsafe", True) or not rc.get("valid") or rc.get("data_source") != 1
                or slot not in range(1, 7) or any(switches.get(k, 0) == 1 for k in
                   ("return_switch", "loiter_switch", "kill_switch", "termination_switch"))):
            self.manual_candidate = None
            return
        pair = mode, slot
        if pair != self.manual_candidate:
            self.manual_candidate, self.manual_candidate_since = pair, now
            return
        if now-self.manual_candidate_since < 1.2:
            return
        if mode in self.manual_slots and self.manual_slots[mode] != slot:
            self.selector_fault = "rc_mode_slot_changed"
        elif any(other != mode and known == slot for other, known in self.manual_slots.items()):
            self.selector_fault = "rc_manual_slots_not_distinct"
        else:
            self.manual_slots[mode] = slot

    def selector_configuration_reason(self):
        if self.selector_fault:
            return self.selector_fault
        if self.three_position:
            if 1 not in self.manual_slots:
                return "observe_altitude_detent"
            if 2 not in self.manual_slots:
                return "observe_position_detent"
        return ""

    def ingest(self, stream, stamp_us, now, wall_us, payload, sample_us=None):
        """stamp_us is publication time; sample_us may legitimately repeat.

        Exact publication duplicates never renew a lease. New publications with
        unchanged samples are accepted only while the raw sample remains fresh.
        Safety payload changes (RC invalid, selector off) are still processed.
        """
        limit = self.LIMITS[stream]
        if sample_us is None:
            sample_us = stamp_us
        if (type(stamp_us) is not int or stamp_us <= 0
                or stamp_us < self.source.get(stream, 0)
                or not -.05 <= (wall_us - stamp_us) / 1e6 <= limit):
            self.data.pop(stream, None)
            return False
        old = self.data.get(stream)
        if stamp_us == self.source.get(stream):
            if payload != old or sample_us != self.samples.get(stream):
                self.data.pop(stream, None)  # conflicting same-publication payload
            return False  # identical retransmission: retain data, do not renew
        if (type(sample_us) is not int or sample_us <= 0 or sample_us > stamp_us+50000
                or sample_us < self.samples.get(stream, 0)
                or not -.05 <= (wall_us-sample_us)/1e6 <= self.SAMPLE_LIMITS.get(stream, limit)):
            self.data.pop(stream, None)
            return False
        previous_stamp = self.samples.get(stream, 0)
        self.source[stream] = stamp_us
        self.samples[stream] = sample_us
        self.received[stream] = now
        self.publication_age[stream] = max(0., (wall_us-stamp_us)/1e6)
        self.sample_age[stream] = max(0., (wall_us-sample_us)/1e6)
        self.data[stream] = dict(payload)
        if stream == "status" and payload.get("nav_state") != OFFBOARD:
            self.saw_manual = True
        if stream == "switches" and not self.selector(payload):
            self.saw_selector_off = True
        if stream == "local" and old is not None and self.started:
            dt = (sample_us - previous_stamp) / 1e6
            if 0 < dt <= self.LIMITS["local"] and self._finite_local(old) and self._finite_local(payload):
                predicted = tuple(.5 * (a + b) * dt for a, b in zip(old["velocity"], payload["velocity"]))
                residual = math.dist(tuple(b - a for a, b in zip(old["position"], payload["position"])), predicted)
                angle = abs(math.remainder(payload["heading"] - old["heading"], 2 * math.pi))
                if residual > .5 or angle > math.radians(10 + 180 * dt):
                    self.local_fault = "local_pose_discontinuity"
        return True

    def ages(self, now):
        return {stream: dict(receipt_s=max(0., now-rx),
                            publication_s=self.publication_age[stream]+max(0., now-rx),
                            sample_s=self.sample_age[stream]+max(0., now-rx))
                for stream, rx in self.received.items()}

    def freshness_reason(self, stream, now):
        if stream not in self.data or not 0 <= now-self.received[stream] <= self.LIMITS[stream]:
            return "stale_"+stream
        elapsed = now-self.received[stream]
        if self.publication_age[stream]+elapsed > self.LIMITS[stream]:
            return "stale_"+stream
        if self.sample_age[stream]+elapsed > self.SAMPLE_LIMITS.get(stream, self.LIMITS[stream]):
            return "stale_"+stream+"_sample"
        return ""

    @staticmethod
    def _finite_local(p):
        try:
            return (len(p["position"]) == len(p["velocity"]) == 3
                    and all(math.isfinite(x) for x in (*p["position"], *p["velocity"], p["heading"])))
        except (KeyError, TypeError, ValueError):
            return False

    def ready_reason(self, now):
        if self.stopped:
            return self.reason
        if self.selector_configuration_reason():
            return self.selector_configuration_reason()
        for stream in self.LIMITS:
            reason = self.freshness_reason(stream, now)
            if reason:
                return reason
        s, rc, sw, local, health, land = (self.data[k] for k in self.LIMITS)
        if not s.get("armed") or s.get("failsafe", True):
            return "disarmed_or_failsafe"
        if not rc.get("valid") or rc.get("data_source") != 1:
            return "physical_rc_unavailable"
        if any(sw.get(k, 0) == 1 for k in ("return_switch", "loiter_switch", "kill_switch", "termination_switch")):
            return "pilot_override_switch"
        if land.get("landed", True) or land.get("freefall", True):
            return "manual_takeoff_required"
        if not local.get("valid") or not self._finite_local(local):
            return "position_invalid"
        if self.local_fault:
            return self.local_fault
        if (health.get("schema_version") != 1 or health.get("navigation_ready") is not True
                or health.get("airborne_fault") is not False or health.get("requires_disarm") is not False
                or health.get("armed") is not True or not health.get("instance_id")
                or any(k not in health for k in TOKEN_KEYS)):
            return "vio_unready"
        if tuple(local.get("resets", ())) != tuple(health[k] for k in TOKEN_KEYS[-3:]):
            return "estimator_frame_mismatch"
        if self.started and tuple(health[k] for k in TOKEN_KEYS) != self.token:
            return "reference_changed"
        return ""

    def acquire(self, now):
        reason = self.ready_reason(now)
        if reason:
            self.reason = reason
            return False
        s, sw = self.data["status"], self.data["switches"]
        if not self.saw_manual or not self.saw_selector_off:
            self.reason = "cycle_rc_selector_before_start"
            return False
        if not self.selector(sw) or s.get("nav_state") != OFFBOARD or s.get("intended_mode") != OFFBOARD:
            self.reason = "waiting_for_rc_offboard"
            return False
        self.token = tuple(self.data["health"][k] for k in TOKEN_KEYS)
        self.started = True
        self.reason = "pilot_authorized"
        return True

    def authorized(self, now):
        if not self.started or self.stopped:
            return False
        reason = self.ready_reason(now)
        if not reason:
            s = self.data["status"]
            if not self.selector(self.data["switches"]) or s.get("nav_state") != OFFBOARD or s.get("intended_mode") != OFFBOARD:
                reason = "pilot_takeover"
        if reason:
            self.revoke(reason)
            return False
        return True

    def prestream(self, now):
        # PX4 needs >1 s of proof-of-life BEFORE accepting RC Offboard selection.
        # Only OffboardControlMode may be sent here; NEVER a trajectory/command.
        return (not self.started and not self.ready_reason(now) and self.saw_manual
                and self.saw_selector_off and self.data["status"].get("nav_state") != OFFBOARD)

    def revoke(self, reason):
        self.stopped = True
        self.reason = reason
