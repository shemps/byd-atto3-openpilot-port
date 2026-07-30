"""BYD Atto 3 RadarInterface — Veoneer MVS4 private-bus object tracks (carrot tree).

Parses RADAR_TRACK_00..09 (0x280-0x289, 64-byte FD frames on panda bus 1 = the
private radar<->camera bus, tapped READ-ONLY) into RadarData points, written
against the carrot (ajouatom) RadarInterfaceBase API: base update_carrot()
supplies self.v_ego (radarDelay-compensated) and derives smoothed
aLead/jLead/yRel per track via MyTrack from the vLead we set.

Every mapping was replay-validated OFFLINE against two both-bus drive captures
before this file existed (harness/radar_replay_validate.py, 2026-07-15):
  - integrity: CRC8/J1850+DataID passed 287k/287k frames across both drives
  - VLEAD is ABSOLUTE lead velocity: d(LONG_DIST)/dt tracks VLEAD - vEgo with
    r=0.93-0.95, slope 0.97-0.99 (ego = ESC 0x121 * 0.0725)
      ->  pt.vLead = VLEAD;  pt.vRel = VLEAD - self.v_ego
  - LAT_DIST is already ISO +y=left (oncoming traffic in left-hand
    traffic sits at negative LAT)  ->  yRel = +LAT_DIST
  - lead coverage vs the camera's fused HUD lead flag while ACC-active:
    96.6% / 99.8% on the two drives
  - 0x28A-0x29D are overflow slots for the 11th+ object (dense traffic only,
    rare) — not in the DBC, not parsed; extend byd_radar_fd.dbc if needed

Filter constants mirror the validated reference implementation in
veoneer_tracks.py (the bench module the replay validator exercises).

Deployment (M9 carrot): interface.py must set radarUnavailable=False,
radarTimeStep=0.075 (slots repeat at ~13.3 Hz; feeds base estimate_dt) and
leave radarDelay 0.0 until measured. byd_radar_fd.dbc already ships with the
port. The panda bus-1 monitor-mode fw change (silent tap, no ACK) is a
separate task and does not affect this RX-only code.
"""
from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase

RADAR_DBC = "byd_radar_fd"
N_SLOTS = 10
TRIGGER_MSG = 0x289          # last slot; all 10 slots TX every ~75 ms cycle
EMPTY_TRACK_ID = 255
# When the Veoneer is idle (parked/standstill it stops TXing tracks), publish an
# empty RadarData at ~1/IDLE_PUBLISH_DIV of the call rate so radarState stays fresh
# and doesn't go stale -> radarFault (NO_ENTRY) -> block engagement. IDLE_FRAMES
# must exceed the active inter-trigger gap (~7-8 card cycles at 13 Hz radar / 100 Hz
# card) so this never fires while the radar is actually streaming.
IDLE_FRAMES = 20
IDLE_PUBLISH_DIV = 5
# Fixed filter thresholds (= veoneer_tracks.py, keep in sync). Checked against
# the replay validator percentiles (RADAR_VALIDATION_2026-07-15.md): CONFIDENCE
# is binary 0/1 (0.30 sits in the dead band), LONG_DIST p99 161 m (< 200),
# ACC-active lead coverage 96.6%/99.8%. Defaults hold; no tuning warranted.
MIN_CONF = 0.30
MAX_LONG_M = 200.0
# Cut-in gate (2026-07-20, TASKS 6b): Veoneer CONFIDENCE ramps 0->1 over ~1.9 s on
# newborn tracks, so MIN_CONF alone hides close cut-ins from radard for ~1.4 s (route-6b
# ramp merge). Accept low-conf tracks that are close, in-corridor, persistent >= 2
# cycles, and CO-MOVING (VLEAD is absolute; > 1 m/s rejects stationary clutter and
# oncoming traffic — those dominated the raw gate's false accepts on drive_v3).
# Replay-validated 2026-07-20: 0 phantom leads while stock-ACC-active on drive_v3 +
# drive_cam_v6; on routes 6b/7b ~400 cut-in-class births each, visibility gains
# cluster at +1.25-1.40 s. radard's own establishment still filters sub-second
# transients out of leadOne.
CUTIN_MAX_LONG_M = 20.0
CUTIN_MAX_LAT_M = 3.5
CUTIN_MIN_CYCLES = 2
CUTIN_MIN_VLEAD_MS = 1.0


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.radar_off_can = CP.radarUnavailable
    self.last_track_frame = 0
    self.rcp = None
    if not self.radar_off_can:
      messages = [(f"RADAR_TRACK_{i:02d}", 13) for i in range(N_SLOTS)]
      self.rcp = CANParser(RADAR_DBC, messages, 1)
      # per-slot (tid, consecutive-cycle count) for the cut-in gate debounce
      self.seen = {}
      # pts are keyed by transport slot; trackId carries the radar's own
      # object id so the base MyTrack smoothing follows objects across slots
      for slot in range(N_SLOTS):
        pt = structs.RadarData.RadarPoint()
        pt.trackId = slot
        pt.measured = False
        self.pts[slot] = pt

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    self.updated_messages.update(self.rcp.update(can_strings))
    self.frame += 1
    if TRIGGER_MSG not in self.updated_messages:
      # Radar idle (parked/standstill, no track frames). Once it's been idle past
      # the active inter-trigger gap, emit an empty RadarData at a steady low rate
      # so radarState stays fresh (no radarFault). Never fabricate tracks.
      if (self.frame - self.last_track_frame) > IDLE_FRAMES and (self.frame % IDLE_PUBLISH_DIV) == 0:
        for pt in self.pts.values():
          pt.measured = False
        ret = structs.RadarData()
        ret.points = list(self.pts.values())
        return ret
      return None
    self.updated_messages.clear()
    self.last_track_frame = self.frame

    for slot in range(N_SLOTS):
      sig = self.rcp.vl[f"RADAR_TRACK_{slot:02d}"]
      pt = self.pts[slot]
      # NB not `or EMPTY_TRACK_ID`: tid 0 is a valid track id per the DBC (0-254
      # live; never yet observed on this radar - live tids start at 1 - but the
      # `or` idiom would silently drop it as an empty slot).
      raw_tid = sig["TRACK_ID"]
      tid = EMPTY_TRACK_ID if raw_tid is None else int(raw_tid)
      long_dist = float(sig["LONG_DIST"] or 0.0)
      conf = float(sig["CONFIDENCE"] or 0.0)
      lat_dist = float(sig["LAT_DIST"] or 0.0)
      v_lead = float(sig["VLEAD"] or 0.0)
      if tid == EMPTY_TRACK_ID:
        self.seen.pop(slot, None)
        pt.measured = False
        continue
      prev = self.seen.get(slot)
      seen = prev[1] + 1 if prev is not None and prev[0] == tid else 1
      self.seen[slot] = (tid, seen)
      conf_ok = conf >= MIN_CONF or (long_dist < CUTIN_MAX_LONG_M
                                     and abs(lat_dist) <= CUTIN_MAX_LAT_M
                                     and seen >= CUTIN_MIN_CYCLES
                                     and v_lead > CUTIN_MIN_VLEAD_MS)
      if not conf_ok or not (0.0 < long_dist <= MAX_LONG_M):
        pt.measured = False
        continue
      pt.trackId = tid
      pt.dRel = long_dist
      pt.yRel = lat_dist                     # validated: already ISO +y = left
      pt.vLead = v_lead                      # validated: VLEAD is absolute
      pt.vRel = v_lead - self.v_ego          # base supplies delay-compensated v_ego
      pt.measured = True

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    ret.points = list(self.pts.values())
    return ret
