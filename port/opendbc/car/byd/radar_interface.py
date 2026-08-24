"""BYD Atto 3 RadarInterface — Veoneer MVS4 private-bus object tracks.

Parses RADAR_TRACK_00..09 (0x280-0x289, 64-byte FD frames on panda bus 1 = the
private radar<->camera bus, tapped READ-ONLY) into RadarData points. Ported to
the stock opendbc RadarInterfaceBase from the validated carrot-tree version:
the base class there supplied a radarDelay-compensated self.v_ego and derived
aLead/jLead per track; stock radard does its own lead processing, so here we
only need vRel — computed against ego speed parsed from the chassis bus
(0x1F0 WHEELSPEED_CLEAN, the same message carstate reads), because VLEAD is
ABSOLUTE lead velocity.

Every mapping was replay-validated OFFLINE against two both-bus drive captures
before this file existed (harness/radar_replay_validate.py, 2026-07-15):
  - integrity: CRC8/J1850+DataID passed 287k/287k frames across both drives
  - VLEAD is ABSOLUTE lead velocity: d(LONG_DIST)/dt tracks VLEAD - vEgo with
    r=0.93-0.95, slope 0.97-0.99 (ego = ESC 0x121 * 0.0725)
      ->  pt.vRel = VLEAD - v_ego
  - LAT_DIST is already ISO +y=left (oncoming traffic in Thai left-hand
    traffic sits at negative LAT)  ->  yRel = +LAT_DIST
  - LONG_DIST is NOT sensor-referenced: it comes from the ego rear-axle origin, so
    dRel = LONG_DIST - BUMPER_TO_RADAR_ORIGIN_M (FINDINGS §16, measured 2026-08-19
    against four leads of known body width; the raw value ran ~3.2 m long on cars
    and trucks alike)
  - lead coverage vs the camera's fused HUD lead flag while ACC-active:
    96.6% / 99.8% on the two drives
  - 0x28A-0x29D are overflow slots for the 11th+ object (dense traffic only,
    rare) — not in the DBC, not parsed; extend byd_radar_fd.dbc if needed

Filter constants mirror the validated reference implementation in
veoneer_tracks.py (the bench module the replay validator exercises).

Deployment: interface.py must set radarUnavailable=False, radarTimeStep=0.075
(slots repeat at ~13.3 Hz) and leave radarDelay 0.0 until measured.
byd_radar_fd.dbc ships with the port. The panda bus-1 monitor-mode fw change
(silent tap, no ACK) is a separate task and does not affect this RX-only code.
"""
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import RadarData
from opendbc.car.byd.values import DBC
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

# RANGE ORIGIN (FINDINGS §16, measured 2026-08-19). LONG_DIST is referenced to the ego
# REAR-AXLE centre - the ISO 8855 vehicle frame - not to the sensor face. openpilot's
# dRel must be measured from the FRONT BUMPER, so the bumper->rear-axle distance comes
# off every track: front overhang 885 mm (dimension drawing) + wheelbase 2720 mm.
# Evidence: on four leads of KNOWN body width the raw range ran +3.21/+4.11/+2.04/+3.62 m
# long (Corolla taxi, CR-V, Yaris-class hatch, box truck) while the vision lead sat within
# 0.1-0.8 m of the same geometry, and cars were biased exactly as much as the truck - so
# this is the sensor's origin, not a reflection point on the target. Acceptance test
# (byd_port/lead_range_acceptance.py, 7863 radar-matched frames): the two rulers disagreed
# by +4.18 m raw, +0.43 m in the 0-12 m stopping band after this subtraction.
BUMPER_TO_RADAR_ORIGIN_M = 3.60
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
    self.v_ego = 0.0
    if not self.radar_off_can:
      messages = [(f"RADAR_TRACK_{i:02d}", 13) for i in range(N_SLOTS)]
      self.rcp = CANParser(RADAR_DBC, messages, 1)
      # ego speed off the chassis bus: VLEAD is absolute, vRel needs vEgo and the
      # stock RadarInterfaceBase doesn't supply one
      self.cp_ego = CANParser(DBC["BYD_ATTO_3"][Bus.pt], [("WHEELSPEED_CLEAN", 50)], 0)
      # per-slot (tid, consecutive-cycle count) for the cut-in gate debounce
      self.seen = {}

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    self.cp_ego.update(can_strings)
    if self.cp_ego.can_valid:
      self.v_ego = float(self.cp_ego.vl["WHEELSPEED_CLEAN"]["WHEELSPEED_CLEAN"]) * CV.KPH_TO_MS

    self.updated_messages.update(self.rcp.update(can_strings))
    self.frame += 1
    if TRIGGER_MSG not in self.updated_messages:
      # Radar idle (parked/standstill, no track frames). Once it's been idle past
      # the active inter-trigger gap, emit an empty RadarData at a steady low rate
      # so radarState stays fresh (no radarFault). Never fabricate tracks.
      if (self.frame - self.last_track_frame) > IDLE_FRAMES and (self.frame % IDLE_PUBLISH_DIV) == 0:
        self.pts.clear()
        self.seen.clear()
        return RadarData()
      return None
    self.updated_messages.clear()
    self.last_track_frame = self.frame

    for slot in range(N_SLOTS):
      sig = self.rcp.vl[f"RADAR_TRACK_{slot:02d}"]
      # NB not `or EMPTY_TRACK_ID`: tid 0 is a valid track id per the DBC (0-254
      # live; never yet observed on this radar - live tids start at 1 - but the
      # `or` idiom would silently drop it as an empty slot).
      raw_tid = sig["TRACK_ID"]
      tid = EMPTY_TRACK_ID if raw_tid is None else int(raw_tid)
      raw_long = float(sig["LONG_DIST"] or 0.0)
      # bumper-referenced, per BUMPER_TO_RADAR_ORIGIN_M above. Two knock-ons, both
      # deliberate: the MAX_LONG_M gate below now also drops the non-physical negatives
      # (a target touching the bumper reads ~3.6 raw) instead of publishing a lead from
      # behind the bumper, and the CUTIN_MAX_LONG_M window becomes a real 20 m of road
      # rather than the 16.4 m it silently was.
      long_dist = raw_long - BUMPER_TO_RADAR_ORIGIN_M
      conf = float(sig["CONFIDENCE"] or 0.0)
      lat_dist = float(sig["LAT_DIST"] or 0.0)
      v_lead = float(sig["VLEAD"] or 0.0)
      if tid == EMPTY_TRACK_ID:
        self.seen.pop(slot, None)
        self.pts.pop(slot, None)
        continue
      prev = self.seen.get(slot)
      seen = prev[1] + 1 if prev is not None and prev[0] == tid else 1
      self.seen[slot] = (tid, seen)
      conf_ok = conf >= MIN_CONF or (long_dist < CUTIN_MAX_LONG_M
                                     and abs(lat_dist) <= CUTIN_MAX_LAT_M
                                     and seen >= CUTIN_MIN_CYCLES
                                     and v_lead > CUTIN_MIN_VLEAD_MS)
      if not conf_ok or not (0.0 < long_dist <= MAX_LONG_M):
        self.pts.pop(slot, None)
        continue
      # pts are keyed by transport slot; trackId carries the radar's own object id
      # so radard's per-track processing follows objects across slots
      pt = RadarData.RadarPoint()
      pt.trackId = tid
      pt.dRel = long_dist
      pt.yRel = lat_dist                     # validated: already ISO +y = left
      pt.vRel = v_lead - self.v_ego          # validated: VLEAD is absolute
      self.pts[slot] = pt

    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    ret.points = list(self.pts.values())
    return ret
