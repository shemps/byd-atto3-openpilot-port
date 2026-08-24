"""BYD Atto 3 RadarInterface — Veoneer MVS4 private-bus object tracks.

This module parses RADAR_TRACK_00 to RADAR_TRACK_09, addresses 0x280 to 0x289,
into RadarData points. The frames are 64-byte FD frames on panda bus 1, the
private radar-to-camera bus, which this port reads only.

radard does its own lead processing, so this module supplies vRel only. VLEAD is
the absolute lead velocity, so vRel needs the ego speed. The ego speed comes from
the chassis bus, 0x1F0 WHEELSPEED_CLEAN, which is the message carstate reads.

Signal mapping:
  - Frame integrity is CRC8/J1850 with DataID.
  - VLEAD is the absolute lead velocity.  ->  pt.vRel = VLEAD - v_ego
  - LAT_DIST is ISO +y = left.  ->  yRel = +LAT_DIST
  - LONG_DIST comes from the ego rear-axle origin.
      ->  dRel = LONG_DIST - BUMPER_TO_RADAR_ORIGIN_M
  - 0x28A to 0x29D are overflow slots for the 11th object and above. They are not
    in the DBC and this module does not parse them. Add them to byd_radar_fd.dbc
    if you need them.

The filter constants are the same as the reference implementation in
veoneer_tracks.py.

To deploy: interface.py must set radarUnavailable = False, and radarDelay stays
0.0 until you measure it. byd_radar_fd.dbc is part of the port. The panda bus-1
monitor-mode firmware change gives a silent tap with no ACK. It is separate and
does not affect this receive-only code.
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
# Fixed filter thresholds. They are the same as veoneer_tracks.py. Keep the two in
# agreement. CONFIDENCE is binary 0 or 1, so 0.30 sits in the dead band.
MIN_CONF = 0.30
MAX_LONG_M = 200.0

# Range origin. LONG_DIST is referenced to the ego rear-axle centre, the ISO 8855 vehicle
# frame, and not to the sensor face. openpilot dRel is referenced to the front bumper, so
# subtract the bumper-to-rear-axle distance from each track: front overhang 885 mm plus
# wheelbase 2720 mm. Without the subtraction every lead reads 3.6 m too far. With it, radar
# and vision agree to within 0.5 m below 12 m.
BUMPER_TO_RADAR_ORIGIN_M = 3.60
# Cut-in gate. On a new track the Veoneer CONFIDENCE goes from 0 to 1 over about 1.9 s, so
# MIN_CONF alone hides a close cut-in from radard for about 1.4 s. Accept a low-confidence
# track if it is close, inside the corridor, present for 2 cycles or more, and moving in the
# same direction. VLEAD is absolute, so the 1 m/s limit rejects stationary clutter and
# oncoming traffic. radard still removes transients shorter than one second from leadOne.
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
      # Referenced to the bumper, per BUMPER_TO_RADAR_ORIGIN_M above. This has two
      # intentional effects. The MAX_LONG_M gate below also removes negative ranges, because
      # a target at the bumper reads about 3.6 raw. The CUTIN_MAX_LONG_M window becomes a
      # true 20 m of road, instead of the 16.4 m it was before.
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
