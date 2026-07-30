"""Veoneer MVS4 private-bus radar track state for the BYD Atto 3.

Pure logic, no imports — the REFERENCE implementation exercised by the bench
replay validator (harness/radar_replay_validate.py, cantools decode of text
logs). The deployed carrot RadarInterface (radar_interface.py) mirrors this
module's filter constants but uses the carrot base-class track lifecycle
(pre-allocated pts + MyTrack smoothing) instead of slot expiry — keep the
thresholds here and there in sync. Both decode RADAR_TRACK_00..09
(0x280-0x289, 64-byte FD frames, byd_radar_fd.dbc).

Signal semantics (tembikai DBC claims, arbitrated by the replay validator —
see the validator report in byd_port/ before trusting scaling changes):
  TRACK_ID    0-254 live, 255 = empty slot
  LONG_DIST   m ahead of the radar (front bumper FLR)
  LAT_DIST    m lateral
  VLEAD       m/s (relative vs absolute is decided by the validator)
  ALEAD       m/s^2
  CONFIDENCE  0..1 (25-bit fraction)
"""

EMPTY_TRACK_ID = 255
N_SLOTS = 10          # RADAR_TRACK_00..09; 0x28A-0x29D exist on the bus but are
                      # not in the DBC — validator reports whether they carry data

# ponytail: fixed thresholds, tune from validator percentiles if the defaults
# drop real leads (min_conf) or pass clutter (max_long).
# Loop ran once 2026-07-17 vs the 07-15 percentile report (RADAR_VALIDATION doc):
# values checked and KEPT — retune only when a drive shows either symptom.
DEFAULT_EXPIRY_S = 0.5
DEFAULT_MIN_CONF = 0.30
DEFAULT_MAX_LONG_M = 200.0
LEAD_MAX_LAT_M = 2.0


class VeoneerTracks:
  """Slot-keyed latest-track state with expiry + plausibility filtering."""

  def __init__(self, expiry=DEFAULT_EXPIRY_S, min_conf=DEFAULT_MIN_CONF,
               max_long=DEFAULT_MAX_LONG_M):
    self.expiry = expiry
    self.min_conf = min_conf
    self.max_long = max_long
    self.slots = {}   # slot -> dict(t, slot, tid, long, lat, vlead, alead, conf)

  def update(self, slot, sig, t):
    """Feed one decoded RADAR_TRACK_XX signal dict for `slot` at time `t`."""
    tid = sig.get("TRACK_ID")
    if tid is None or int(tid) == EMPTY_TRACK_ID:
      self.slots.pop(slot, None)
      return
    self.slots[slot] = {
      "t": t, "slot": slot, "tid": int(tid),
      "long": float(sig.get("LONG_DIST") or 0.0),
      "lat": float(sig.get("LAT_DIST") or 0.0),
      "vlead": float(sig.get("VLEAD") or 0.0),
      "alead": float(sig.get("ALEAD") or 0.0),
      "conf": float(sig.get("CONFIDENCE") or 0.0),
    }

  def live(self, now):
    """Tracks that are fresh, confident, and physically plausible."""
    out = []
    for slot in list(self.slots):
      tr = self.slots[slot]
      if now - tr["t"] > self.expiry:
        del self.slots[slot]
        continue
      if tr["conf"] < self.min_conf:
        continue
      if not (0.0 < tr["long"] <= self.max_long):
        continue
      out.append(tr)
    return out

  def lead(self, now, max_lat=LEAD_MAX_LAT_M):
    """Nearest in-path track, or None."""
    cands = [tr for tr in self.live(now) if abs(tr["lat"]) <= max_lat]
    return min(cands, key=lambda tr: tr["long"]) if cands else None
