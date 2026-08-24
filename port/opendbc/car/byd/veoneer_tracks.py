"""Veoneer MVS4 private-bus radar track state for the BYD Atto 3.

This module is pure logic with no imports. It is the reference implementation that
the bench replay validator uses. radar_interface.py has the same filter constants,
but a different track lifecycle. Keep the thresholds in the two files the same.
Both decode RADAR_TRACK_00 to RADAR_TRACK_09, addresses 0x280 to 0x289, which are
64-byte FD frames in byd_radar_fd.dbc.

Signal meanings:
  TRACK_ID    0 to 254 live, 255 = empty slot
  LONG_DIST   m from the ego rear-axle origin, the ISO 8855 vehicle frame. It is not
              from the sensor face. This module keeps the raw value.
              radar_interface.py holds the conversion, BUMPER_TO_RADAR_ORIGIN_M.
  LAT_DIST    m lateral
  VLEAD       m/s, absolute
  ALEAD       m/s^2
  CONFIDENCE  0 to 1, a 25-bit fraction
"""

EMPTY_TRACK_ID = 255
N_SLOTS = 10          # RADAR_TRACK_00..09. 0x28A-0x29D exist on the bus but are
                      # not in the DBC.

# Fixed thresholds. Change them only if a drive drops real leads (min_conf) or
# passes clutter (max_long).
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
