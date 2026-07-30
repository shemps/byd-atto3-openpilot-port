""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

# Atto 3 (Yuan Plus EV), 2024 RHD export. Bus-0 chassis set: union of 23.2M frames
# across 13 listen-only drives, confirmed live against the panda (every live ID
# is in this set, zero DLC mismatches). Includes idle/ADAS-only IDs so the
# candidate stays a superset of anything the car emits at fingerprint time.
FINGERPRINTS = {
  CAR.BYD_ATTO_3: [{
    65: 8, 85: 8, 140: 8, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8, 324: 8, 327: 8, 330: 8, 337: 8, 356: 8, 371: 8, 384: 8, 410: 8, 418: 8, 450: 8, 482: 8, 496: 8, 508: 8, 511: 8, 522: 8, 536: 8, 537: 8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8, 588: 8, 629: 8, 639: 8, 660: 8, 665: 8, 692: 8, 694: 8, 724: 8, 748: 8, 786: 8, 790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8, 814: 8, 815: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8, 848: 8, 854: 8, 860: 8, 863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8, 985: 8, 1004: 8, 1023: 8, 1028: 8, 1031: 8, 1036: 8, 1037: 8, 1040: 8, 1048: 8, 1052: 8, 1058: 8, 1062: 8, 1074: 8, 1076: 8, 1098: 8, 1107: 8, 1141: 8, 1168: 8, 1178: 8, 1184: 8, 1189: 8, 1192: 8, 1193: 8, 1211: 8, 1215: 8, 1217: 8, 1246: 8, 1252: 8, 1274: 8, 1278: 8, 1297: 8, 1298: 8, 1319: 8, 1322: 8, 1824: 8, 1832: 8
  }],
}

# FW fingerprint captured on a real Atto 3 — bus 0, DID 0xF195
# (SUPPLIER_SOFTWARE_VERSION), the DID BYD ECUs actually answer (0xF188 returns
# NRC). 24 ECUs responded; the six below are confidently identified,
# cross-referenced against an ELM327/Car Scanner OBD scan that named each
# module. The camera (MPC, 0x704) wasn't in the bus-0 DID dump but answered the
# OBD DTC scan; its F195 is taken from the Car Scanner "supplier software
# version" field (non-essential for matching, see values.py).
FW_VERSIONS = {
  CAR.BYD_ATTO_3: {
    (Ecu.fwdCamera, 0x704, None): [
      b'\x9c\xa8\x18\x05\x19\x00',  # MPC front ADAS camera, part SC2EM-3619100A (supplier 5450)
    ],
    (Ecu.eps, 0x783, None): [
      b'\x9c@\x17\x0c\x06\x00',  # steering, app-data fingerprint ASY-BY062544-S3-V00AA00_0001
    ],
    (Ecu.abs, 0x782, None): [
      b'\x9cV\x19\x04\n\x01',  # IPB integrated power brake, BOSCH (F197='IPB', HW TA-3568010-D5)
    ],
    (Ecu.fwdRadar, 0x7f2, None): [
      b'\x9c\xa4\x16\x02\x19\x01',  # FMRR front radar (F194='TPVp')
    ],
    (Ecu.engine, 0x7e0, None): [
      b'ug\x18\x04\x147',  # EV drive unit / VCU (F197='SCEV'), 2026-06 DID dump
      b'uq\x18\x0c\nA',  # same VCU, live query 2026-07-11 (updated during dealer/ADAS-recal visit?)
    ],
    (Ecu.unknown, 0x7f1, None): [
      b'\x9cE\x18\x01\x1a\x01',  # SRS airbag, part SC2EM-D / SEC30-P50 (supplier 8893)
    ],
  },
}
