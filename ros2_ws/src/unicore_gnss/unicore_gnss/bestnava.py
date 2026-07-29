"""Parse Unicore BESTNAVA ASCII logs (N4 command manual §7.3.26)."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Optional

from unicore_gnss.crc32 import verify_ascii_crc

# Complete #...*xxxxxxxx message; whitespace/newlines allowed inside (receiver may wrap).
_ASCII_MSG_RE = re.compile(
    rb"#(?P<body>[^*\r\n]*(?:\r?\n[^*\r\n]*)*)\*(?P<crc>[0-9A-Fa-f]{8})",
    re.MULTILINE,
)

# RTK / differential-ish position types -> GBAS; SBAS/PSRDIFF -> SBAS; else FIX.
_GBAS_TYPES = frozenset({
    "L1_FLOAT", "IONOFREE_FLOAT", "NARROW_FLOAT",
    "L1_INT", "WIDE_INT", "NARROW_INT",
    "INS_RTKFLOAT", "INS_RTKFIXED",
    "PPP", "PPP_CONVERGING",
})
_SBAS_TYPES = frozenset({"SBAS", "PSRDIFF", "INS_PSRDIFF"})


@dataclass
class AsciiHeader:
    message: str
    cpu_idle: int
    time_ref: str
    time_status: str
    week: int
    ms: int
    version: int
    reserved: int
    leap_sec: int
    output_delay_ms: int


@dataclass
class BestNavA:
    """Decoded BESTNAVA fields (WGS84 lat/lon, MSL height + undulation)."""

    header: AsciiHeader
    sol_status: str
    pos_type: str
    lat_deg: float
    lon_deg: float
    height_msl_m: float
    undulation_m: float
    datum: str
    lat_std_m: float
    lon_std_m: float
    hgt_std_m: float
    station_id: str
    diff_age_s: float
    sol_age_s: float
    num_svs: int
    num_soln_svs: int
    reserved1: int
    reserved2: int
    reserved3: int
    ext_sol_stat: int
    galileo_bds3_mask: int
    gps_glo_bds2_mask: int
    vel_sol_status: str
    vel_type: str
    latency_s: float
    vel_diff_age_s: float
    hor_spd_mps: float
    track_ground_deg: float
    vert_spd_mps: float
    vert_spd_std_mps: float
    hor_spd_std_mps: float
    raw: str
    crc_ok: bool

    @property
    def height_ellipsoid_m(self) -> float:
        """WGS84 ellipsoidal height = MSL height + undulation."""
        return self.height_msl_m + self.undulation_m

    @property
    def has_position(self) -> bool:
        return self.sol_status == "SOL_COMPUTED" and self.pos_type not in ("NONE", "")

    @property
    def has_velocity(self) -> bool:
        return self.vel_sol_status == "SOL_COMPUTED" and self.vel_type not in ("NONE", "")

    def east_north_up_velocity(self) -> tuple[float, float, float]:
        """ENU ground velocity from horizontal speed + track + vertical speed."""
        if not self.has_velocity:
            return (float("nan"), float("nan"), float("nan"))
        heading_rad = math.radians(self.track_ground_deg)
        east = self.hor_spd_mps * math.sin(heading_rad)
        north = self.hor_spd_mps * math.cos(heading_rad)
        up = self.vert_spd_mps
        return (east, north, up)

    def navsat_status_fix(self) -> int:
        """Map to sensor_msgs/NavSatStatus.status (NO_FIX / FIX / SBAS / GBAS)."""
        if not self.has_position:
            return -1  # STATUS_NO_FIX
        if self.pos_type in _GBAS_TYPES:
            return 2  # STATUS_GBAS_FIX
        if self.pos_type in _SBAS_TYPES:
            return 1  # STATUS_SBAS_FIX
        return 0  # STATUS_FIX


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
        return value[1:-1]
    return value


def _to_int(value: str, default: int = 0) -> int:
    value = _strip_quotes(value)
    if not value:
        return default
    try:
        return int(value, 0)
    except ValueError:
        return default


def _to_float(value: str, default: float = float("nan")) -> float:
    value = _strip_quotes(value)
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_bestnava(message: str, check_crc: bool = True) -> Optional[BestNavA]:
    """Parse one complete BESTNAVA sentence (may contain embedded newlines)."""
    message = message.strip()
    if not message.startswith("#"):
        return None

    star = message.rfind("*")
    if star < 0:
        return None
    body = message[1:star]
    crc_ok = verify_ascii_crc(message) if check_crc else True
    if check_crc and not crc_ok:
        return None

    # Header and payload separated by ';'
    if ";" not in body:
        return None
    header_part, payload_part = body.split(";", 1)
    hfields = header_part.split(",")
    # BESTNAVA,cpu,time_ref,time_status,week,ms,version,reserved,leap,delay
    if len(hfields) < 10 or not hfields[0].upper().startswith("BESTNAVA"):
        return None

    header = AsciiHeader(
        message=hfields[0],
        cpu_idle=_to_int(hfields[1]),
        time_ref=hfields[2],
        time_status=hfields[3],
        week=_to_int(hfields[4]),
        ms=_to_int(hfields[5]),
        version=_to_int(hfields[6]),
        reserved=_to_int(hfields[7]),
        leap_sec=_to_int(hfields[8]),
        output_delay_ms=_to_int(hfields[9]),
    )

    # Collapse newlines/spaces that receivers insert when wrapping long lines.
    payload_part = re.sub(r"[\r\n]+", "", payload_part)
    fields = payload_part.split(",")
    # BESTNAV body has 30 fields (IDs 1..30 before CRC).
    if len(fields) < 30:
        return None

    return BestNavA(
        header=header,
        sol_status=fields[0],
        pos_type=fields[1],
        lat_deg=_to_float(fields[2]),
        lon_deg=_to_float(fields[3]),
        height_msl_m=_to_float(fields[4]),
        undulation_m=_to_float(fields[5]),
        datum=fields[6],
        lat_std_m=_to_float(fields[7]),
        lon_std_m=_to_float(fields[8]),
        hgt_std_m=_to_float(fields[9]),
        station_id=_strip_quotes(fields[10]),
        diff_age_s=_to_float(fields[11], 0.0),
        sol_age_s=_to_float(fields[12], 0.0),
        num_svs=_to_int(fields[13]),
        num_soln_svs=_to_int(fields[14]),
        reserved1=_to_int(fields[15]),
        reserved2=_to_int(fields[16]),
        reserved3=_to_int(fields[17]),
        ext_sol_stat=_to_int(fields[18]),
        galileo_bds3_mask=_to_int(fields[19]),
        gps_glo_bds2_mask=_to_int(fields[20]),
        vel_sol_status=fields[21],
        vel_type=fields[22],
        latency_s=_to_float(fields[23], 0.0),
        vel_diff_age_s=_to_float(fields[24], 0.0),
        hor_spd_mps=_to_float(fields[25], 0.0),
        track_ground_deg=_to_float(fields[26], 0.0),
        vert_spd_mps=_to_float(fields[27], 0.0),
        vert_spd_std_mps=_to_float(fields[28], 0.0),
        hor_spd_std_mps=_to_float(fields[29], 0.0),
        raw=message,
        crc_ok=crc_ok,
    )


def extract_ascii_messages(buffer: bytearray) -> tuple[list[str], bytearray]:
    """Pull complete ``#...*CRC`` messages out of a serial byte buffer.

    Returns (messages, remaining_buffer). Non-Unicore noise before the first
    ``#`` is discarded. Incomplete trailing fragment is kept.
    """
    messages: list[str] = []
    while True:
        match = _ASCII_MSG_RE.search(buffer)
        if not match:
            # Drop junk before a possible partial '#...'
            hash_idx = buffer.find(b"#")
            if hash_idx > 0:
                del buffer[:hash_idx]
            elif hash_idx < 0 and len(buffer) > 4096:
                buffer.clear()
            break
        raw = match.group(0)
        try:
            messages.append(raw.decode("ascii"))
        except UnicodeDecodeError:
            pass
        end = match.end()
        # Consume trailing CR/LF after *CRC (standard Unicore terminator).
        while end < len(buffer) and buffer[end] in (0x0D, 0x0A):
            end += 1
        del buffer[:end]
    return messages, buffer
