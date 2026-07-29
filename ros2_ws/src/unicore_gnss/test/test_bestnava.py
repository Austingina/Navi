"""Unit tests for BESTNAVA parser (manual sample from Unicore N4 §7.3.26)."""

from unicore_gnss.bestnava import extract_ascii_messages, parse_bestnava
from unicore_gnss.crc32 import crc32, verify_ascii_crc

SAMPLE = (
    "#BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;"
    "SOL_COMPUTED,SINGLE,40.07895888272,116.23651029820,65.8312,-8.4925,WGS84,"
    '1.2221,1.1053,2.1970,"0",0.000,0.000,50,28,28,0,1,12,12,41,'
    "SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0046,335.592288,0.0045,0.0194,0.0123"
    "*c1b4f7fe"
)

# Receiver may wrap the same sentence across lines (as printed in the manual).
WRAPPED = (
    "#BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;SOL_COMPUTED,SINGLE,40.0\r\n"
    "7895888272,116.23651029820,65.8312,-8.4925,WGS84,1.2221,1.1053,2.1970,\"0\",0.000,\r\n"
    "0.000,50,28,28,0,1,12,12,41,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0046,\r\n"
    "335.592288,0.0045,0.0194,0.0123*c1b4f7fe\r\n"
)


def test_crc_matches_manual_sample():
    body = SAMPLE[1:SAMPLE.rfind("*")].encode("ascii")
    assert crc32(body) == 0xC1B4F7FE
    assert verify_ascii_crc(SAMPLE)


def test_parse_bestnava_fields():
    nav = parse_bestnava(SAMPLE)
    assert nav is not None
    assert nav.sol_status == "SOL_COMPUTED"
    assert nav.pos_type == "SINGLE"
    assert abs(nav.lat_deg - 40.07895888272) < 1e-10
    assert abs(nav.lon_deg - 116.23651029820) < 1e-10
    assert abs(nav.height_msl_m - 65.8312) < 1e-4
    assert abs(nav.undulation_m - (-8.4925)) < 1e-4
    assert abs(nav.height_ellipsoid_m - (65.8312 - 8.4925)) < 1e-4
    assert nav.num_svs == 50
    assert nav.num_soln_svs == 28
    assert nav.vel_type == "DOPPLER_VELOCITY"
    assert abs(nav.hor_spd_mps - 0.0046) < 1e-6
    assert abs(nav.track_ground_deg - 335.592288) < 1e-6
    assert nav.has_position
    assert nav.navsat_status_fix() == 0  # STATUS_FIX


def test_extract_wrapped_message():
    buf = bytearray(WRAPPED.encode("ascii"))
    messages, remaining = extract_ascii_messages(buf)
    assert len(messages) == 1
    assert remaining == bytearray()
    nav = parse_bestnava(messages[0])
    assert nav is not None
    assert abs(nav.lat_deg - 40.07895888272) < 1e-10
