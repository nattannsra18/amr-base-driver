from amr_base_driver.serial_bridge import crc16_ccitt
from amr_base_driver.serial_bridge import TELEMETRY_LENGTH
from amr_base_driver.serial_bridge import wrapped_int32_delta


def test_telemetry_wire_size():
    assert TELEMETRY_LENGTH == 44


def test_crc16_ccitt_false_reference_vector():
    assert crc16_ccitt(b'123456789') == 0x29B1


def test_wrapped_int32_delta_crosses_positive_boundary():
    assert wrapped_int32_delta(-0x80000000, 0x7FFFFFFF) == 1


def test_wrapped_int32_delta_crosses_negative_boundary():
    assert wrapped_int32_delta(0x7FFFFFFF, -0x80000000) == -1
