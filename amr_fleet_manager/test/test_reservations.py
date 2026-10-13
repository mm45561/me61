"""Unit tests for ReservationTable."""

from amr_fleet_manager.reservations import ReservationTable


def test_one_holder_per_resource():
    table = ReservationTable()
    assert table.try_acquire('dock:gtp_1', 'amr_1')
    assert table.try_acquire('dock:gtp_1', 'amr_1')       # re-acquiring is fine
    assert not table.try_acquire('dock:gtp_1', 'amr_2')
    assert table.holder('dock:gtp_1') == 'amr_1'


def test_only_the_holder_can_release():
    table = ReservationTable()
    table.try_acquire('slot:q1', 'amr_1')
    table.release('slot:q1', 'amr_2')
    assert table.holder('slot:q1') == 'amr_1'
    table.release('slot:q1', 'amr_1')
    assert table.holder('slot:q1') is None


def test_held_by():
    table = ReservationTable()
    table.try_acquire('slot:q1', 'amr_1')
    table.try_acquire('dock:gtp_1', 'amr_1')
    table.try_acquire('slot:q2', 'amr_2')
    assert table.held_by('amr_1') == ['dock:gtp_1', 'slot:q1']
    assert table.held_by('amr_3') == []
