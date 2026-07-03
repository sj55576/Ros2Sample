"""Tests for mapping_utils module."""
import math

from nav2_learning.mapping_utils import (
    bresenham_line,
    clamp,
    integrate_scan,
    log_odds_to_occupancy,
    log_odds_to_prob,
    prob_to_log_odds,
)


def test_bresenham_line_horizontal():
    """bresenham_line が水平線のセルを正しく列挙することを確認。"""
    cells = bresenham_line(0, 0, 4, 0)
    assert cells == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]


def test_bresenham_line_vertical():
    """bresenham_line が垂直線のセルを正しく列挙することを確認。"""
    cells = bresenham_line(0, 0, 0, 4)
    assert cells == [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)]


def test_bresenham_line_diagonal():
    """bresenham_line が45度の対角線のセルを正しく列挙することを確認。"""
    cells = bresenham_line(0, 0, 3, 3)
    assert cells == [(0, 0), (1, 1), (2, 2), (3, 3)]


def test_bresenham_line_steep():
    """bresenham_line が急勾配の線でも両端を含めて正しく列挙することを確認。"""
    cells = bresenham_line(0, 0, 1, 5)
    assert cells[0] == (0, 0)
    assert cells[-1] == (1, 5)
    assert len(cells) >= 6


def test_bresenham_line_reversed_endpoints():
    """bresenham_line が始点と終点の順序を維持することを確認。"""
    cells = bresenham_line(4, 4, 0, 0)
    assert cells[0] == (4, 4)
    assert cells[-1] == (0, 0)


def test_bresenham_line_single_point():
    """bresenham_line が始点と終点が同じ場合に1セルのみ返すことを確認。"""
    cells = bresenham_line(2, 2, 2, 2)
    assert cells == [(2, 2)]


def test_bresenham_line_step_continuity():
    """bresenham_line の連続するセルが各軸で最大1しか変化しないことを確認。"""
    cells = bresenham_line(-3, 7, 5, -2)
    assert cells[0] == (-3, 7)
    assert cells[-1] == (5, -2)
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        assert abs(x1 - x0) <= 1
        assert abs(y1 - y0) <= 1


def test_prob_to_log_odds_round_trip():
    """prob_to_log_odds と log_odds_to_prob が往復変換で元の値に戻ることを確認。"""
    for probability in (0.1, 0.35, 0.5, 0.65, 0.9):
        log_odds = prob_to_log_odds(probability)
        assert abs(log_odds_to_prob(log_odds) - probability) < 1e-9


def test_log_odds_to_prob_zero():
    """log_odds_to_prob(0.0) が 0.5 を返すことを確認。"""
    assert log_odds_to_prob(0.0) == 0.5


def test_log_odds_to_prob_monotonic():
    """log_odds_to_prob が単調増加であることを確認。"""
    values = [-5.0, -1.0, 0.0, 1.0, 5.0]
    probabilities = [log_odds_to_prob(v) for v in values]
    assert probabilities == sorted(probabilities)


def test_log_odds_to_prob_numerical_stability():
    """log_odds_to_prob が極端な値でもオーバーフローせず [0, 1] に収まることを確認。"""
    high = log_odds_to_prob(50.0)
    low = log_odds_to_prob(-50.0)
    assert 0.0 <= high <= 1.0
    assert 0.0 <= low <= 1.0
    assert high > 0.999999
    assert low < 0.000001


def test_clamp():
    """clamp が値を範囲内に収めることを確認。"""
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-1.0, 0.0, 10.0) == 0.0
    assert clamp(11.0, 0.0, 10.0) == 10.0


GRID_WIDTH = 21
GRID_HEIGHT = 21
ORIGIN_X = -10.5
ORIGIN_Y = -10.5
RESOLUTION = 1.0


def _make_grid():
    """Return a fresh flat log-odds grid initialized to zero."""
    return [0.0] * (GRID_WIDTH * GRID_HEIGHT)


def _index(gx, gy):
    """Return the flat index for grid coordinates (gx, gy)."""
    return gy * GRID_WIDTH + gx


def test_integrate_scan_single_hit_ray():
    """integrate_scan が単一のヒットレイで終点セルを占有、途中セルを空きにすることを確認。"""
    log_odds = _make_grid()
    integrate_scan(
        log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
        sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
        ranges=[5.0], angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=10.0,
    )
    sensor_gx = int((0.0 - ORIGIN_X) / RESOLUTION)
    sensor_gy = int((0.0 - ORIGIN_Y) / RESOLUTION)
    end_gx = int((5.0 - ORIGIN_X) / RESOLUTION)
    end_gy = sensor_gy

    assert log_odds[_index(end_gx, end_gy)] > 0
    # Intermediate cell between sensor and endpoint should be marked free.
    mid_gx = sensor_gx + 2
    assert log_odds[_index(mid_gx, sensor_gy)] < 0
    # Cells behind the sensor (opposite direction) must be untouched.
    behind_gx = sensor_gx - 1
    assert log_odds[_index(behind_gx, sensor_gy)] == 0.0


def test_integrate_scan_out_of_range_ray():
    """integrate_scan が range=inf のレイで range_max までの空き空間を刻むことを確認。"""
    log_odds = _make_grid()
    integrate_scan(
        log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
        sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
        ranges=[math.inf], angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=10.0,
    )
    assert not any(value > 0 for value in log_odds)
    assert any(value < 0 for value in log_odds)


def test_integrate_scan_range_below_min():
    """integrate_scan が range_min 未満のレイを無視することを確認。"""
    log_odds = _make_grid()
    integrate_scan(
        log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
        sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
        ranges=[0.05], angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=10.0,
    )
    assert all(value == 0.0 for value in log_odds)


def test_integrate_scan_nan_range():
    """integrate_scan が NaN のレイを無視することを確認。"""
    log_odds = _make_grid()
    integrate_scan(
        log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
        sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
        ranges=[math.nan], angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=10.0,
    )
    assert all(value == 0.0 for value in log_odds)


def test_integrate_scan_repeated_hit_clamps():
    """integrate_scan を繰り返し適用しても終点セルが log_odds_max を超えないことを確認。"""
    log_odds = _make_grid()
    for _ in range(50):
        integrate_scan(
            log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
            sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
            ranges=[5.0], angle_min=0.0, angle_increment=0.0,
            range_min=0.1, range_max=10.0, log_odds_max=4.0,
        )
    end_gx = int((5.0 - ORIGIN_X) / RESOLUTION)
    end_gy = int((0.0 - ORIGIN_Y) / RESOLUTION)
    assert log_odds[_index(end_gx, end_gy)] == 4.0


def test_integrate_scan_endpoint_beyond_grid():
    """integrate_scan が終点がグリッド外でも例外を出さず範囲内セルを更新することを確認。"""
    log_odds = _make_grid()
    integrate_scan(
        log_odds, GRID_WIDTH, GRID_HEIGHT, ORIGIN_X, ORIGIN_Y, RESOLUTION,
        sensor_x=0.0, sensor_y=0.0, sensor_yaw=0.0,
        ranges=[100.0], angle_min=0.0, angle_increment=0.0,
        range_min=0.1, range_max=200.0,
    )
    sensor_gx = int((0.0 - ORIGIN_X) / RESOLUTION)
    sensor_gy = int((0.0 - ORIGIN_Y) / RESOLUTION)
    # In-bounds cells near the sensor should still have been carved as free.
    assert log_odds[_index(sensor_gx + 2, sensor_gy)] < 0
    assert log_odds[_index(GRID_WIDTH - 1, sensor_gy)] < 0


def test_log_odds_to_occupancy_unknown():
    """log_odds_to_occupancy が 0.0 を未知 (-1) に変換することを確認。"""
    assert log_odds_to_occupancy([0.0]) == [-1]


def test_log_odds_to_occupancy_occupied():
    """log_odds_to_occupancy が強い正の値を占有 (100) に変換することを確認。"""
    assert log_odds_to_occupancy([4.0]) == [100]


def test_log_odds_to_occupancy_free():
    """log_odds_to_occupancy が強い負の値を空き (0) に変換することを確認。"""
    assert log_odds_to_occupancy([-4.0]) == [0]


def test_log_odds_to_occupancy_custom_thresholds():
    """log_odds_to_occupancy がカスタム閾値を尊重することを確認。"""
    log_odds = prob_to_log_odds(0.6)
    assert log_odds_to_occupancy([log_odds], occupied_threshold=0.65, free_threshold=0.35) == [-1]
    assert log_odds_to_occupancy([log_odds], occupied_threshold=0.55, free_threshold=0.35) == [100]
