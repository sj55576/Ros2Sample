"""Tests for map_utils module."""
from nav2_learning.map_utils import (
    create_empty_grid,
    draw_filled_circle,
    draw_filled_rectangle,
    draw_walls,
    get_cell,
    grid_to_world,
    inflate_obstacles,
    is_valid_cell,
    world_to_grid,
)
import pytest  # noqa: F401


def test_create_empty_grid():
    """create_empty_grid が正しいサイズとデフォルト値のグリッドを返すことを確認."""
    grid = create_empty_grid(10, 10)
    assert len(grid) == 100
    assert all(v == 0 for v in grid)


def test_create_empty_grid_custom_value():
    """create_empty_grid がカスタムデフォルト値を正しく設定することを確認."""
    grid = create_empty_grid(5, 5, default_value=-1)
    assert all(v == -1 for v in grid)


def test_world_to_grid():
    """world_to_grid が正しいグリッド座標を返すことを確認."""
    gx, gy = world_to_grid(0.1, 0.2, 0.0, 0.0, 0.05)
    assert gx == 2
    assert gy == 4


def test_grid_to_world():
    """grid_to_world がセル中心のワールド座標を返すことを確認."""
    wx, wy = grid_to_world(3, 5, 0.0, 0.0, 0.05)
    assert abs(wx - 0.175) < 1e-6  # (3 + 0.5) * 0.05
    assert abs(wy - 0.275) < 1e-6


def test_world_grid_roundtrip():
    """world→grid→world の変換が解像度以内の精度で元の座標に戻ることを確認."""
    origin_x, origin_y, res = -2.5, -2.5, 0.05
    wx_in, wy_in = 1.23, -0.87
    gx, gy = world_to_grid(wx_in, wy_in, origin_x, origin_y, res)
    wx_out, wy_out = grid_to_world(gx, gy, origin_x, origin_y, res)
    assert abs(wx_out - wx_in) < res
    assert abs(wy_out - wy_in) < res


def test_draw_filled_rectangle():
    """draw_filled_rectangle が矩形領域を正しく塗りつぶすことを確認."""
    grid = create_empty_grid(10, 10)
    draw_filled_rectangle(grid, 10, 10, 2, 2, 4, 4, value=100)
    assert get_cell(grid, 3, 3, 10) == 100
    assert get_cell(grid, 0, 0, 10) == 0


def test_draw_filled_circle():
    """draw_filled_circle が円形領域を正しく塗りつぶすことを確認."""
    grid = create_empty_grid(20, 20)
    draw_filled_circle(grid, 20, 20, 10, 10, 3, value=100)
    assert get_cell(grid, 10, 10, 20) == 100
    assert get_cell(grid, 0, 0, 20) == 0


def test_draw_walls():
    """draw_walls がグリッドの外周を壁として設定することを確認."""
    grid = create_empty_grid(10, 10)
    draw_walls(grid, 10, 10)
    # 四隅と外周は占有済みであること
    assert get_cell(grid, 0, 0, 10) == 100
    assert get_cell(grid, 9, 9, 10) == 100
    # 内部は空きであること
    assert get_cell(grid, 5, 5, 10) == 0


def test_is_valid_cell():
    """is_valid_cell がグリッド境界を正しく検証することを確認."""
    assert is_valid_cell(0, 0, 10, 10)
    assert is_valid_cell(9, 9, 10, 10)
    assert not is_valid_cell(-1, 0, 10, 10)
    assert not is_valid_cell(10, 0, 10, 10)


def test_inflate_obstacles():
    """inflate_obstacles が障害物周辺にコストを正しく伝播させることを確認."""
    grid = create_empty_grid(10, 10)
    grid[5 * 10 + 5] = 100  # (5, 5) に障害物を1つ配置
    inflated = inflate_obstacles(grid, 10, 10, inflation_radius=2)
    # 障害物セルは 100 のままであること
    assert inflated[5 * 10 + 5] == 100
    # 隣接セルにはコスト > 0 が伝播していること
    assert inflated[5 * 10 + 4] > 0
    assert inflated[4 * 10 + 5] > 0
    # 遠いセルは 0 のままであること
    assert inflated[0 * 10 + 0] == 0
