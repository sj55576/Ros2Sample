"""Tests for A* path planning logic."""
from nav2_learning.map_utils import (
    a_star_search,
    create_empty_grid,
    draw_filled_rectangle,
    get_cell,
)


def test_straight_line_path():
    """障害物のない空グリッドで A* が直接経路を見つけることを確認."""
    grid = create_empty_grid(20, 20)
    path = a_star_search(
        grid, 20, 20, (1, 1), (18, 18), cost_threshold=50, allow_diagonal=True
    )
    assert path is not None
    assert len(path) > 0
    assert path[0] == (1, 1)
    assert path[-1] == (18, 18)


def test_path_around_obstacle():
    """A* が障害物を回避して経路を見つけることを確認."""
    grid = create_empty_grid(20, 20)
    # (10, 5) から (10, 15) への縦方向の壁
    draw_filled_rectangle(grid, 20, 20, 10, 5, 10, 15, value=100)
    path = a_star_search(
        grid, 20, 20, (5, 10), (15, 10), cost_threshold=50, allow_diagonal=True
    )
    assert path is not None
    # 経路は壁を通過しないこと
    for gx, gy in path:
        assert get_cell(grid, gx, gy, 20) < 50


def test_no_path_blocked():
    """ゴールへの経路が完全に塞がれている場合に A* が None を返すことを確認."""
    grid = create_empty_grid(10, 10)
    # 経路を完全に塞ぐ縦壁
    draw_filled_rectangle(grid, 10, 10, 5, 0, 5, 9, value=100)
    path = a_star_search(
        grid, 10, 10, (2, 5), (8, 5), cost_threshold=50, allow_diagonal=False
    )
    assert path is None


def test_four_connected():
    """4方向接続での探索が縦横移動のみを使用することを確認."""
    grid = create_empty_grid(10, 10)
    path = a_star_search(
        grid, 10, 10, (0, 0), (5, 5), cost_threshold=50, allow_diagonal=False
    )
    assert path is not None
    for i in range(1, len(path)):
        dx = abs(path[i][0] - path[i - 1][0])
        dy = abs(path[i][1] - path[i - 1][1])
        assert dx + dy == 1  # 縦横移動のみ
