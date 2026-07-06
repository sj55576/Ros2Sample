"""Tests for path_utils module."""
from nav2_learning.map_utils import a_star_search, create_empty_grid, draw_filled_rectangle
from nav2_learning.path_utils import (
    bresenham_line,
    has_line_of_sight,
    is_path_blocked,
    shortcut_path,
    smooth_path_moving_average,
)
import pytest  # noqa: F401


class TestBresenhamLine:
    """Tests for bresenham_line."""

    def test_includes_both_endpoints(self):
        """始点と終点が結果に含まれることを確認."""
        cells = bresenham_line(0, 0, 3, 3)
        assert cells[0] == (0, 0)
        assert cells[-1] == (3, 3)

    def test_horizontal_line(self):
        """水平線が全ての中間セルを含むことを確認."""
        cells = bresenham_line(0, 5, 4, 5)
        assert cells == [(0, 5), (1, 5), (2, 5), (3, 5), (4, 5)]

    def test_symmetry(self):
        """始点と終点を入れ替えても同じセル集合になる（対称性）ことを確認."""
        forward = bresenham_line(1, 1, 6, 4)
        backward = bresenham_line(6, 4, 1, 1)
        assert set(forward) == set(backward)
        assert forward[0] == backward[-1]
        assert forward[-1] == backward[0]

    def test_steep_slope(self):
        """急勾配（dy > dx）の線分でも全セルが連続して繋がることを確認."""
        cells = bresenham_line(0, 0, 1, 5)
        assert cells[0] == (0, 0)
        assert cells[-1] == (1, 5)
        for i in range(1, len(cells)):
            dx = abs(cells[i][0] - cells[i - 1][0])
            dy = abs(cells[i][1] - cells[i - 1][1])
            assert dx <= 1 and dy <= 1

    def test_single_point(self):
        """始点と終点が同じ場合は1セルのみ返すことを確認."""
        cells = bresenham_line(2, 2, 2, 2)
        assert cells == [(2, 2)]


class TestHasLineOfSight:
    """Tests for has_line_of_sight."""

    def test_empty_grid_is_visible(self):
        """障害物のない空グリッドでは見通しが通ることを確認."""
        grid = create_empty_grid(10, 10)
        assert has_line_of_sight(grid, 10, 10, (0, 0), (9, 9), cost_threshold=50)

    def test_wall_blocks_line_of_sight(self):
        """壁が線分上にある場合に見通しが通らないことを確認."""
        grid = create_empty_grid(10, 10)
        draw_filled_rectangle(grid, 10, 10, 5, 0, 5, 9, value=100)
        assert not has_line_of_sight(grid, 10, 10, (0, 5), (9, 5), cost_threshold=50)

    def test_out_of_bounds_blocks_line_of_sight(self):
        """線分がグリッド範囲外に出る場合は見通しなしと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        assert not has_line_of_sight(grid, 10, 10, (0, 0), (15, 15), cost_threshold=50)

    def test_unknown_cell_blocks_line_of_sight(self):
        """未知セル(-1)が線分上にある場合に見通しが通らないことを確認."""
        grid = create_empty_grid(10, 10, default_value=-1)
        assert not has_line_of_sight(grid, 10, 10, (0, 0), (9, 9), cost_threshold=50)

    def test_cost_at_threshold_blocks(self):
        """コストが閾値と同値の場合は通行不可（閾値未満のみ許可）と判定されることを確認."""
        grid = create_empty_grid(10, 10)
        grid[5 * 10 + 5] = 50
        assert not has_line_of_sight(grid, 10, 10, (0, 5), (9, 5), cost_threshold=50)


class TestShortcutPath:
    """Tests for shortcut_path."""

    def test_straight_open_path_collapses_to_two_points(self):
        """障害物のない直線経路はショートカットで始点・終点の2点に短縮されることを確認."""
        grid = create_empty_grid(20, 20)
        path = [(i, i) for i in range(10)]
        result = shortcut_path(grid, 20, 20, path, cost_threshold=50)
        assert result == [path[0], path[-1]]

    def test_keeps_start_and_end(self):
        """ショートカット後も始点・終点が必ず保持されることを確認."""
        grid = create_empty_grid(20, 20)
        draw_filled_rectangle(grid, 20, 20, 10, 0, 10, 15, value=100)
        path = [(i, 5) for i in range(9)] + [(9, i) for i in range(5, 20)] + \
            [(i, 19) for i in range(9, 15)]
        result = shortcut_path(grid, 20, 20, path, cost_threshold=50)
        assert result[0] == path[0]
        assert result[-1] == path[-1]

    def test_empty_path_returned_as_is(self):
        """空の経路はそのまま返されることを確認."""
        grid = create_empty_grid(10, 10)
        assert shortcut_path(grid, 10, 10, [], cost_threshold=50) == []

    def test_short_path_returned_as_is(self):
        """長さ2以下の経路はそのまま返されることを確認."""
        grid = create_empty_grid(10, 10)
        path = [(0, 0), (5, 5)]
        assert shortcut_path(grid, 10, 10, path, cost_threshold=50) == path

    def test_shortcut_does_not_cut_through_l_shaped_wall(self):
        """L字型の障害物を迂回する経路がショートカットで壁を突き抜けないことを確認."""
        grid = create_empty_grid(20, 20)
        # (10,0)-(10,10) の縦壁と (10,10)-(19,10) の横壁で L字（角を持つ壁）を形成
        draw_filled_rectangle(grid, 20, 20, 10, 0, 10, 10, value=100)
        draw_filled_rectangle(grid, 20, 20, 10, 10, 19, 10, value=100)

        # A* で壁を迂回する実際の経路を生成し、それをショートカットにかける
        start, goal = (2, 2), (17, 17)
        path = a_star_search(grid, 20, 20, start, goal, cost_threshold=50, allow_diagonal=True)
        assert path is not None

        result = shortcut_path(grid, 20, 20, path, cost_threshold=50)

        # ショートカット後の隣接ウェイポイント間は、常に見通しが通っていること
        # （壁の角を突き抜けるショートカットが行われていないこと）
        for i in range(1, len(result)):
            assert has_line_of_sight(grid, 20, 20, result[i - 1], result[i], cost_threshold=50)
        assert result[0] == start
        assert result[-1] == goal
        assert len(result) <= len(path)


class TestSmoothPathMovingAverage:
    """Tests for smooth_path_moving_average."""

    def test_endpoints_are_fixed(self):
        """始点・終点は平滑化後も変化しないことを確認."""
        points = [(0.0, 0.0), (1.0, 2.0), (2.0, 0.0), (3.0, 2.0), (4.0, 0.0)]
        smoothed = smooth_path_moving_average(points, window=3)
        assert smoothed[0] == points[0]
        assert smoothed[-1] == points[-1]

    def test_short_path_returned_as_is(self):
        """長さ3未満の点列はそのまま返されることを確認."""
        points = [(0.0, 0.0), (1.0, 1.0)]
        assert smooth_path_moving_average(points, window=5) == points

    def test_even_window_is_made_odd(self):
        """偶数のwindowが奇数化されても例外なく処理できることを確認."""
        points = [(float(i), 0.0) for i in range(10)]
        smoothed = smooth_path_moving_average(points, window=4)
        assert len(smoothed) == len(points)
        assert smoothed[0] == points[0]
        assert smoothed[-1] == points[-1]

    def test_flat_line_is_unchanged_at_full_window_interior(self):
        """完全なウィンドウが取れる内側の点では、直線状の点列の座標が変化しないことを確認."""
        points = [(float(i), 2.0) for i in range(10)]
        window = 5
        half_window = window // 2
        smoothed = smooth_path_moving_average(points, window=window)
        # 完全な対称ウィンドウが確保できるインデックス範囲のみ検証する
        # （端に近い点は縮小ウィンドウのため非対称な平均になり、値がずれ得る）
        for i in range(half_window, len(points) - half_window):
            ox, oy = points[i]
            x, y = smoothed[i]
            assert abs(x - ox) < 1e-9
            assert abs(y - oy) < 1e-9

    def test_smooths_a_zigzag(self):
        """ジグザグ経路が平滑化により振幅が縮小することを確認."""
        points = [(float(i), 1.0 if i % 2 == 0 else -1.0) for i in range(9)]
        smoothed = smooth_path_moving_average(points, window=5)
        interior_original = [abs(y) for _, y in points[1:-1]]
        interior_smoothed = [abs(y) for _, y in smoothed[1:-1]]
        assert max(interior_smoothed) < max(interior_original)


class TestIsPathBlocked:
    """Tests for is_path_blocked."""

    def test_clear_path_is_not_blocked(self):
        """障害物のない経路はブロックされていないと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        path_cells = [(i, i) for i in range(5)]
        assert not is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_new_obstacle_blocks_path(self):
        """経路上に新たな障害物が現れた場合にブロックと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        grid[3 * 10 + 3] = 100
        path_cells = [(i, i) for i in range(5)]
        assert is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_out_of_bounds_cell_blocks_path(self):
        """経路セルが範囲外の場合にブロックと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        path_cells = [(0, 0), (10, 10)]
        assert is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_unknown_cell_blocks_path(self):
        """経路セルが未知(-1)の場合にブロックと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        grid[2 * 10 + 2] = -1
        path_cells = [(2, 2)]
        assert is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_cost_at_threshold_blocks_path(self):
        """コストが閾値と同値の場合にブロックと判定される境界条件を確認."""
        grid = create_empty_grid(10, 10)
        grid[4 * 10 + 4] = 50
        path_cells = [(4, 4)]
        assert is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_cost_just_below_threshold_not_blocked(self):
        """コストが閾値未満の場合はブロックされないという境界条件を確認."""
        grid = create_empty_grid(10, 10)
        grid[4 * 10 + 4] = 49
        path_cells = [(4, 4)]
        assert not is_path_blocked(grid, 10, 10, path_cells, cost_threshold=50)

    def test_empty_path_is_not_blocked(self):
        """空の経路はブロックされていないと判定されることを確認."""
        grid = create_empty_grid(10, 10)
        assert not is_path_blocked(grid, 10, 10, [], cost_threshold=50)
