"""Educational node for observing and understanding costmap data."""

from typing import Optional

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class CostmapMonitor(Node):
    """Subscribe to /map and periodically log analysis for learning about costmaps."""

    def __init__(self) -> None:
        super().__init__('costmap_monitor')
        self.declare_parameter('analyze_rate', 0.5)
        self.declare_parameter('cost_threshold', 50)

        self._map: Optional[OccupancyGrid] = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, latched_qos)

        rate = max(0.1, float(self.get_parameter('analyze_rate').value))
        self.create_timer(1.0 / rate, self._analyze_tick)
        self.get_logger().info('コストマップモニター起動: マップを待っています...')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Store the received map."""
        self._map = msg

    def _analyze_tick(self) -> None:
        """Analyze the stored map and log a breakdown of cell types."""
        if self._map is None:
            return

        info = self._map.info
        width = info.width
        height = info.height
        resolution = info.resolution
        threshold = int(self.get_parameter('cost_threshold').value)

        total = width * height
        free = 0
        occupied = 0
        unknown = 0

        for cell in self._map.data:
            if cell == -1:
                unknown += 1
            elif cell >= threshold:
                occupied += 1
            else:
                free += 1

        free_pct = 100.0 * free / total if total > 0 else 0.0
        occ_pct = 100.0 * occupied / total if total > 0 else 0.0
        unk_pct = 100.0 * unknown / total if total > 0 else 0.0

        w_m = width * resolution
        h_m = height * resolution

        self.get_logger().info(
            f'マップ解析: {width}x{height}セル ({w_m:.1f}x{h_m:.1f}m), '
            f'解像度: {resolution}m/cell'
        )
        self.get_logger().info(
            f'空きセル: {free} ({free_pct:.1f}%), '
            f'障害物: {occupied} ({occ_pct:.1f}%), '
            f'未知: {unknown} ({unk_pct:.1f}%)'
        )


def main(args=None) -> None:
    """Initialise rclpy, spin the CostmapMonitor node, and shut down cleanly."""
    rclpy.init(args=args)
    node = CostmapMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
