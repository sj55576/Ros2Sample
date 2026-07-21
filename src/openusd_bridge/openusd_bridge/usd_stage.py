"""Small OpenUSD stage writer used by the ROS node."""

import math
from pathlib import Path
from typing import Tuple


class OpenUsdUnavailableError(RuntimeError):
    """Raised when the optional OpenUSD Python bindings are unavailable."""


class RobotPoseStage:
    """Create a stage and author time-sampled robot transforms."""

    def __init__(
        self,
        output_path: str,
        robot_prim_path: str,
        time_codes_per_second: float,
    ) -> None:
        """Create a new stage with simple ground and robot geometry."""
        try:
            from pxr import Gf, Usd, UsdGeom
        except ImportError as exc:
            raise OpenUsdUnavailableError(
                'OpenUSD Python bindings were not found. Install the '
                '`usd-core` package into the Python environment used by ROS 2.'
            ) from exc

        self._gf = Gf
        self._usd = Usd
        self._usd_geom = UsdGeom

        path = Path(output_path).expanduser().resolve()
        if path.suffix.lower() not in {'.usd', '.usda', '.usdc'}:
            raise ValueError('output_path must end in .usd, .usda, or .usdc')
        if not robot_prim_path.startswith('/'):
            raise ValueError(
                'robot_prim_path must be an absolute USD prim path',
            )
        if (
            time_codes_per_second <= 0.0
            or not math.isfinite(time_codes_per_second)
        ):
            raise ValueError(
                'time_codes_per_second must be finite and positive',
            )
        path.parent.mkdir(parents=True, exist_ok=True)

        self.output_path = str(path)
        self.stage = Usd.Stage.CreateNew(self.output_path)
        if self.stage is None:
            raise RuntimeError(
                f'failed to create OpenUSD stage: {self.output_path}',
            )
        self.stage.SetTimeCodesPerSecond(time_codes_per_second)
        self.stage.SetFramesPerSecond(time_codes_per_second)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)

        world = UsdGeom.Xform.Define(self.stage, '/World')
        self.stage.SetDefaultPrim(world.GetPrim())
        self._define_ground()
        robot = UsdGeom.Xform.Define(self.stage, robot_prim_path)
        self._translate_op = robot.AddTranslateOp(
            precision=UsdGeom.XformOp.PrecisionDouble,
        )
        self._orient_op = robot.AddOrientOp(
            precision=UsdGeom.XformOp.PrecisionDouble,
        )
        self._define_robot_body(f'{robot_prim_path}/Body')
        self._start_time_code = None
        self._end_time_code = None

    def _define_ground(self) -> None:
        """Add a thin cube as a simple reference ground plane."""
        ground = self._usd_geom.Cube.Define(self.stage, '/World/Ground')
        ground.CreateSizeAttr(1.0)
        ground.AddScaleOp().Set(self._gf.Vec3f(20.0, 20.0, 0.05))
        ground.AddTranslateOp().Set(self._gf.Vec3d(0.0, 0.0, -0.025))

    def _define_robot_body(self, prim_path: str) -> None:
        """Add a visible placeholder body below the animated robot Xform."""
        body = self._usd_geom.Cube.Define(self.stage, prim_path)
        body.CreateSizeAttr(1.0)
        body.AddScaleOp().Set(self._gf.Vec3f(0.6, 0.4, 0.2))
        body.AddTranslateOp().Set(self._gf.Vec3d(0.0, 0.0, 0.2))
        body.CreateDisplayColorAttr([self._gf.Vec3f(0.15, 0.45, 0.8)])

    def add_pose(
        self,
        time_code: float,
        position: Tuple[float, float, float],
        orientation_xyzw: Tuple[float, float, float, float],
    ) -> None:
        """Write position and orientation at one USD time code."""
        time = self._usd.TimeCode(time_code)
        self._translate_op.Set(self._gf.Vec3d(*position), time)
        x, y, z, w = orientation_xyzw
        self._orient_op.Set(
            self._gf.Quatd(w, self._gf.Vec3d(x, y, z)),
            time,
        )
        if self._start_time_code is None:
            self._start_time_code = time_code
            self.stage.SetStartTimeCode(time_code)
        self._end_time_code = time_code
        self.stage.SetEndTimeCode(time_code)

    def save(self) -> None:
        """Persist all authored data to the stage's root layer."""
        if not self.stage.GetRootLayer().Save():
            raise RuntimeError(
                f'failed to save OpenUSD stage: {self.output_path}',
            )
