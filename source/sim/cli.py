"""Command-line contract for the standalone grasp scene.

This module deliberately has no Isaac Sim imports. Invalid combinations can be
rejected before ``SimulationApp`` starts, and the CLI remains unit-testable from
the lightweight GraspGen environment.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from graspgen import config as graspgen_config
from robots import DEFAULT_ROBOT, available_robots
from sim.config import YCB_CONFIG


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        default=DEFAULT_ROBOT,
        choices=available_robots(),
        help=f"구동할 로봇 (source/robots 레지스트리, 기본 {DEFAULT_ROBOT})",
    )
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument(
        "--no-ros2",
        action="store_true",
        help="ROS2 bridge와 Action Graph를 켜지 않는다. grasp 파이프라인은 ROS2를 "
        "쓰지 않으며, 시작 크래시 진단에도 사용한다.",
    )
    parser.add_argument(
        "--ycb-dynamic",
        action="store_true",
        help="세틀 후에도 YCB를 dynamic으로 둔다. 종료 시 물체별 이동/회전량을 출력한다.",
    )
    parser.add_argument(
        "--ycb-only",
        default=None,
        help="지정한 이름의 YCB 물체 하나만 스폰한다(예: 005_tomato_soup_can).",
    )
    parser.add_argument(
        "--ycb-radius",
        type=float,
        default=None,
        help="YCB 스폰 반경(m)을 덮어쓴다. 기본 0.70은 Indy7 리치 기준이다.",
    )
    parser.add_argument(
        "--ycb-collision",
        choices=["convexHull", "convexDecomposition", "sdf", "boundingCube"],
        default=None,
        help="모든 YCB의 충돌 근사를 강제한다(단일 변수 A/B용).",
    )
    parser.add_argument("--max-steps", type=int, default=0, help="N 스텝 후 자동 종료 (0=무한)")
    parser.add_argument(
        "--wrist-viewport",
        action="store_true",
        help="별도 wrist-camera viewport를 연다(기본 off).",
    )
    parser.add_argument("--target-position", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--target-orientation",
        type=float,
        nargs=4,
        default=[0.0, 0.0, 1.0, 0.0],
        metavar=("W", "X", "Y", "Z"),
        help="목표 TCP orientation, Isaac wxyz quaternion",
    )
    parser.add_argument(
        "--gripper",
        choices=["open", "close", "hold"],
        default="open",
        help="그리퍼 초기/유지 명령(기본 open; hold은 USD 초기 자세 유지)",
    )
    parser.add_argument(
        "--graspgen",
        action="store_true",
        help="wrist point cloud를 GraspGen ZMQ 서버로 보내고 grasp pose를 표시",
    )
    parser.add_argument("--graspgen-host", default=graspgen_config.HOST)
    parser.add_argument("--graspgen-port", type=int, default=graspgen_config.PORT)
    parser.add_argument("--graspgen-timeout-ms", type=int, default=graspgen_config.TIMEOUT_MS)
    parser.add_argument(
        "--graspgen-step",
        type=int,
        default=graspgen_config.TRIGGER_STEP,
        help="GraspGen을 한 번 호출할 simulation step",
    )
    parser.add_argument("--grasp-object-index", type=int, default=0)
    parser.add_argument("--grasp-point-count", type=int, default=graspgen_config.POINT_COUNT)
    parser.add_argument("--grasp-num-grasps", type=int, default=graspgen_config.NUM_GRASPS)
    parser.add_argument("--grasp-topk", type=int, default=graspgen_config.TOPK)
    parser.add_argument("--grasp-seed", type=int, default=graspgen_config.SEED)
    parser.add_argument(
        "--grasp-oracle-centering",
        action="store_true",
        help="정렬도를 관측 점군 대신 simulator 물체 중심으로 계산하는 실행 진단용 옵션",
    )
    parser.add_argument(
        "--execute-grasp",
        action="store_true",
        help="선택한 grasp를 pregrasp→approach→close→lift로 실행 (--graspgen 필요)",
    )
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse scene options while leaving Kit's own ``--/...`` flags alone."""
    args, _kit_args = build_parser().parse_known_args(argv)
    if args.execute_grasp and not args.graspgen:
        raise ValueError("--execute-grasp requires --graspgen")

    names = [str(item["name"]) for item in YCB_CONFIG["objects"]]
    if args.ycb_only is not None and args.ycb_only not in names:
        raise ValueError(f"--ycb-only '{args.ycb_only}' not in {names}")
    if args.ycb_radius is not None and args.ycb_radius <= 0.0:
        raise ValueError("--ycb-radius must be > 0")
    if args.max_steps < 0:
        raise ValueError("--max-steps must be >= 0")
    for name in ("grasp_point_count", "grasp_num_grasps", "grasp_topk"):
        if getattr(args, name) < 1:
            raise ValueError(f"--{name.replace('_', '-')} must be >= 1")
    return args
