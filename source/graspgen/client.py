"""Small, Isaac-Sim-safe client for the existing GraspGen ZMQ server.

This module deliberately implements the wire protocol locally instead of importing
``/home/frlab/GraspGen``.  Isaac Sim therefore only needs the lightweight serving
dependencies and never loads GraspGen's PyTorch/CUDA stack in-process.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    import msgpack
    import msgpack_numpy
    import zmq
except ImportError as exc:  # pragma: no cover - exercised by the Isaac runtime setup
    raise ImportError(
        "GraspGen client dependencies are missing. Run "
        "'./scripts/install_graspgen_client_deps.py' from isaac_graspgen."
    ) from exc


msgpack_numpy.patch()


@dataclass(frozen=True)
class GraspGenResult:
    """One GraspGen inference response in the input point-cloud frame."""

    grasps: np.ndarray
    confidences: np.ndarray
    infer_ms: float | None


class GraspGenClient:
    """REQ client matching ``GraspGen/grasp_gen/serving/zmq_server.py``."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5556, timeout_ms: int = 60_000) -> None:
        self._address = f"tcp://{host}:{int(port)}"
        self._timeout_ms = int(timeout_ms)
        self._context = zmq.Context()
        self._socket: zmq.Socket | None = None

    def _new_socket(self) -> zmq.Socket:
        socket = self._context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(self._address)
        return socket

    def _reset_socket(self) -> None:
        if self._socket is not None:
            self._socket.close()
        self._socket = None

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._socket is None:
            self._socket = self._new_socket()
        try:
            self._socket.send(msgpack.packb(payload, use_bin_type=True))
            raw = self._socket.recv()
        except (zmq.Again, zmq.ZMQError) as exc:
            # A REQ socket cannot be reused after a timed-out request.
            self._reset_socket()
            raise ConnectionError(f"GraspGen server request failed at {self._address}: {exc}") from exc

        response = msgpack.unpackb(raw, raw=False)
        if "error" in response:
            raise RuntimeError(f"GraspGen server error: {response['error']}")
        return response

    def health_check(self) -> bool:
        try:
            return self._request({"action": "health"}).get("status") == "ok"
        except (ConnectionError, RuntimeError):
            return False

    def metadata(self) -> dict[str, Any]:
        return self._request({"action": "metadata"})

    def infer(
        self,
        point_cloud: np.ndarray,
        *,
        grasp_threshold: float = -1.0,
        num_grasps: int = 200,
        topk_num_grasps: int = 20,
        min_grasps: int = 1,
        max_tries: int = 1,
        remove_outliers: bool = True,
    ) -> GraspGenResult:
        points = np.ascontiguousarray(point_cloud, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"point_cloud must have shape (N, 3), got {points.shape}")
        if len(points) == 0 or not np.isfinite(points).all():
            raise ValueError("point_cloud must contain finite points")

        response = self._request(
            {
                "action": "infer",
                "point_cloud": points,
                "grasp_threshold": float(grasp_threshold),
                "num_grasps": int(num_grasps),
                "topk_num_grasps": int(topk_num_grasps),
                "min_grasps": int(min_grasps),
                "max_tries": int(max_tries),
                "remove_outliers": bool(remove_outliers),
            }
        )
        grasps = np.asarray(response["grasps"], dtype=np.float32).reshape(-1, 4, 4)
        confidences = np.asarray(response["confidences"], dtype=np.float32).reshape(-1)
        if len(grasps) != len(confidences):
            raise RuntimeError(
                f"GraspGen returned {len(grasps)} poses but {len(confidences)} confidences"
            )
        infer_ms = response.get("timing", {}).get("infer_ms")
        return GraspGenResult(
            grasps=grasps,
            confidences=confidences,
            infer_ms=None if infer_ms is None else float(infer_ms),
        )

    def close(self) -> None:
        self._reset_socket()
        self._context.term()

    def __enter__(self) -> "GraspGenClient":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
