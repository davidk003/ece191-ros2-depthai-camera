import sys
from typing import Optional

import depthai as dai
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("camera_publisher")

        self.declare_parameter("topic_name", "/oak/rgb/image_raw")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 400)
        self.declare_parameter("frame_id", "oak_rgb_camera_frame")
        self.declare_parameter("anti_banding_mode", "mains_60_hz")
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_depth", 10)

        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.anti_banding_mode = (
            self.get_parameter("anti_banding_mode").get_parameter_value().string_value.lower()
        )
        self.qos_reliability = (
            self.get_parameter("qos_reliability").get_parameter_value().string_value.lower()
        )
        self.qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value

        self._validate_parameters()

        qos_profile = self._make_qos_profile(self.qos_reliability, self.qos_depth)
        self.publisher = self.create_publisher(Image, self.topic_name, qos_profile)

        self.pipeline = self._build_pipeline(
            width=self.width,
            height=self.height,
            fps=self.fps,
            anti_banding_mode_name=self.anti_banding_mode,
        )
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

        self.timer = self.create_timer(1.0 / self.fps, self.publish_rgb_frame)
        self.get_logger().info(
            f"Publishing RGB frames on {self.topic_name} as sensor_msgs/msg/Image "
            f"at {self.width}x{self.height} @ {self.fps:.1f} FPS "
            f"(qos_reliability={self.qos_reliability}, qos_depth={self.qos_depth}, "
            f"anti_banding_mode={self.anti_banding_mode})"
        )

    def _validate_parameters(self) -> None:
        if not self.topic_name:
            raise ValueError("topic_name must not be empty")
        if self.fps <= 0:
            raise ValueError("fps must be > 0")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be > 0")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if self.qos_depth <= 0:
            raise ValueError("qos_depth must be > 0")

    def _make_qos_profile(self, reliability_name: str, depth: int) -> QoSProfile:
        reliability_map = {
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }
        if reliability_name not in reliability_map:
            raise ValueError("qos_reliability must be one of: reliable, best_effort")

        return QoSProfile(
            reliability=reliability_map[reliability_name],
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(depth),
        )

    def _build_pipeline(
        self, width: int, height: int, fps: float, anti_banding_mode_name: str
    ) -> dai.Pipeline:
        anti_banding_map = {
            "off": dai.CameraControl.AntiBandingMode.OFF,
            "auto": dai.CameraControl.AntiBandingMode.AUTO,
            "mains_50_hz": dai.CameraControl.AntiBandingMode.MAINS_50_HZ,
            "mains_60_hz": dai.CameraControl.AntiBandingMode.MAINS_60_HZ,
        }
        if anti_banding_mode_name not in anti_banding_map:
            raise ValueError(
                "anti_banding_mode must be one of: off, auto, mains_50_hz, mains_60_hz"
            )

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName("rgb")
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setPreviewSize(int(width), int(height))
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(float(fps))
        cam_rgb.initialControl.setAntiBandingMode(anti_banding_map[anti_banding_mode_name])
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def publish_rgb_frame(self) -> None:
        packet = self.rgb_queue.tryGet()
        if packet is None:
            return

        frame = packet.getCvFrame()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        if hasattr(self, "device") and self.device is not None:
            self.device.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init(args=sys.argv)
    node: Optional[CameraPublisher] = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
