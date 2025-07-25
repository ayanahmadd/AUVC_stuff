
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import gi
from cv_bridge import CvBridge
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class BlueROV2CameraInterface(Node):
    frame_id = 0

    def __init__(self):
        super().__init__("bluerov2_camera_interface")
        self.declare_parameter(
            "video_src",
            "udpsrc port=5600",
        )
        self.video_src = self.get_parameter("video_src").value

        self.declare_parameter(
            "video_codec",
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264",
        )
        self.video_codec = self.get_parameter("video_codec").value

        self.declare_parameter(
            "video_decode",
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
        )
        self.video_decode = self.get_parameter("video_decode").value

        self.declare_parameter(
            "video_sink",
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true",
        )
        self.video_sink = self.get_parameter("video_sink").value

        self.cvb = CvBridge()
        self.publisher = self.create_publisher(Image, "bluerov2/camera", 10)

        pipeline_str = " ".join(
            [self.video_src, self.video_codec, self.video_decode, self.video_sink]
        )
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.appsink = self.pipeline.get_by_name("appsink0")
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self.on_new_sample)

        self.get_logger().info(
            "BlueRov2CameraInterface node has started. Waiting for images..."
        )

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        self.get_logger().debug("Received new sample")
        buffer = sample.get_buffer()
        success, info = buffer.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().error("Could not map buffer")
            return

        caps_structure = sample.get_caps().get_structure(0)
        self.get_logger().debug(
            f"Received image with size: {caps_structure.get_value('width')} x {caps_structure.get_value('height')}"
        )

        np_image = np.ndarray(
            (caps_structure.get_value("height"), caps_structure.get_value("width"), 3),
            buffer=buffer.extract_dup(0, buffer.get_size()),
            dtype=np.uint8,
        )

        msg = self.cvb.cv2_to_imgmsg(np_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)
        self.publisher.publish(msg)

        self.get_logger().debug(f"Published image with frame_id: {self.frame_id}")

        self.frame_id += 1
        buffer.unmap(info)
        return Gst.FlowReturn.OK


def main(args=None):
    rclpy.init(args=args)

    node = BlueRov2CameraInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()