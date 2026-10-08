"""Bounded freshness allowing ROS clock/peer subscription delivery order races."""
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy

# Periodic complete views are rebroadcast at 5 Hz. Dropped state samples do not
# need retransmission; waiting for old reliable samples can block a resumed peer.
PEER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def fresh_peer(stamp, now, timeout):
    # Nodes consume /clock independently. A peer sample can arrive before the
    # matching clock tick; reject substantial future stamps and stale samples.
    return (isinstance(stamp, (int, float)) and math.isfinite(stamp) and
            -.25 <= now-stamp <= timeout)
