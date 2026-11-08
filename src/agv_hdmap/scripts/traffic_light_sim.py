#!/usr/bin/env python3
"""仿真红绿灯：按固定周期切换两组灯（ew / ns），用 Autoware 的消息发出去，再在灯的位置画当前颜色。

    /hdmap/traffic_light_states（autoware_perception_msgs/TrafficLightGroupArray）：group id = 地图里红绿灯
        regulatory element 的 id，和 Autoware 的感知输出同一个接口
    /hdmap/traffic_light_markers（MarkerArray）
周期参数 green / yellow / red（秒）可以运行时用 ros2 param set 改。
"""
import os
import sys

import rclpy
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hdmap  # noqa: E402

MSG_COLOR = {'green': TrafficLightElement.GREEN, 'yellow': TrafficLightElement.AMBER, 'red': TrafficLightElement.RED}
RGB = {'green': (0.1, 0.9, 0.1), 'yellow': (1.0, 0.8, 0.0), 'red': (1.0, 0.1, 0.1)}


class TrafficLightSim(Node):
    def __init__(self):
        super().__init__('traffic_light_sim')
        m = hdmap.load(self.declare_parameter('hdmap_file', '').value)
        for name, default in (('green', 20.0), ('yellow', 3.0), ('red', 23.0)):
            self.declare_parameter(name, default)
        self.groups = {i: g for i, (g, _) in hdmap.traffic_lights(m).items()}
        # 灯的位置：regulatory element 引用的 traffic_light 线段的中点，高度取线的 z 再加上灯高的一半
        self.lamps = {}
        for r in m.lmap.regulatoryElementLayer:
            if r.id in self.groups:
                ls = r.trafficLights[0]
                x = sum(p.x for p in ls) / len(ls)
                y = sum(p.y for p in ls) / len(ls)
                self.lamps[r.id] = (x, y, ls[0].z + float(ls.attributes['height']) / 2)
        self.state_pub = self.create_publisher(TrafficLightGroupArray, '/hdmap/traffic_light_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/hdmap/traffic_light_markers', 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info('%d 个红绿灯：%s' % (len(self.groups), self.groups))

    def on_timer(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        g, y, r = (self.get_parameter(n).value for n in ('green', 'yellow', 'red'))
        states = TrafficLightGroupArray(stamp=now.to_msg())
        markers = MarkerArray()
        for i, group in sorted(self.groups.items()):
            color = hdmap.light_color(t, group, g, y, r)
            e = TrafficLightElement(color=MSG_COLOR[color], shape=TrafficLightElement.CIRCLE,
                                    status=TrafficLightElement.SOLID_ON, confidence=1.0)
            states.traffic_light_groups.append(TrafficLightGroup(traffic_light_group_id=i, elements=[e]))
            mk = Marker(type=Marker.SPHERE, action=Marker.ADD, ns='traffic_light', id=i)
            mk.header.frame_id, mk.header.stamp = 'map', now.to_msg()
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = self.lamps[i]
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.6
            mk.color.r, mk.color.g, mk.color.b = RGB[color]
            mk.color.a = 1.0
            markers.markers.append(mk)
        self.state_pub.publish(states)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = TrafficLightSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
