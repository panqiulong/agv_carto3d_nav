#!/usr/bin/env python3
"""由语义文件的 zones 生成 Nav2 禁行 / 限速过滤器掩码（与 map.yaml 同尺寸、同原点、raw 模式）。

用法：ros2 run agv_semantic make_filter_masks.py <map.yaml> <map_semantic.yaml>
改了区域之后要重新运行，并重启导航。
"""
import sys

import semantic_map


def main():
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    try:
        paths = semantic_map.write_masks(sys.argv[1], sys.argv[2])
    except (semantic_map.SemanticMapError, OSError, KeyError) as e:
        print('失败：%s' % e, file=sys.stderr)
        sys.exit(1)
    print('saved: %s %s %s' % (paths['keepout_yaml'], paths['speed_yaml'], paths['sha1']))


if __name__ == '__main__':
    main()
