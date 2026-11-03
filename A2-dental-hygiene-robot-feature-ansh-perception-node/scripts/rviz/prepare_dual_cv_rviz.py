#!/usr/bin/env python3
"""Add two dental annotated-image displays to an existing MoveIt RViz config.

This preserves the existing RobotModel, MotionPlanning, scene and TF displays.
No ROS connection or robot commands. Work on a COPY of the UR view_robot.rviz.
"""
import argparse
from pathlib import Path
import yaml

FEEDS = (
    ('Dental Overhead CV', '/dental/perception/overhead_preview'),
    ('Dental Wrist RGB-D CV', '/dental/perception/rgbd_preview'),
)


def display(name, topic):
    return {
        'Class': 'rviz_default_plugins/Image',
        'Enabled': True,
        'Name': name,
        'Normalize Range': True,
        'Topic': {
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': topic,
        },
        'Value': True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    path = Path(args.base_config).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if not path.is_file():
        parser.error('Base RViz config not found: '+str(path))
    if path == output or output.exists():
        parser.error('Output must not exist or overwrite the original RViz config')
    cfg = yaml.safe_load(path.read_text())
    if not isinstance(cfg, dict) or not isinstance(cfg.get('Visualization Manager'), dict):
        parser.error('Source is not a recognised RViz .rviz YAML config')
    existing = cfg['Visualization Manager'].get('Displays')
    if not isinstance(existing, list):
        parser.error('Source .rviz missing Visualization Manager.Displays list')
    names = {x.get('Name') for x in existing if isinstance(x, dict)}
    for name, topic in FEEDS:
        if name not in names:
            existing.append(display(name, topic))
    for name, topic in (
        ('Dental jaw CAD overlay', '/dental/workcell/jaw_mesh'),
        ('Dental brush CAD overlay', '/dental/workcell/brush_mesh')):
        if name not in names:
            existing.append({
                'Class': 'rviz_default_plugins/Marker',
                'Enabled': True,
                'Name': name,
                'Topic': {'Depth': 1, 'Durability Policy': 'Transient Local',
                          'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                          'Value': topic},
                'Value': True,
            })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
    print('Wrote:', output)
    print('Added Image displays:')
    for name, topic in FEEDS:
        print(' - '+name+': '+topic)
    print('Camera nodes must be publishing fresh images for panels to display live video.')
    print('Check placement and save the layout in RViz after opening.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
