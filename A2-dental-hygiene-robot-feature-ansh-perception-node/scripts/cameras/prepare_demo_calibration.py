#!/usr/bin/env python3
"""Import checked camera C2 output and independently measured marker-to-jaw layout.

No robot commands, no TF publication, no guessed physical measurements.
Creates a VERIFIED-BY-OPERATOR candidate real perception config and TF script.
Use --layout only after physically measuring every marker in the CAD frame.
"""
import argparse
import math
import shlex
from pathlib import Path
import yaml


def triplet(values, length, label):
    if not isinstance(values, list) or len(values) != length or not all(
        isinstance(v, (int,float)) and math.isfinite(v) for v in values):
        raise ValueError(f'{label}: expected {length} finite numeric values')
    return [float(v) for v in values]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overhead', type=Path, required=True,
                        help='overhead_dental_bench.yaml saved by overhead_calib.py')
    parser.add_argument('--layout', type=Path, required=True,
                        help='measured marker_layout.yaml; never approximate for real UR5e')
    parser.add_argument('--output-dir', type=Path, default=Path('calibration/session'))
    parser.add_argument('--template', type=Path,
                        default=Path('src/dental_perception/config/perception.real.yaml'))
    args = parser.parse_args()
    overhead = yaml.safe_load(args.overhead.read_text())
    layout = yaml.safe_load(args.layout.read_text())
    if overhead.get('parent') != 'base_link' or overhead.get('child') != 'overhead_cam_optical':
        raise ValueError('Calibration frames must be base_link -> overhead_cam_optical. Check camera_info frame ID')
    xyz=triplet(overhead.get('xyz'),3,'C2 translation')
    q=triplet(overhead.get('quat_xyzw'),4,'C2 quaternion')
    if not 0.999 < math.sqrt(sum(v*v for v in q)) < 1.001:
        raise ValueError('C2 quaternion is not normalized')
    rms=float(overhead.get('reprojection_px',math.inf))
    loo=overhead.get('leave_one_out_mm',{})
    if not isinstance(loo,dict) or len(loo)<4:
        raise ValueError('Missing four-marker leave-one-out calibration checks')
    worst=max(float(v) for v in loo.values())
    if not math.isfinite(rms) or rms>4 or not math.isfinite(worst) or worst>10:
        raise ValueError(f'C2 measurement is not acceptable even for a preliminary hover: '
                         f'RMS={rms:.2f}px, worst LOO={worst:.1f}mm')
    size=float(layout.get('marker_size_m',0))
    if abs(size-0.060)>0.0005:
        raise ValueError('The actual ArUco black square is 0.060m, NOT 0.080m outer card')
    centres=triplet(layout.get('marker_centres_xyz_m'),12,'marker centres')
    yaw=triplet(layout.get('marker_yaw_rad'),4,'decoded marker +X yaw')
    for i in range(4):
        for j in range(i+1,4):
            dist=math.hypot(centres[3*i]-centres[3*j],centres[3*i+1]-centres[3*j+1])
            if dist<0.048:
                raise ValueError(f'Marker centres {i},{j} overlap; are your 3D measurements correct?')
    if not layout.get('physically_measured',False):
        raise ValueError('Set physically_measured: true after checking centres, axes and yaw angles')
    config=yaml.safe_load(args.template.read_text())
    localizer=config['model_localizer']['ros__parameters']
    localizer.update(marker_layout_calibrated=True,marker_size_m=size,
                     marker_centres_xyz_m=centres,marker_yaw_rad=yaw,
                     overhead_input_mode='camera',model_pose_reference='camera')
    inspect=config['perception']['ros__parameters']
    inspect['rgbd_input_mode']='camera'
    args.output_dir.mkdir(parents=True,exist_ok=True)
    config_file=args.output_dir/'perception.real.yaml'
    config_file.write_text('# GENERATED FROM MEASURED marker layout + C2 calibration.\n'
                           '# Real robot movement still requires independently verified collision scene, brush TCP,\n'
                           '# active controller, and operator approval.\n'
                           +yaml.safe_dump(config,sort_keys=False))
    command='ros2 run tf2_ros static_transform_publisher '
    command+=' '.join(f'--{key} {value:.8f}' for key,value in zip(
        ('x','y','z','qx','qy','qz','qw'),xyz+q))
    command+=' --frame-id base_link --child-frame-id overhead_cam_optical'
    tf_script=args.output_dir/'overhead_tf.sh'
    tf_script.write_text('#!/usr/bin/env bash\nset -euo pipefail\n'+command+'\n')
    tf_script.chmod(0o755)
    print('C2 preliminary checks: rms=%.2fpx, worst leave-one-out=%.1fmm' % (rms,worst))
    print('READY FOR INSPECTION-ONLY VALIDATION, NOT APPROVED FOR PHYSICAL MOTION.')
    print('Config:',config_file,'\nTransform script:',tf_script)
    print('Next: independently check tooth-frame locations and model-to-camera depth at the real workcell.')

if __name__ == '__main__':
    main()
