#!/usr/bin/env python3
"""Compile firmware geometry on the host and compare with the CAD dynamics model.

Requires numpy and the sibling Morpheus reference repository. All build
artifacts are temporary. The external reference model and URDF are read-only.
"""
import argparse
import hashlib
import importlib.util
import itertools
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True
PX4 = Path(__file__).resolve().parents[2]
TRANSFORM = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
MOTOR_TO_ARM = [0, 2, 3, 1]  # FR, RL, FL, RR


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    roots = [PX4.parent.parent / name for name in ('Morpheus', 'morphing-quadrotor-learning')]
    default_root = next((root for root in roots
                         if (root / 'dynamics/morphing_quadrotor.py').is_file()
                         and (root / 'robot_description/morphing_quadrotor.urdf').is_file()), roots[0])
    parser.add_argument('--model-root', type=Path, default=default_root,
                        help='CAD/model checkout (defaults to sibling Morpheus, then legacy repository name)')
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'clang++'))
    args = parser.parse_args()
    urdf = args.model_root / 'robot_description/morphing_quadrotor.urdf'
    model_path = args.model_root / 'dynamics/morphing_quadrotor.py'
    if not urdf.is_file() or not model_path.is_file():
        parser.error(f'Reference model or URDF missing under {args.model_root}; specify --model-root')
    spec = importlib.util.spec_from_file_location('cad_reference', model_path)
    reference = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = reference
    spec.loader.exec_module(reference)
    model = reference.MorphingQuadrotorDynamics.from_urdf(urdf)
    # Model torque = +spin * ratio * axis. PX4 torque = -KM * CT * axis.
    km = -model.spin_directions * model.reaction_torque_to_thrust
    rng = np.random.default_rng(170)
    angles = np.vstack([
        np.deg2rad(list(itertools.product([-30., 0., 30.], repeat=4))),
        rng.uniform(-np.pi / 6, np.pi / 6, (1000, 4)),
    ])
    inputs, expected, expected_positions = [], [], []
    for index, q in enumerate(angles):
        positions, _ = model.rotor_kinematics(q)
        com_cad = model.mass_properties(q).center_of_mass_body
        # Cover fixed-origin, reference dynamic CoG, and user-specified offsets.
        cog = (np.zeros(3) if index % 3 == 0 else
               TRANSFORM @ com_cad if index % 3 == 1 else
               rng.uniform(-0.01, 0.01, 3))
        ct = np.ones(4) if index < 81 else rng.uniform(0.2, 8., 4)
        physical = model.allocation_matrix(q).copy()
        # Translate torque from model CoM to the requested CoG before rotating.
        delta = com_cad - TRANSFORM.T @ cog
        physical[3:] += np.cross(delta, physical[:3].T).T
        b = np.vstack([TRANSFORM @ physical[3:], TRANSFORM @ physical[:3]])
        b *= ct
        expected.append(b[:, MOTOR_TO_ARM])
        expected_positions.append((TRANSFORM @ positions.T).T[MOTOR_TO_ARM])
        inputs.append(np.concatenate([q, ct, km, cog]))

    with tempfile.TemporaryDirectory(prefix='nxt-geometry-') as temporary:
        binary = Path(temporary) / 'geometry_probe'
        command = shlex.split(args.compiler) + [
            '-std=c++14', '-Wall', '-Wextra', '-Werror', '-pedantic',
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
            '-I', str(PX4 / 'src/modules/control_allocator/ActuatorEffectiveness'),
            str(Path(__file__).with_name('geometry_probe.cpp')), '-o', str(binary),
        ]
        subprocess.run(command, check=True)
        output = subprocess.run(
            [str(binary)], check=True, capture_output=True, text=True,
            input='\n'.join(' '.join(f'{v:.12g}' for v in row) for row in inputs) + '\n',
        ).stdout

    actual = np.array([[float(v) for v in line.split()] for line in output.splitlines()])
    assert actual.shape == (len(inputs), 36), actual.shape
    positions = actual[:, :12].reshape(-1, 4, 3)
    matrices = actual[:, 12:].reshape(-1, 6, 4)
    np.testing.assert_allclose(positions, expected_positions, rtol=0, atol=2e-8)
    np.testing.assert_allclose(matrices, expected, rtol=3e-6, atol=2e-7)
    assert np.all(matrices[:, 3:5] == 0), 'Lateral thrust must stay zero'
    # All sampled configurations retain four-axis authority before PX4 filtering.
    assert all(np.linalg.matrix_rank(b[[0, 1, 2, 5]]) == 4 for b in matrices)
    # Unequal per-motor effort also verifies wrench composition and column order.
    efforts = rng.uniform(0, 1, (len(inputs), 4))
    np.testing.assert_allclose(np.einsum('nij,nj->ni', matrices, efforts),
                               np.einsum('nij,nj->ni', expected, efforts),
                               rtol=3e-6, atol=4e-7)
    print(f'PASS: {len(inputs)} CAD/model comparisons (81 limit/neutral combinations + 1000 seeded samples)')
    print('PASS: frame/sign/order, fixed and moving CoG, unequal CT, wrench and rank checks')
    print('PASS: C++ invalid-input/overflow/unchanged-output checks; address/undefined sanitizers')
    print(f'Max position error: {np.max(np.abs(positions - expected_positions)):.3g} m')
    print(f'Max matrix error: {np.max(np.abs(matrices - expected)):.3g}')
    print(f'URDF SHA256: {hashlib.sha256(urdf.read_bytes()).hexdigest()}')
    print(f'Model SHA256: {hashlib.sha256(model_path.read_bytes()).hexdigest()}')


if __name__ == '__main__':
    main()
