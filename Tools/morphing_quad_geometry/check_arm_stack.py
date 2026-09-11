#!/usr/bin/env python3
"""Compile allocator sources with ARM stack reports and check the added call paths.

This is a static regression check, not a hardware stack high-water measurement.
Use the same configured GCC ARM build directory as the firmware build.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    entries = json.loads((args.build_dir / 'compile_commands.json').read_text())
    targets = {'ControlAllocator.cpp', 'ActuatorEffectivenessMorphingQuad.cpp', 'ControlAllocationPseudoInverse.cpp'}
    records = []
    sources = set()
    with tempfile.TemporaryDirectory(prefix='morphing-stack-') as tmp:
        for entry in entries:
            name = Path(entry['file']).name
            if name not in targets:
                continue
            command = shlex.split(entry['command'])
            if 'arm-none-eabi' not in command[0]:
                raise RuntimeError('An ARM build is required')
            command[command.index('-o') + 1] = str(Path(tmp) / (name + '.o'))
            command.append('-fstack-usage')
            subprocess.run(command, cwd=entry['directory'], check=True)
            sources.add(name)
        if sources != targets:
            raise RuntimeError(f'Missing compile commands: {targets - sources}')
        for file in Path(tmp).glob('*.su'):
            for line in file.read_text().splitlines():
                function, size, kind = line.rsplit('\t', 2)
                records.append((function, int(size), kind))

    def frame(fragment):
        found = [(size, kind) for name, size, kind in records if fragment in name]
        if not found or any(kind != 'static' for _, kind in found):
            raise RuntimeError(f'Missing or non-static stack report: {fragment}')
        return max(size for size, _ in found)

    ca = next(Path(e['file']) for e in entries if Path(e['file']).name == 'ControlAllocator.cpp')
    source_root = ca.parents[3]
    header = source_root / 'platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp'
    match = re.search(r'rate_ctrl\{"wq:rate_ctrl",\s*(\d+)', header.read_text())
    if not match:
        raise RuntimeError('Cannot read rate-control work-queue stack budget')
    budget = int(match.group(1))
    common = frame('ControlAllocator::Run()') + frame('ControlAllocator::parameters_updated()') + frame('ControlAllocator::update_effectiveness_matrix_if_needed(')
    paths = {
        'geometry_logging': common + frame('ActuatorEffectivenessMorphingQuad::getEffectivenessMatrix(') + frame('ActuatorEffectivenessMorphingQuad::publishGeometry(') + frame('ActuatorEffectivenessMorphingQuad::computeMatrix('),
        'normalization_inverse': common + frame('ControlAllocationPseudoInverse::setNormalizationReference(') + frame('bool matrix::geninv(') + max(frame('matrix::fullRankCholesky('), frame('bool matrix::inv(')),
    }
    report = {'work_queue_stack_bytes': budget, 'update_function_frame_bytes': frame('ControlAllocator::update_effectiveness_matrix_if_needed('), 'audited_path_bytes': paths, 'reserved_headroom_bytes': 512,
              'limitation': 'Known nested frames only; reserve covers uncounted wrappers/library calls. Not proof of all runtime paths or interrupt behavior.'}
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + '\n')
    if report['update_function_frame_bytes'] > 256 or max(paths.values()) + 512 > budget:
        raise RuntimeError('Morphing allocator exceeds the static stack regression budget')


if __name__ == '__main__':
    main()
