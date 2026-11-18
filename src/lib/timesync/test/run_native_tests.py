#!/usr/bin/env python3
"""Compile production C++ and replay closed recordings; no device/network I/O."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess


REPO = Path(__file__).resolve().parents[4]
TEST = Path(__file__).resolve().parent
DATASETS = (
    ('resume_first', 'resume_flight_prepare_20261106T1012Z/preparation/attempt_2', None),
    ('resume_settled', 'resume_flight_prepare_settled_20261106T1020Z/preparation/attempt_1', None),
    ('old_receipt_fallback', None, 'fixedneutral_speed1radius5prep'),
    ('old_passed_bias015', None, 'fixedneutral_bias015prep'),
    ('battery_converted_failure', 'battery_requalification_20261105/preparation/attempt_2', None),
    ('battery_receipt_fallback', 'fixedneutral_hover_20261105T215559_316726490Z/preparation/attempt_1', None),
    ('old_filter18ms_failure', None, 'timesyncdiag'),
    ('old_postconvergence5ms_failure', None, 'fixedneutralready'),
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return dict(count=len(values), min=values[0], median=values[len(values) // 2], max=values[-1])


def source_contract():
    receiver = (REPO / 'src/modules/mavlink/mavlink_receiver.cpp').read_text()
    body = receiver.split('MavlinkReceiver::handle_message_odometry(')[1].split('\nvoid\n')[0]
    block = body.split('const matrix::Vector3f odom_in_p')[0]
    assert 'MorpheusVisionTiming::strict_input(msg->sysid, msg->compid, odom_in.estimator_type,' in block
    assert 'odom_in.frame_id, odom_in.child_frame_id)' in block
    assert 'if (!_mavlink_timesync.sync_vision_stamp(odom_in.time_usec, odom.timestamp_sample))' in block
    assert 'return;' in block.split('if (!_mavlink_timesync.sync_vision_stamp')[1].split('} else')[0]
    assert '} else {\n\t\todom.timestamp_sample = _mavlink_timesync.sync_stamp(odom_in.time_usec);' in block
    assert '_visual_odometry_pub.publish' not in block
    assert 'sync_stamp(att_pos_mocap.time_usec)' in receiver
    assert 'sync_stamp(vpe.usec)' in receiver
    assert 'sync_stamp(trajectory.time_usec)' in receiver
    assert 'sync_stamp(landing_target.time_usec)' in receiver
    return dict(passed=True, scope='receiver rejection before parsing/publication; all four other legacy callers preserved')


def replay(binary, root, output, name, directory, prefix):
    from pyulog import ULog
    if directory:
        base = root / directory
        proof_path, ulog_path, events_path = base / 'timing_report.json', base / 'timestamp.ulg', base / 'recording/events.jsonl'
    else:
        proof_path = root / (prefix + '_fc_timestamp_review.json')
        ulog_path = root / (prefix + '_fc_timestamp.ulg')
        events_path = root / (prefix + '_fc_timestamp_probe/events.jsonl')
    proof = json.loads(proof_path.read_text())
    requests, answers = {}, []
    for line in events_path.open():
        event = json.loads(line)
        message = event.get('message', {})
        if message.get('mavpackettype') != 'TIMESYNC':
            continue
        if event['kind'] == 'fc_rx' and message['tc1'] == 0:
            requests[message['ts1']] = event
        elif event['kind'] == 'fc_tx' and message['tc1'] > 0:
            answers.append(event)
    paired = {a['message']['tc1'] // 1000: a for a in answers if a['message']['ts1'] in requests}
    updates = []
    unmatched = 0
    for dataset in ULog(str(ulog_path)).data_list:
        if dataset.name != 'timesync_status':
            continue
        for index in range(len(dataset.data['timestamp'])):
            row = {k: int(v[index]) for k, v in dataset.data.items()}
            answer = paired.get(row['remote_timestamp'])
            if answer is None:
                unmatched += 1
                continue
            message = answer['message']
            # RTT was computed at Timesync::update(now); the publication timestamp
            # is a few us later. Reconstruct the actual update instant, not a fit.
            originated_us = message['ts1'] // 1000
            now = originated_us + row['round_trip_time']
            updates.append((now, 0, f"U {now} {message['tc1']} {message['ts1']}", None))
    commands = updates + [(m['px4_timestamp_us'], 1,
        f"Q {m['px4_timestamp_us']} {m['sent_capture_host_ns'] // 1000}", m) for m in proof['matches']]
    commands.sort(key=lambda c: (c[0], c[1]))
    run_dir = output / name
    run_dir.mkdir(exist_ok=False)
    input_path = run_dir / 'commands.txt'
    input_path.write_text('\n'.join(c[2] for c in commands) + '\n')
    result = subprocess.run([str(binary), '--replay'], input=input_path.read_text(), text=True,
                            capture_output=True, check=True)
    (run_dir / 'native_output.csv').write_text(result.stdout)
    parsed = list(csv.DictReader(io.StringIO(result.stdout)))
    if len(parsed) != len(commands):
        raise ValueError('Native output count mismatch')
    errors, bounds, drops, faults, generations, original_errors = [], [], [], set(), set(), []
    for native, command in zip(parsed, commands):
        faults.add(int(native['fault']))
        generations.add(int(native['generation']))
        match = command[3]
        if match is None:
            continue
        if native['converted'] != '1':
            drops.append(dict(fc_us=command[0], ready=int(native['ready']), fault=int(native['fault']),
                              samples=int(native['samples']), generation=int(native['generation'])))
            continue
        # Reuse the ORIGINAL independent clock centre and original uncertainty.
        # Neither the candidate nor its outputs are fitted to these values.
        expected_fc_us = match['px4_timestamp_sample_us'] - match['error_ms'] * 1000
        error_ms = (int(native['converted_us']) - expected_fc_us) / 1000
        independent_bound_ms = match['absolute_error_bound_ms'] - abs(match['error_ms'])
        errors.append(error_ms)
        bounds.append(abs(error_ms) + independent_bound_ms)
        original_errors.append(match['error_ms'])
    summary = dict(name=name, original_proof_passed=proof['passed'], original_blockers=proof['blockers'],
        original_error_ms=proof['sample_minus_capture_ms'], original_bound_ms=proof['absolute_error_bound_ms'],
        replay_update_count=len(updates), unmatched_status_rows=unmatched,
        queried_poses=len(proof['matches']), converted_poses=len(errors), dropped_poses=len(drops),
        first_drop=drops[0] if drops else None, last_drop=drops[-1] if drops else None,
        candidate_centre_error_ms=distribution(errors),
        candidate_bound_using_original_independent_map_ms=distribution(bounds),
        candidate_bound_exceeded6ms_count=sum(b > 6 for b in bounds),
        candidate_fault_codes=sorted(faults), candidate_generations=sorted(generations),
        provenance={str(p.resolve()): sha(p) for p in (proof_path, ulog_path, events_path, input_path)},
        native_output_sha256=sha(run_dir / 'native_output.csv'))
    first_update_us = min(c[0] for c in updates)
    ready_rows = [r for r in parsed if r['ready'] == '1']
    converted_rows = [r for r in parsed if r['converted'] == '1']
    summary.update(first_ready_after_first_update_s=(int(ready_rows[0]['now_us']) - first_update_us) / 1e6
                   if ready_rows else None,
        first_conversion_after_first_query_s=(int(converted_rows[0]['now_us']) - proof['matches'][0]['px4_timestamp_us']) / 1e6
                   if converted_rows else None,
        dropped_fraction=len(drops) / len(proof['matches']) if proof['matches'] else None)
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def replay_combined_idle(binary, root, output):
    """Carry one actual native object through both windows and the observed gap."""
    source_dirs = ['resume_first', 'resume_settled']
    commands = []
    match_index = {}
    ranges = {}
    for label, (_, directory, _) in zip(source_dirs, DATASETS[:2]):
        label_commands = []
        for line in (output / label / 'commands.txt').read_text().splitlines():
            fields = line.split()
            label_commands.append((int(fields[1]), 0 if fields[0] == 'U' else 1, line))
        commands.extend(label_commands)
        ranges[label] = (min(c[0] for c in label_commands), max(c[0] for c in label_commands))
        for match in json.loads((root / directory / 'timing_report.json').read_text())['matches']:
            match_index[(match['px4_timestamp_us'], match['sent_capture_host_ns'] // 1000)] = label, match
    first_end = ranges['resume_first'][1]
    second_start = ranges['resume_settled'][0]
    gap_probe = (first_end + second_start) // 2
    commands.append((gap_probe, 1, f'Q {gap_probe} 1'))
    commands.sort(key=lambda c: (c[0], c[1]))
    directory = output / 'combined_actual_idle'
    directory.mkdir()
    input_path = directory / 'commands.txt'
    input_path.write_text('\n'.join(c[2] for c in commands) + '\n')
    result = subprocess.run([str(binary), '--replay'], input=input_path.read_text(), text=True,
                            capture_output=True, check=True)
    (directory / 'native_output.csv').write_text(result.stdout)
    parsed = list(csv.DictReader(io.StringIO(result.stdout)))
    aggregates = {label: dict(errors=[], bounds=[], dropped=0) for label in source_dirs}
    probe_result = None
    for native, command in zip(parsed, commands):
        fields = command[2].split()
        if fields[0] != 'Q':
            continue
        if command[0] == gap_probe:
            probe_result = native
            continue
        label, match = match_index[(command[0], int(fields[2]))]
        selected = aggregates[label]
        if native['converted'] != '1':
            selected['dropped'] += 1
            continue
        expected = match['px4_timestamp_sample_us'] - match['error_ms'] * 1000
        error = (int(native['converted_us']) - expected) / 1000
        selected['errors'].append(error)
        selected['bounds'].append(abs(error) + match['absolute_error_bound_ms'] - abs(match['error_ms']))
    report = dict(scope='same compiled production object, actual two-window idle and explicit synthetic midgap query',
        synthetic_midgap_query=probe_result,
        faults=sorted({int(r['fault']) for r in parsed}),
        generations=sorted({int(r['generation']) for r in parsed}),
        windows={label: dict(dropped=value['dropped'], converted=len(value['errors']),
            centre_error_ms=distribution(value['errors']), bound_ms=distribution(value['bounds']))
                 for label, value in aggregates.items()},
        commands_sha256=sha(input_path), output_sha256=sha(directory / 'native_output.csv'))
    assert probe_result['converted'] == '0' and probe_result['ready'] == '0'
    assert int(probe_result['generation']) == 1
    assert report['faults'] == [0] and report['windows']['resume_settled']['converted'] > 100
    (directory / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--session-root', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    binary = args.output / 'native_clock'
    compile_command = ['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g', '-O1',
        '-I', str(TEST / 'stubs'), str(REPO / 'src/lib/timesync/Timesync.cpp'),
		str(REPO / 'src/lib/timesync/BoundedVisionClock.cpp'),
        str(TEST / 'NativeVisionClockTest.cpp'), '-o', str(binary)]
    compiled = subprocess.run(compile_command, text=True, capture_output=True, check=True)
    native = subprocess.run([str(binary)], text=True, capture_output=True, check=True)
    (args.output / 'compile.log').write_text(compiled.stdout + compiled.stderr)
    (args.output / 'native_tests.log').write_text(native.stdout + native.stderr)
    sources = [REPO / 'src/lib/timesync/Timesync.cpp', REPO / 'src/lib/timesync/Timesync.hpp',
		REPO / 'src/lib/timesync/BoundedVisionClock.cpp',
        REPO / 'src/lib/timesync/BoundedVisionClock.hpp', REPO / 'src/lib/timesync/MorpheusVisionTiming.hpp',
        REPO / 'src/modules/mavlink/mavlink_timesync.cpp', REPO / 'src/modules/mavlink/mavlink_timesync.h',
        REPO / 'src/modules/mavlink/mavlink_receiver.cpp', REPO / 'msg/TimesyncStatus.msg',
        TEST / 'NativeVisionClockTest.cpp', Path(__file__)]
    report = dict(schema='morpheus_native_bounded_clock_candidate_v1', flight_qualified=False,
        compile_command=compile_command, native_tests=native.stdout.strip(),
        source_contract=source_contract(), binary_sha256=sha(binary),
        source_hashes={str(p): sha(p) for p in sources}, datasets=[],
        limitations=[
            'Actual production Timesync.cpp and strict mapper are compiled. uORB publication and hrt are deterministic test adapters.',
            'The full MAVLink receiver translation unit is checked by source contract here; the separate full firmware build is also required.',
            'Counterfactual native outputs are not a new physical timing proof or flight qualification. Original failed proofs remain failed.',
            'No host clock, firmware, device, radio, motor or aircraft state is accessed or changed.',
            'Replay includes only exact matched recorded exchanges; unrecorded clients/history are not invented.',
            'Each replay starts a new candidate object, matching fresh firmware initialization rather than fitting an unknown initial skew.'])
    if args.session_root:
        for name, directory, prefix in DATASETS:
            report['datasets'].append(replay(binary, args.session_root, args.output, name, directory, prefix))
        report['combined_idle'] = replay_combined_idle(binary, args.session_root, args.output)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(native.stdout.strip())
    for result in report['datasets']:
        print(result['name'], 'converted/dropped', result['converted_poses'], result['dropped_poses'],
              'bounds', result['candidate_bound_using_original_independent_map_ms'], 'faults', result['candidate_fault_codes'])
    print(args.output / 'report.json')


if __name__ == '__main__':
    main()
