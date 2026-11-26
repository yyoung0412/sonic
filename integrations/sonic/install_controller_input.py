"""Install private input binding and shared single-reader tracking for SONIC."""
from pathlib import Path
import argparse
import hashlib
from datetime import datetime, timezone

BASELINE = '4ea92b19a42604a2d487a7cc78ea7cca1bd4be93359217832889c290fbb08925'
PATCHED = '98591749fa2f0ba43f860f9c6eee81971ca8e1d5eb5349efd7067d7bd0694ebc'
MARKER = '# Robot Station: optional private manager uplink.'
TRACKING_MARKER = '# Robot Station: single SDK reader with shared tracking.'
PREVIOUS_TRACKED = '1f86f283806b41d6a1818cf0b02059c6aba1591a4d4a4d14569693b184707375'
PREVIOUS_HELPERS = ('91e48496cd96058d713ee61db50c2b822a8e4a4d65cea7823abc01706af6b559',
                    '88b5c345157a1435db86058088722b9a899f3469ff6d6a8954c84394659ca76c',
                    '526acc8f3a25124ff3b247d4e4098ef47cfe3413425233f6b526e3bf37216ecc')
PREVIOUS_THREADED = '43fbc025844ddba782b44102e6eb1e29ac10da9c74473086be546b16e3050682'
PREVIOUS_READY = '06b7cb21b8ed64f68246220949d27aa8e607b6d3708c9db0456ec06e131defe3'
PREVIOUS_RELAY = '39a120fbed1eeaf75fe218bb7eebf8940b81afc5f7f3dd71f6458be7095b0291'
TRACKED = '0875560325f78c08eece751f6c5baf0a974c8e51e5c308d0d557adae591cdbc6'


def install(root):
    root = Path(root).resolve()
    path = root / 'gear_sonic/scripts/pico_manager_thread_server.py'
    source = path.read_text()
    checksum = hashlib.sha256(path.read_bytes()).hexdigest()
    helper = root / 'gear_sonic/utils/teleop/station_tracking.py'
    helper_source = Path(__file__).with_name('station_tracking.py').read_bytes()
    if checksum == TRACKED and helper.is_file() and helper.read_bytes() == helper_source:
        compile(source, str(path), 'exec')
        return 'already installed'
    if checksum not in (BASELINE, PATCHED, PREVIOUS_TRACKED, PREVIOUS_THREADED, PREVIOUS_READY, PREVIOUS_RELAY, TRACKED):
        raise RuntimeError('PICO manager differs from reviewed baseline; inspect before applying integration')
    if checksum != TRACKED:
        if checksum == BASELINE:
            source = bind_patch(source)
        if checksum == PREVIOUS_RELAY:
            source = pace_patch(source)
        elif checksum in (PREVIOUS_TRACKED, PREVIOUS_THREADED, PREVIOUS_READY):
            source = relay_patch(source)
        else:
            source = tracking_patch(source)
    compile(source, str(path), 'exec')
    compile(helper_source, str(helper), 'exec')
    backup = root / '.local-backups' / ('station-manager-tracking-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S'))
    backup.mkdir(parents=True, exist_ok=False)
    (backup / path.name).write_bytes(path.read_bytes())
    if helper.exists():
        if helper.read_bytes() != helper_source and hashlib.sha256(helper.read_bytes()).hexdigest() not in PREVIOUS_HELPERS:
            raise RuntimeError('Existing shared tracking helper differs; inspect before overwriting')
        (backup / helper.name).write_bytes(helper.read_bytes())
    temporary = helper.with_suffix('.station-tmp')
    temporary.write_bytes(helper_source)
    temporary.replace(helper)
    temporary = path.with_name(path.name + '.station-tracking')
    temporary.write_text(source)
    temporary.chmod(path.stat().st_mode)
    temporary.replace(path)
    return 'installed; original saved in ' + str(backup)


def bind_patch(source):
    anchor = 'def run_pico_manager(\n    port: int = 5556,'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor + '\n    bind_host: str = "*",  ' + MARKER)
    anchor = '    socket.bind(f"tcp://*:{port}")'
    start = source.index('def run_pico_manager(')
    before, after = source[:start], source[start:]
    assert after.count(anchor) == 1
    after = after.replace(anchor, '    socket.bind(f"tcp://{bind_host}:{port}")')
    source = before + after
    anchor = '    parser.add_argument("--port", type=int, default=5556, help="ZMQ server port (default: 5556)")'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor + '\n    parser.add_argument("--bind-host", choices=["*", "127.0.0.1"], default="*", help="Manager publisher bind address")')
    start = source.index('        run_pico_manager(')
    anchor = '            port=args.port,'
    before, after = source[:start], source[start:]
    assert anchor in after
    source = before + after.replace(anchor, anchor + '\n            bind_host=args.bind_host,', 1)
    return source


def tracking_patch(source):
    start = source.index('class PicoReader:')
    end = source.index('\ndef _pose_stream_common(', start)
    reader = source[start:end].replace('xrt.get_time_stamp_ns()', 'xrt.get_body_timestamp_ns()')
    reader = reader.replace('if prev_stamp_ns is not None and stamp_ns == prev_stamp_ns:',
                            'if stamp_ns <= 0 or stamp_ns == prev_stamp_ns:')
    source = source[:start] + reader + source[end:]
    start = source.index('def run_pico_manager(')
    end = source.index('\nif __name__ == "__main__":', start)
    manager = source[start:end]
    anchor = '    input_source: str = "xrt",\n'
    assert manager.count(anchor) == 1
    manager = manager.replace(anchor, anchor + '    station_status_file: str = "",  ' + TRACKING_MARKER + '\n')
    anchor = '    reader = _init_input_source(input_source, buffer_size)'
    assert manager.count(anchor) == 1
    manager = manager.replace(anchor,
        '    from gear_sonic.utils.teleop.station_tracking import StationTracking\n'
        '    tracking = StationTracking(station_status_file) if station_status_file else None\n' + anchor)
    anchor = '        reader.stop()\n'
    assert manager.count(anchor) == 1
    manager = manager.replace(anchor, anchor +
        '        if tracking is not None:\n'
        '            try:\n'
        '                if input_source == "xrt":\n'
        '                    xrt.close()\n'
        '            finally:\n'
        '                tracking.close()\n')
    source = source[:start] + manager + source[end:]
    anchor = '    parser.add_argument("--bind-host",'
    pos = source.index(anchor)
    source = source[:pos] + '    parser.add_argument("--station-status-file", default="", help="Robot Station shared tracking status file")\n' + source[pos:]
    anchor = '            bind_host=args.bind_host,\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor + '            station_status_file=args.station_status_file,\n')
    return relay_patch(source)


def relay_patch(source):
    source = source.replace('    if tracking is not None:\n        tracking.start(reader)\n', '')
    source = source.replace('            if tracking is not None:\n                tracking.update(reader.get_latest())\n', '')
    anchor = '    from gear_sonic.utils.teleop.station_tracking import StationTracking\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, '    global xrt\n' + anchor)
    anchor = '    tracking = StationTracking(station_status_file) if station_status_file else None\n'
    assert source.count(anchor) == 1
    source = source.replace(anchor, anchor +
        '    if tracking is not None:\n'
        '        if input_source != "xrt":\n'
        '            raise ValueError("Managed SDK relay requires xrt input")\n'
        '        xrt = tracking.sdk_proxy()\n')
    return pace_patch(source)


def pace_patch(source):
    start = source.index('def run_pico_manager(')
    before, manager = source[:start], source[start:]
    anchor = '        while True:\n            # Poll Pico controller for buttons/axes\n'
    assert manager.count(anchor) == 1
    manager = manager.replace(anchor, '        while True:\n            iteration_started = time.monotonic()\n            # Poll Pico controller for buttons/axes\n')
    anchor = '            prev_left_axis_click = left_axis_click\n'
    assert manager.count(anchor) == 1
    manager = manager.replace(anchor, anchor + '\n            # Bound managed idle traffic so one-shot commands are not crowded out.\n'
        '            if tracking is not None:\n'
        '                time.sleep(max(0.0, 1.0 / target_fps - (time.monotonic() - iteration_started)))\n')
    return before + manager


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sonic_root', type=Path)
    print(install(parser.parse_args().sonic_root))
