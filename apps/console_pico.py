"""Coordinate the single XRoboToolkit SDK connection across local readers."""
import uuid
from pathlib import Path

from console_support import process_identity, read_json


def active_owner(console):
    owner = read_json(Path(console) / 'sonic-control/pico-owner.json')
    try:
        if uuid.UUID(owner['run_id']).hex != owner['run_id']:
            return {}
        if not owner.get('start_ticks') or process_identity(owner.get('pid')) != owner['start_ticks']:
            return {}
    except (KeyError, ValueError, TypeError, AttributeError):
        return {}
    return owner


def tracking_path(console, owner):
    return Path(console) / 'sonic-control/runs' / owner['run_id'] / 'pico_tracking.json'
