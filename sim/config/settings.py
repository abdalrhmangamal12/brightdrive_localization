#!/usr/bin/env python3
"""One scenario's effective settings: config/sim.yaml `defaults`, overwritten by the scenario's own line.

Used by make_scenarios.py, the bridge, the GNSS gate, the initial-pose publisher and the run script, so the
rule "defaults + the scenario's changes" exists in one place. A scenario may sit under `scenarios:` (the task's
required set) or `extras:` (optional runs); both are looked up.

  python3 sim/config/settings.py gnss_outage gnss_outage_duration_s     -> prints 30.0
  python3 sim/config/settings.py nominal                                -> prints every setting
"""
import sys
from pathlib import Path

import yaml

# sim/ folder (this file is sim/config/settings.py).
SIM = Path(__file__).resolve().parents[1]
SCENARIOS_YAML = SIM / 'config' / 'sim.yaml'


def config():
    """The whole sim.yaml."""
    return yaml.safe_load(open(SCENARIOS_YAML))


def names(cfg=None):
    """(required scenario names, extra scenario names), in file order."""
    cfg = cfg or config()
    return list(cfg.get('scenarios') or {}), list(cfg.get('extras') or {})


def load(name, cfg=None):
    """defaults + the scenario's own changes. Raises KeyError with the valid names if `name` is unknown."""
    cfg = cfg or config()
    required, extras = names(cfg)
    if name in required:
        own = cfg['scenarios'][name]
    elif name in extras:
        own = cfg['extras'][name]
    else:
        raise KeyError(f'unknown scenario "{name}"; scenarios: {", ".join(required)}; extras: {", ".join(extras)}')
    return {**cfg['defaults'], **(own or {})}


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    s = load(sys.argv[1])
    if len(sys.argv) == 2:
        for k, v in s.items():
            print(f'{k}: {v}')
    else:
        v = s[sys.argv[2]]
        # Bash-friendly: nothing for null, lower-case booleans.
        print('' if v is None else str(v).lower() if isinstance(v, bool) else v)


if __name__ == '__main__':
    main()
