#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import yaml
import cflib.crtp

from cflib.utils.power_switch import PowerSwitch


DEFAULT_CONFIG = Path("/workspace/config/crazyflies.yaml")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Reboot Crazyflies remotely through Crazyradio."
    )

    parser.add_argument(
        "drones",
        nargs="*",
        help=(
            "Crazyflie names to reboot, for example: cf_1 cf_4. "
            "If omitted, all enabled drones are rebooted."
        ),
    )

    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to crazyflies.yaml.",
    )

    return parser.parse_args()


def load_robots(config_path):
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    robots = config.get("robots", {})

    if not robots:
        raise RuntimeError(
            f"No robots found in {config_path}"
        )

    return robots


def select_drones(robots, requested_names):
    if requested_names:
        selected = {}

        for name in requested_names:
            if name not in robots:
                raise RuntimeError(
                    f"{name} not found in crazyflies.yaml"
                )

            selected[name] = robots[name]

        return selected

    # No names specified -> all enabled robots.
    return {
        name: cfg
        for name, cfg in robots.items()
        if cfg.get("enabled", False)
    }


def reboot_drone(name, uri):
    print(f"{name}: rebooting via {uri}")

    power_switch = PowerSwitch(uri)

    try:
        power_switch.reboot_to_fw()
    finally:
        power_switch.close()

    print(f"{name}: reboot command sent")


def main():
    args = parse_args()

    robots = load_robots(args.config)
    selected = select_drones(
        robots,
        args.drones,
    )

    if not selected:
        raise RuntimeError(
            "No Crazyflies selected."
        )

    print("Initializing Crazyradio...")
    cflib.crtp.init_drivers()

    print()
    print("Crazyflies to reboot:")

    for name, cfg in selected.items():
        print(
            f"  {name}: {cfg['uri']}"
        )

    print()

    failures = []

    for name, cfg in selected.items():
        uri = cfg.get("uri")

        if not uri:
            failures.append(
                (name, "missing URI")
            )
            continue

        try:
            reboot_drone(
                name,
                uri,
            )

            # Give the radio/platform a moment before changing
            # to the next URI.
            time.sleep(1.0)

        except Exception as exc:
            failures.append(
                (name, str(exc))
            )

    print()

    if failures:
        print("Some reboots failed:")

        for name, error in failures:
            print(
                f"  {name}: {error}"
            )

        raise SystemExit(1)

    print("Reboot complete.")


if __name__ == "__main__":
    main()
