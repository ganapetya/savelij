#!/usr/bin/env python3
"""
Wheel-balance mechanical diagnostic for the Yahboom X3.

Why this script exists
----------------------
The Phase 0 odometry test on this robot revealed it physically curves ~17 cm/m
to the right when commanded straight forward, even though /odom honestly
reports the curve. To understand WHY the chassis curves, we need to look
below /vel_raw at the per-wheel level: are the wheels actually spinning at
the same rate, or is one side faster than the other?

The Rosmaster firmware exposes `get_motor_encoder()` returning four raw
encoder values m1, m2, m3, m4. Their physical-wheel mapping is not
documented in the Python wrapper, so this script runs in two modes:

  discover : you manually rotate each wheel; the script tells you which
             encoder index responds. Run this once per hardware setup,
             then write down the m1..m4 -> {FL, FR, BL, BR} mapping.

  measure  : the script commands a pure-forward chassis velocity for a
             few seconds and reports each encoder's rate (ticks/s) plus
             the deviation from the four-wheel mean. If left and right
             rates differ, the firmware-level wheel control is unbalanced.
             If all four match, the asymmetry is mechanical (wheel
             diameter, traction, etc.) -- not in the motor loop.

How to use this
---------------
1. Stop the running bringup (Ctrl-C in the terminal running
   `ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py`).
   This script needs exclusive use of /dev/ttyUSB1.

2. Place the robot ON BLOCKS so the wheels spin freely off the floor.
   This eliminates floor slip from the measurement and is also safer
   (the robot can't run away if the script crashes).

3. Run:

     python3 wheel_balance_diagnostic.py discover
     python3 wheel_balance_diagnostic.py measure --on-blocks

4. When done, restart the bringup as usual.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        "ERROR: Rosmaster_Lib not importable. Make sure you're running on the\n"
        "Jetson where the Yahboom firmware library is installed.\n"
        f"Original error: {exc}\n")
    raise SystemExit(2)


DEFAULT_SERIAL = "/dev/ttyUSB1"
DEFAULT_FORWARD_SPEED = 0.15  # m/s
DEFAULT_MEASURE_DURATION = 3.0  # seconds
SAMPLE_PERIOD_S = 0.1


def open_rosmaster(serial_port: str) -> Rosmaster:
    """Open the chassis MCU and start the receive thread."""
    car = Rosmaster(car_type=1, com=serial_port)
    car.set_car_type(1)
    car.create_receive_threading()
    # Give the receive thread a moment to populate __encoder_m{1..4}.
    time.sleep(0.5)
    return car


def safe_stop(car: Rosmaster) -> None:
    try:
        car.set_car_motion(0.0, 0.0, 0.0)
    except Exception:  # noqa: BLE001 -- best-effort stop on shutdown
        pass


def fmt_quad(label: str, values, fmt: str = "{:+8.0f}") -> str:
    return (
        f"  {label}: "
        f"m1={fmt.format(values[0])}  "
        f"m2={fmt.format(values[1])}  "
        f"m3={fmt.format(values[2])}  "
        f"m4={fmt.format(values[3])}"
    )


# --- discover ---------------------------------------------------------------

def cmd_discover(args: argparse.Namespace) -> int:
    print("=" * 64)
    print("WHEEL-MAPPING DISCOVERY")
    print("=" * 64)
    print()
    print("This will help you label which encoder index (m1..m4) corresponds")
    print("to which physical wheel.")
    print()
    print("INSTRUCTIONS:")
    print("  1. Place the robot on blocks -- wheels off the floor.")
    print("  2. After you press ENTER, encoder values stream at 10 Hz.")
    print("  3. Spin ONE wheel by hand FORWARD for a couple seconds.")
    print("     Note which encoder index changed.")
    print("  4. Stop. Spin the next wheel. Repeat for all four.")
    print("  5. Press Ctrl-C when done. Write down the mapping.")
    print()
    input("Press ENTER when ready, or Ctrl-C to abort...")

    car = open_rosmaster(args.serial_port)
    try:
        # Snapshot baseline encoders so deltas read 0 at start.
        e0 = car.get_motor_encoder()
        print()
        print(f"Baseline encoders: m1={e0[0]}  m2={e0[1]}  m3={e0[2]}  m4={e0[3]}")
        print()
        print("Streaming deltas (wiggle each wheel one at a time)...")
        print("Press Ctrl-C to finish.")
        print()
        try:
            while True:
                e = car.get_motor_encoder()
                de = tuple(e[i] - e0[i] for i in range(4))
                # \r so the line updates in place, like a meter.
                sys.stdout.write(
                    f"\r  delta: m1={de[0]:+8d}  m2={de[1]:+8d}  "
                    f"m3={de[2]:+8d}  m4={de[3]:+8d}     ")
                sys.stdout.flush()
                time.sleep(SAMPLE_PERIOD_S)
        except KeyboardInterrupt:
            print()
            print()
            print("Discovery stopped. Note the mapping you observed:")
            print("  m1 = ?    m2 = ?    m3 = ?    m4 = ?")
            print("  (e.g. m1=FL, m2=BL, m3=FR, m4=BR)")
    finally:
        safe_stop(car)
    return 0


# --- measure ----------------------------------------------------------------

def cmd_measure(args: argparse.Namespace) -> int:
    print("=" * 64)
    print("MECHANICAL BALANCE MEASUREMENT")
    print(f"  speed     : {args.speed:.3f} m/s forward (linear.x only)")
    print(f"  duration  : {args.duration:.1f} s")
    print(f"  on-blocks : {args.on_blocks}")
    print(f"  serial    : {args.serial_port}")
    print("=" * 64)
    print()
    if not args.on_blocks:
        print("WARNING: --on-blocks not set. The robot WILL drive forward")
        print("         for {:.1f} s at {:.2f} m/s ({:.2f} m approx).".format(
            args.duration, args.speed, args.speed * args.duration))
        print("         Make sure you have clear space ahead.")
    else:
        print("Robot is on blocks; wheels will spin freely with no ground motion.")
    print()
    input("Press ENTER to start, or Ctrl-C to abort...")

    car = open_rosmaster(args.serial_port)
    samples = []
    try:
        # Make sure we're stopped, then snap the start.
        safe_stop(car)
        time.sleep(0.4)
        e_start = car.get_motor_encoder()
        t0 = time.monotonic()
        samples.append((0.0, e_start))

        # Command motion.
        print(f"Commanding linear.x = {args.speed:.3f} m/s ...")
        car.set_car_motion(args.speed, 0.0, 0.0)

        # Sample loop.
        next_sample_at = t0 + SAMPLE_PERIOD_S
        while True:
            now = time.monotonic()
            if now - t0 >= args.duration:
                break
            if now >= next_sample_at:
                e = car.get_motor_encoder()
                samples.append((now - t0, e))
                next_sample_at += SAMPLE_PERIOD_S
            time.sleep(0.005)

        # Stop and grab final encoder.
        car.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.4)
        e_end = car.get_motor_encoder()
        t_end = time.monotonic() - t0
        samples.append((t_end, e_end))
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motors.")
        return 130
    finally:
        safe_stop(car)

    # ------------------------- analysis -------------------------
    print()
    print("=" * 64)
    print("RAW ENCODER STREAM")
    print("=" * 64)
    print(f"  {'t(s)':>6}   {'m1':>10}  {'m2':>10}  {'m3':>10}  {'m4':>10}")
    for t, e in samples:
        print(f"  {t:>6.2f}   {e[0]:>10d}  {e[1]:>10d}  {e[2]:>10d}  {e[3]:>10d}")

    print()
    print("=" * 64)
    print("RESULTS")
    print("=" * 64)

    deltas = tuple(e_end[i] - e_start[i] for i in range(4))
    rates = tuple(d / t_end for d in deltas)
    abs_rates = tuple(abs(r) for r in rates)

    print()
    print(f"  Sample window      : {t_end:.2f} s")
    print(fmt_quad("Encoder deltas    ", deltas, "{:+10d}"))
    print(fmt_quad("Rates (ticks/s)   ", rates, "{:+10.1f}"))
    print(fmt_quad("|Rates|           ", abs_rates, "{:10.1f}"))

    avg = sum(abs_rates) / 4.0
    if avg < 1.0:
        print()
        print("Encoders barely moved. Either the chassis didn't move, the")
        print("encoder feedback is broken, or the receive thread didn't")
        print("populate values. Re-run with the bringup definitely stopped.")
        return 1

    deviations = [(r - avg) / avg * 100 for r in abs_rates]
    print()
    print("  Deviation from the 4-wheel mean rate:")
    for i, d in enumerate(deviations):
        print(f"    m{i+1}: {d:+5.1f}%")

    max_dev = max(abs(d) for d in deviations)
    print()
    if max_dev < 2.0:
        verdict = (
            "All four encoders within +/-2 %% of the mean.\n"
            "The motor-loop sees four matched wheel rates, so the curving\n"
            "you observe is NOT motor speed. Likely culprits, in order:\n"
            "  1. Wheel-diameter manufacturing variance (try swapping wheels\n"
            "     left<->right; if drift flips direction, this is it).\n"
            "  2. One wheel mounted slightly toed-in/out.\n"
            "  3. Asymmetric tire wear or rubber compliance.")
    elif max_dev < 5.0:
        verdict = (
            f"Mild asymmetry up to {max_dev:.1f} %%.\n"
            "The motor PID is not perfectly balanced, but is close.\n"
            "Could be motor brushes, gear backlash, or slightly off PID\n"
            "tuning. Probably not the dominant cause of the chassis curve\n"
            "but contributes.")
    else:
        verdict = (
            f"SIGNIFICANT asymmetry up to {max_dev:.1f} %%.\n"
            "Encoder rates differ by more than 5 %% across wheels even\n"
            "though commanded identical. This is most likely the dominant\n"
            "cause of the curving. Look at:\n"
            "  - Motor PID tuning (set_motion_pid).\n"
            "  - Worn brushes or sticky gearbox on the slow wheel.\n"
            "  - Loose connector / poor power on one motor.")
    print(verdict)
    print()
    print("To turn this into a left/right answer, combine with the m1..m4")
    print("-> wheel mapping you found in `discover` mode. Average the two")
    print("left wheels, average the two right wheels, compare.")
    return 0


# --- entry point ------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="wheel_balance_diagnostic",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--serial-port", default=DEFAULT_SERIAL,
        help=f"MCU serial device (default: {DEFAULT_SERIAL}).")

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_disc = sub.add_parser(
        "discover",
        help="Manually spin wheels to map encoder index -> physical wheel.")
    p_disc.set_defaults(func=cmd_discover)

    p_meas = sub.add_parser(
        "measure",
        help="Command forward motion and report per-wheel asymmetry.")
    p_meas.add_argument(
        "--speed", type=float, default=DEFAULT_FORWARD_SPEED,
        help="Linear forward speed (m/s).")
    p_meas.add_argument(
        "--duration", type=float, default=DEFAULT_MEASURE_DURATION,
        help="Measurement duration (s).")
    p_meas.add_argument(
        "--on-blocks", action="store_true",
        help="Confirm the robot is on blocks (suppresses 'will drive' warning).")
    p_meas.set_defaults(func=cmd_measure)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args) or 0


if __name__ == "__main__":
    raise SystemExit(main())
