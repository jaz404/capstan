#!/usr/bin/env python3

import asyncio
import argparse
import math
import time
import csv
import os

import moteus
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def get_val(state, reg, default=np.nan):
    try:
        return state.values[reg]
    except Exception:
        return default


async def query_state(c):
    state = await c.set_position(position=math.nan, query=True)

    data = {
        "mode": get_val(state, moteus.Register.MODE),
        "fault": get_val(state, moteus.Register.FAULT),
        "position": get_val(state, moteus.Register.POSITION),
        "velocity": get_val(state, moteus.Register.VELOCITY),
        "torque": get_val(state, moteus.Register.TORQUE),
        "q_current": get_val(state, moteus.Register.Q_CURRENT),
        "d_current": get_val(state, moteus.Register.D_CURRENT),
        "voltage": get_val(state, moteus.Register.VOLTAGE),
        "temperature": get_val(state, moteus.Register.TEMPERATURE),
    }

    return state, data


def print_state(data):
    print("\n--- Moteus State ---")
    for k, v in data.items():
        print(f"{k:>12}: {v}")
    print("--------------------\n")


async def log_sample(c, log, t0, command_pos=np.nan, command_vel=np.nan, command_torque=np.nan):
    state, data = await query_state(c)

    row = {
        "t": time.time() - t0,
        "cmd_pos": command_pos,
        "cmd_vel": command_vel,
        "cmd_torque": command_torque,
        **data,
    }

    log.append(row)
    return row


async def hold_position(c, duration, rate_hz, max_torque):
    print("Holding current position...")

    _, data = await query_state(c)
    target = data["position"]

    log = []
    t0 = time.time()
    dt = 1.0 / rate_hz

    while time.time() - t0 < duration:
        state = await c.set_position(
            position=target,
            velocity=0.0,
            maximum_torque=max_torque,
            query=True,
        )

        row = {
            "t": time.time() - t0,
            "cmd_pos": target,
            "cmd_vel": 0.0,
            "cmd_torque": max_torque,
            "position": get_val(state, moteus.Register.POSITION),
            "velocity": get_val(state, moteus.Register.VELOCITY),
            "torque": get_val(state, moteus.Register.TORQUE),
            "q_current": get_val(state, moteus.Register.Q_CURRENT),
            "d_current": get_val(state, moteus.Register.D_CURRENT),
            "voltage": get_val(state, moteus.Register.VOLTAGE),
            "temperature": get_val(state, moteus.Register.TEMPERATURE),
            "fault": get_val(state, moteus.Register.FAULT),
            "mode": get_val(state, moteus.Register.MODE),
        }

        log.append(row)
        print(f"t={row['t']:.2f} pos={row['position']:.4f} vel={row['velocity']:.4f} tq={row['torque']:.4f}")
        await asyncio.sleep(dt)

    await c.set_stop()
    return log


async def step_test(c, step_rev, duration, rate_hz, max_torque, velocity_limit, accel_limit):
    print("Running position step test...")

    _, data = await query_state(c)
    start = data["position"]
    target = start + step_rev

    log = []
    t0 = time.time()
    dt = 1.0 / rate_hz

    while time.time() - t0 < duration:
        state = await c.set_position(
            position=target,
            velocity=0.0,
            maximum_torque=max_torque,
            velocity_limit=velocity_limit,
            accel_limit=accel_limit,
            query=True,
        )

        row = {
            "t": time.time() - t0,
            "cmd_pos": target,
            "cmd_vel": 0.0,
            "cmd_torque": max_torque,
            "position": get_val(state, moteus.Register.POSITION),
            "velocity": get_val(state, moteus.Register.VELOCITY),
            "torque": get_val(state, moteus.Register.TORQUE),
            "q_current": get_val(state, moteus.Register.Q_CURRENT),
            "d_current": get_val(state, moteus.Register.D_CURRENT),
            "voltage": get_val(state, moteus.Register.VOLTAGE),
            "temperature": get_val(state, moteus.Register.TEMPERATURE),
            "fault": get_val(state, moteus.Register.FAULT),
            "mode": get_val(state, moteus.Register.MODE),
        }

        log.append(row)
        print(f"t={row['t']:.2f} cmd={target:.4f} pos={row['position']:.4f} vel={row['velocity']:.4f} tq={row['torque']:.4f}")
        await asyncio.sleep(dt)

    await c.set_stop()
    return log


async def velocity_test(c, velocity, duration, rate_hz, max_torque):
    print("Running velocity test...")

    log = []
    t0 = time.time()
    dt = 1.0 / rate_hz

    while time.time() - t0 < duration:
        state = await c.set_position(
            position=math.nan,
            velocity=velocity,
            maximum_torque=max_torque,
            query=True,
        )

        row = {
            "t": time.time() - t0,
            "cmd_pos": math.nan,
            "cmd_vel": velocity,
            "cmd_torque": max_torque,
            "position": get_val(state, moteus.Register.POSITION),
            "velocity": get_val(state, moteus.Register.VELOCITY),
            "torque": get_val(state, moteus.Register.TORQUE),
            "q_current": get_val(state, moteus.Register.Q_CURRENT),
            "d_current": get_val(state, moteus.Register.D_CURRENT),
            "voltage": get_val(state, moteus.Register.VOLTAGE),
            "temperature": get_val(state, moteus.Register.TEMPERATURE),
            "fault": get_val(state, moteus.Register.FAULT),
            "mode": get_val(state, moteus.Register.MODE),
        }

        log.append(row)
        print(f"t={row['t']:.2f} cmd_vel={velocity:.4f} pos={row['position']:.4f} vel={row['velocity']:.4f} tq={row['torque']:.4f}")
        await asyncio.sleep(dt)

    await c.set_stop()
    return log


async def torque_test(c, torque, duration, rate_hz):
    print("Running torque test...")

    log = []
    t0 = time.time()
    dt = 1.0 / rate_hz

    while time.time() - t0 < duration:
        state = await c.set_position(
            position=math.nan,
            velocity=0.0,
            maximum_torque=abs(torque),
            feedforward_torque=torque,
            query=True,
        )

        row = {
            "t": time.time() - t0,
            "cmd_pos": math.nan,
            "cmd_vel": 0.0,
            "cmd_torque": torque,
            "position": get_val(state, moteus.Register.POSITION),
            "velocity": get_val(state, moteus.Register.VELOCITY),
            "torque": get_val(state, moteus.Register.TORQUE),
            "q_current": get_val(state, moteus.Register.Q_CURRENT),
            "d_current": get_val(state, moteus.Register.D_CURRENT),
            "voltage": get_val(state, moteus.Register.VOLTAGE),
            "temperature": get_val(state, moteus.Register.TEMPERATURE),
            "fault": get_val(state, moteus.Register.FAULT),
            "mode": get_val(state, moteus.Register.MODE),
        }

        log.append(row)
        print(f"t={row['t']:.2f} cmd_tq={torque:.4f} pos={row['position']:.4f} vel={row['velocity']:.4f} tq={row['torque']:.4f}")
        await asyncio.sleep(dt)

    await c.set_stop()
    return log


def save_csv(log, filename):
    if not log:
        print("No data to save.")
        return

    # save in /logs
    # check if /logs dir exists
    if not os.path.exists("logs"):
        os.makedirs("logs")
        
    filename = os.path.join("logs", filename)
    df = pd.DataFrame(log)
    df.to_csv(filename, index=False)
    print(f"Saved CSV: {filename}")


def plot_log(csv_file):

    # check if /logs dir exists
    if not os.path.exists("logs"):
        os.makedirs("logs")

    # save all charts in /logs
    df = pd.read_csv(csv_file)

    base = os.path.join("logs", os.path.splitext(os.path.basename(csv_file))[0])

    plt.figure()
    plt.plot(df["t"], df["position"], label="measured position")
    if "cmd_pos" in df and not df["cmd_pos"].isna().all():
        plt.plot(df["t"], df["cmd_pos"], "--", label="command position")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [rev]")
    plt.legend()
    plt.grid(True)
    plt.savefig(base + "_position.png", dpi=200)

    plt.figure()
    plt.plot(df["t"], df["velocity"], label="measured velocity")
    if "cmd_vel" in df and not df["cmd_vel"].isna().all():
        plt.plot(df["t"], df["cmd_vel"], "--", label="command velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [rev/s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(base + "_velocity.png", dpi=200)

    plt.figure()
    plt.plot(df["t"], df["torque"], label="measured torque")
    if "cmd_torque" in df and not df["cmd_torque"].isna().all():
        plt.plot(df["t"], df["cmd_torque"], "--", label="command torque")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.legend()
    plt.grid(True)
    plt.savefig(base + "_torque.png", dpi=200)

    plt.figure()
    plt.plot(df["t"], df["voltage"], label="bus voltage")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.legend()
    plt.grid(True)
    plt.savefig(base + "_voltage.png", dpi=200)

    plt.figure()
    plt.plot(df["t"], df["temperature"], label="temperature")
    plt.xlabel("Time [s]")
    plt.ylabel("Temperature [C]")
    plt.legend()
    plt.grid(True)
    plt.savefig(base + "_temperature.png", dpi=200)

    print("Saved plots:")
    print(base + "_position.png")
    print(base + "_velocity.png")
    print(base + "_torque.png")
    print(base + "_voltage.png")
    print(base + "_temperature.png")


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--mode", choices=["query", "hold", "step", "velocity", "torque", "plot"], default="query")

    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=50.0)

    parser.add_argument("--step-rev", type=float, default=0.25)
    parser.add_argument("--velocity", type=float, default=0.5)
    parser.add_argument("--torque", type=float, default=0.1)
    parser.add_argument("--max-torque", type=float, default=0.5)

    parser.add_argument("--velocity-limit", type=float, default=4.0)
    parser.add_argument("--accel-limit", type=float, default=5.0)

    parser.add_argument("--csv", type=str, default="moteus_log.csv")

    args = parser.parse_args()

    if args.mode == "plot":
        plot_log(args.csv)
        return

    c = moteus.Controller(id=args.id)

    try:
        state, data = await query_state(c)
        print_state(data)

        if data["fault"] != 0:
            print("Motor is reporting a fault. Not running motion test.")
            print("Clear/diagnose the fault first using tview or moteus_tool.")
            return

        if args.mode == "query":
            return

        if args.mode == "hold":
            log = await hold_position(c, args.duration, args.rate, args.max_torque)

        elif args.mode == "step":
            log = await step_test(
                c,
                args.step_rev,
                args.duration,
                args.rate,
                args.max_torque,
                args.velocity_limit,
                args.accel_limit,
            )

        elif args.mode == "velocity":
            log = await velocity_test(
                c,
                args.velocity,
                args.duration,
                args.rate,
                args.max_torque,
            )

        elif args.mode == "torque":
            log = await torque_test(
                c,
                args.torque,
                args.duration,
                args.rate,
            )

        save_csv(log, args.csv)
        plot_log(args.csv)

    finally:
        print("Stopping motor.")
        await c.set_stop()


if __name__ == "__main__":
    asyncio.run(main())