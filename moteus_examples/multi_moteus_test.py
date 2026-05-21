#!/usr/bin/python3 -B

# yet to be tested 

import argparse
import asyncio
import math
import time
import csv
import moteus


def get_value(result, register, default=float("nan")):
    try:
        return result.values[register]
    except Exception:
        return default


async def main():
    parser = argparse.ArgumentParser()
    moteus.make_transport_args(parser)

    parser.add_argument("--mode", choices=["query", "velocity", "step", "sine"], default="query")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=50.0)

    parser.add_argument("--velocity", type=float, default=1.0)      # rev/s
    parser.add_argument("--step-rev", type=float, default=0.25)     # rev
    parser.add_argument("--sine-amp", type=float, default=0.5)      # rev/s
    parser.add_argument("--sine-freq", type=float, default=0.5)     # Hz

    parser.add_argument("--max-torque", type=float, default=0.5)    # Nm
    parser.add_argument("--velocity-limit", type=float, default=5.0)
    parser.add_argument("--accel-limit", type=float, default=20.0)

    parser.add_argument("--csv", type=str, default="multi_moteus_log.csv")

    args = parser.parse_args()

    transport = moteus.get_singleton_transport(args)

    print("Discovering moteus controllers...")
    devices = await transport.discover()
    addresses = [x.address for x in devices]

    if not addresses:
        print("No moteus controllers found.")
        return

    print("Found controllers:")
    for address in addresses:
        print(f"  ID: {address}")

    servos = [
        moteus.Controller(id=address, transport=transport)
        for address in addresses
    ]

    print("\nSending stop to all controllers...")
    await transport.cycle([servo.make_stop() for servo in servos])

    # Initial query
    print("\nInitial query:")
    query_commands = [
        servo.make_position(position=math.nan, query=True)
        for servo in servos
    ]
    results = await transport.cycle(query_commands)

    start_positions = {}

    for result in results:
        sid = result.arbitration_id
        pos = get_value(result, moteus.Register.POSITION)
        vel = get_value(result, moteus.Register.VELOCITY)
        tq = get_value(result, moteus.Register.TORQUE)
        fault = get_value(result, moteus.Register.FAULT)
        voltage = get_value(result, moteus.Register.VOLTAGE)
        temp = get_value(result, moteus.Register.TEMPERATURE)

        start_positions[sid] = pos

        print(
            f"ID={sid} "
            f"pos={pos:.4f} rev "
            f"vel={vel:.4f} rev/s "
            f"torque={tq:.4f} Nm "
            f"fault={fault} "
            f"voltage={voltage:.2f} V "
            f"temp={temp:.2f} C"
        )

    if args.mode == "query":
        await transport.cycle([servo.make_stop() for servo in servos])
        return

    log_rows = []
    dt = 1.0 / args.rate
    t0 = time.time()

    try:
        while True:
            t = time.time() - t0

            if t > args.duration:
                break

            commands = []

            for i, servo in enumerate(servos):
                sid = addresses[i]

                if args.mode == "velocity":
                    cmd_pos = math.nan
                    cmd_vel = args.velocity

                elif args.mode == "step":
                    cmd_pos = start_positions.get(sid, 0.0) + args.step_rev
                    cmd_vel = 0.0

                elif args.mode == "sine":
                    cmd_pos = math.nan
                    cmd_vel = args.sine_amp * math.sin(2.0 * math.pi * args.sine_freq * t + i)

                commands.append(
                    servo.make_position(
                        position=cmd_pos,
                        velocity=cmd_vel,
                        maximum_torque=args.max_torque,
                        velocity_limit=args.velocity_limit,
                        accel_limit=args.accel_limit,
                        query=True,
                    )
                )

            results = await transport.cycle(commands)

            line_parts = []

            for result in results:
                sid = result.arbitration_id

                pos = get_value(result, moteus.Register.POSITION)
                vel = get_value(result, moteus.Register.VELOCITY)
                tq = get_value(result, moteus.Register.TORQUE)
                voltage = get_value(result, moteus.Register.VOLTAGE)
                temp = get_value(result, moteus.Register.TEMPERATURE)
                fault = get_value(result, moteus.Register.FAULT)
                mode = get_value(result, moteus.Register.MODE)

                line_parts.append(
                    f"ID={sid} pos={pos:.3f} vel={vel:.3f} tq={tq:.3f} fault={fault}"
                )

                log_rows.append({
                    "t": t,
                    "id": sid,
                    "mode": mode,
                    "fault": fault,
                    "position": pos,
                    "velocity": vel,
                    "torque": tq,
                    "voltage": voltage,
                    "temperature": temp,
                })

            print(" | ".join(line_parts))

            await asyncio.sleep(dt)

    finally:
        print("\nStopping all controllers...")
        await transport.cycle([servo.make_stop() for servo in servos])

    if log_rows:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=log_rows[0].keys())
            writer.writeheader()
            writer.writerows(log_rows)

        print(f"\nSaved log: {args.csv}")


if __name__ == "__main__":
    asyncio.run(main())