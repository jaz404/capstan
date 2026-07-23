#!/usr/bin/env python3

import asyncio
import math
import time

import moteus


MOTEUS_ID_1 = 3
MOTEUS_ID_2 = 4

SEARCH_VELOCITY = 1    # rev/s
MAX_TORQUE = 0.05           # Nm
TORQUE_THRESHOLD = 0.015    # Nm
VELOCITY_THRESHOLD = 0.05  # rev/s

STALL_TIME = 0.25
SEARCH_TIMEOUT = 15.0
MAX_TRAVEL = 4.0
RATE_HZ = 100.0


def get(state, register):
    return state.values.get(register, math.nan)


async def find_bound(controller, direction):
    state = await controller.set_position(position=math.nan, query=True)
    start_position = get(state, moteus.Register.POSITION)

    velocity_command = direction * SEARCH_VELOCITY
    start_time = time.monotonic()
    stall_start = None

    while True:
        now = time.monotonic()

        if now - start_time > SEARCH_TIMEOUT:
            raise RuntimeError("Search timed out")

        state = await controller.set_position(
            position=math.nan,
            velocity=velocity_command,
            maximum_torque=MAX_TORQUE,
            query=True,
        )

        position = get(state, moteus.Register.POSITION)
        velocity = get(state, moteus.Register.VELOCITY)
        torque = get(state, moteus.Register.TORQUE)
        fault = get(state, moteus.Register.FAULT)

        if fault != 0:
            raise RuntimeError(f"Moteus fault: {fault}")

        if abs(position - start_position) > MAX_TRAVEL:
            raise RuntimeError("Maximum travel exceeded")

        stalled = (
            abs(velocity) < VELOCITY_THRESHOLD
            and abs(torque) > TORQUE_THRESHOLD
        )

        if stalled:
            if stall_start is None:
                stall_start = now
            elif now - stall_start >= STALL_TIME:
                return position
        else:
            stall_start = None

        print(
            f"\rpos={position:+.4f}  "
            f"vel={velocity:+.4f}  "
            f"torque={torque:+.3f}",
            end="",
        )

        await asyncio.sleep(1.0 / RATE_HZ)


async def main():
    controller = moteus.Controller(id=MOTEUS_ID_1)

    try:
        print("Searching positive bound...")
        positive_bound = await find_bound(controller, +1)

        await controller.set_stop()
        await asyncio.sleep(0.5)

        print("\nSearching negative bound...")
        negative_bound = await find_bound(controller, -1)

        lower = min(negative_bound, positive_bound)
        upper = max(negative_bound, positive_bound)

        print(f"\n\nLower bound: {lower:.6f} rev")
        print(f"Upper bound: {upper:.6f} rev")

        center = (lower + upper) / 2.0

        print(f"Center:      {(lower + upper) / 2:.6f} rev")

        # go to the center
        while True:
            state = await controller.set_position(
                position=center,
                velocity=0.0,
                maximum_torque=MAX_TORQUE,
                query=True,
            )

            pos = get(state, moteus.Register.POSITION)

            if abs(pos - center) < 0.002:      # ~0.7°
                break

            await asyncio.sleep(1.0 / RATE_HZ)

        await controller.set_stop()

    finally:
        await controller.set_stop()



    controller = moteus.Controller(id=MOTEUS_ID_2)

    try:
        print("Searching positive bound...")
        positive_bound = await find_bound(controller, +1)

        await controller.set_stop()
        await asyncio.sleep(0.5)

        print("\nSearching negative bound...")
        negative_bound = await find_bound(controller, -1)

        lower = min(negative_bound, positive_bound)
        upper = max(negative_bound, positive_bound)

        print(f"\n\nLower bound: {lower:.6f} rev")
        print(f"Upper bound: {upper:.6f} rev")

        center = (lower + upper) / 2.0

        print(f"Center:      {(lower + upper) / 2:.6f} rev")

        # go to the center
        while True:
            state = await controller.set_position(
                position=center,
                velocity=0.0,
                maximum_torque=MAX_TORQUE,
                query=True,
            )

            pos = get(state, moteus.Register.POSITION)

            if abs(pos - center) < 0.002:      # ~0.7°
                break

            await asyncio.sleep(1.0 / RATE_HZ)

        await controller.set_stop()

    finally:
        await controller.set_stop()


asyncio.run(main())