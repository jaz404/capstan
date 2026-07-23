#!/usr/bin/env python3

import asyncio
import moteus

MOTOR_ID = 1

MIN_POS = -1.0
MAX_POS = 1.6

DT = 0.01

SPEED = 2   # 👈 main speed control (increase for faster)
MAX_TORQUE = 0.18


async def main():
    c = moteus.Controller(id=MOTOR_ID)

    target = 0.0
    direction = 1

    try:
        while True:
            state = await c.query()
            pos = state.values[moteus.Register.POSITION]

            print(f"pos={pos:.3f} target={target:.3f}")

            # safety bounds
            if pos < MIN_POS - 0.2 or pos > MAX_POS + 0.2:
                print("SAFETY STOP")
                break

            # continuous trajectory (KEY FIX)
            target += direction * SPEED * DT

            if target >= MAX_POS:
                target = MAX_POS
                direction = -1
            elif target <= MIN_POS:
                target = MIN_POS
                direction = 1

            await c.set_position(
                position=target,
                velocity=SPEED,          # 👈 controls smoothness + speed
                maximum_torque=MAX_TORQUE,
                watchdog_timeout=0.1,
            )

            await asyncio.sleep(DT)

    finally:
        print("Stopping motor...")
        await c.set_stop()


asyncio.run(main())