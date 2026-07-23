#!/usr/bin/env python3

import asyncio
import math
import time

import moteus


# Motor configuration

MOTEUS_ID_1 = 3
MOTEUS_ID_2 = 4

SEARCH_VELOCITY = 1.0       # motor rev/s
MOVE_VELOCITY_LIMIT = 1.0  # motor rev/s
MOVE_ACCEL_LIMIT = 1.0      # motor rev/s^2

MAX_TORQUE = 0.05           # Nm
TORQUE_THRESHOLD = 0.015    # Nm
VELOCITY_THRESHOLD = 0.05   # motor rev/s

STALL_TIME = 0.25
SEARCH_TIMEOUT = 15.0
MAX_TRAVEL = 4.0
RATE_HZ = 100.0

POSITION_TOLERANCE = 0.002  # motor revolutions


# Five-bar geometry

C = 98.192  # mm
D = 110.0   # mm

X_TARGET = 0.0
Y_TARGET = 100.0

X_TARGET_2 = 0.0
Y_TARGET_2 = 200.0

X_TARGET_3 = 80.0
Y_TARGET_3 = 180.0

X_TARGET_4 = -80.0
Y_TARGET_4 = 180.0


# Motor revolutions per joint revolution.

MOTOR_1_GEAR_RATIO = 8.55
MOTOR_2_GEAR_RATIO = 8.55


MOTOR_1_SIGN = +1.0
MOTOR_2_SIGN = -1.0


# IK angle represented when each motor is at its calibrated center.

JOINT_1_CENTER_ANGLE = math.radians(30.75)
JOINT_2_CENTER_ANGLE = math.radians(30.75)


def get(state, register):
    return state.values.get(register, math.nan)


def inverse_kinematics(x, y):
    radius = math.hypot(x, y)

    if radius == 0.0:
        raise ValueError("Target cannot be at the IK origin")

    cos_elbow = (
        x * x + y * y + C * C - D * D
    ) / (
        2.0 * C * radius
    )

    if cos_elbow < -1.0 or cos_elbow > 1.0:
        raise ValueError(
            f"Target ({x:.2f}, {y:.2f}) mm is outside the workspace"
        )

    # Protect against tiny floating-point errors.
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    elbow_angle = math.acos(cos_elbow)
    target_angle = math.atan2(x, y)

    q1 = elbow_angle + target_angle
    q2 = elbow_angle - target_angle

    return q1, q2


def joint_angle_to_motor_position(
    joint_angle,
    center_joint_angle,
    motor_center,
    gear_ratio,
    motor_sign,
):
    joint_displacement = joint_angle - center_joint_angle

    joint_revolutions = joint_displacement / (2.0 * math.pi)
    motor_displacement = motor_sign * gear_ratio * joint_revolutions

    return motor_center + motor_displacement


async def find_bound(controller, direction):
    state = await controller.set_position(
        position=math.nan,
        query=True,
    )

    start_position = get(state, moteus.Register.POSITION)

    if not math.isfinite(start_position):
        raise RuntimeError("Could not read motor position")

    start_time = time.monotonic()
    stall_start = None

    while True:
        now = time.monotonic()

        if now - start_time > SEARCH_TIMEOUT:
            raise RuntimeError("Bound search timed out")

        state = await controller.set_position(
            position=math.nan,
            velocity=direction * SEARCH_VELOCITY,
            maximum_torque=MAX_TORQUE,
            query=True,
        )

        position = get(state, moteus.Register.POSITION)
        velocity = get(state, moteus.Register.VELOCITY)
        torque = get(state, moteus.Register.TORQUE)
        fault = get(state, moteus.Register.FAULT)

        if fault != 0:
            raise RuntimeError(f"Moteus fault: {fault}")

        if not all(map(math.isfinite, [position, velocity, torque])):
            raise RuntimeError("Invalid telemetry received")

        if abs(position - start_position) > MAX_TRAVEL:
            raise RuntimeError("Maximum calibration travel exceeded")

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
            f"\rpos={position:+.4f} rev  "
            f"vel={velocity:+.4f} rev/s  "
            f"torque={torque:+.3f} Nm",
            end="",
            flush=True,
        )

        await asyncio.sleep(1.0 / RATE_HZ)


async def move_to_position(controller, target):
    while True:
        state = await controller.set_position(
            position=target,
            velocity=0.0,
            velocity_limit=MOVE_VELOCITY_LIMIT,
            accel_limit=MOVE_ACCEL_LIMIT,
            maximum_torque=MAX_TORQUE,
            query=True,
        )

        position = get(state, moteus.Register.POSITION)
        fault = get(state, moteus.Register.FAULT)

        if fault != 0:
            raise RuntimeError(f"Moteus fault while moving: {fault}")

        if abs(position - target) < POSITION_TOLERANCE:
            return

        await asyncio.sleep(1.0 / RATE_HZ)

async def move_to_position_tq(controller, target):
    while True:
        state = await controller.set_position(
            position=target,
            velocity=0.0,
            velocity_limit=MOVE_VELOCITY_LIMIT,
            accel_limit=MOVE_ACCEL_LIMIT,
            maximum_torque=0.1,
            query=True,
        )

        position = get(state, moteus.Register.POSITION)
        fault = get(state, moteus.Register.FAULT)

        if fault != 0:
            raise RuntimeError(f"Moteus fault while moving: {fault}")

        if abs(position - target) < POSITION_TOLERANCE:
            return

        await asyncio.sleep(1.0 / RATE_HZ)


async def calibrate_motor(controller, name):
    print(f"\n{name}: searching positive bound...")
    positive_bound = await find_bound(controller, +1)

    await controller.set_stop()
    await asyncio.sleep(0.5)

    print(f"\n{name}: searching negative bound...")
    negative_bound = await find_bound(controller, -1)

    await controller.set_stop()
    await asyncio.sleep(0.5)

    lower = min(negative_bound, positive_bound)
    upper = max(negative_bound, positive_bound)
    center = (lower + upper) / 2.0

    print(f"\n{name} lower:  {lower:.6f} rev")
    print(f"{name} upper:  {upper:.6f} rev")
    print(f"{name} center: {center:.6f} rev")

    print(f"{name}: moving to center...")
    await move_to_position(controller, center)

    return lower, upper, center


async def move_both(controller_1, controller_2, target_1, target_2):
    while True:
        state_1, state_2 = await asyncio.gather(
            controller_1.set_position(
                position=target_1,
                velocity=0.0,
                velocity_limit=MOVE_VELOCITY_LIMIT,
                accel_limit=MOVE_ACCEL_LIMIT,
                maximum_torque=MAX_TORQUE,
                query=True,
            ),
            controller_2.set_position(
                position=target_2,
                velocity=0.0,
                velocity_limit=MOVE_VELOCITY_LIMIT,
                accel_limit=MOVE_ACCEL_LIMIT,
                maximum_torque=MAX_TORQUE,
                query=True,
            ),
        )

        position_1 = get(state_1, moteus.Register.POSITION)
        position_2 = get(state_2, moteus.Register.POSITION)

        fault_1 = get(state_1, moteus.Register.FAULT)
        fault_2 = get(state_2, moteus.Register.FAULT)

        if fault_1 != 0 or fault_2 != 0:
            raise RuntimeError(
                f"Moteus faults: motor 1={fault_1}, motor 2={fault_2}"
            )

        error_1 = abs(position_1 - target_1)
        error_2 = abs(position_2 - target_2)

        print(
            f"\rM1={position_1:+.4f}/{target_1:+.4f}  "
            f"M2={position_2:+.4f}/{target_2:+.4f}",
            end="",
            flush=True,
        )

        if (
            error_1 < POSITION_TOLERANCE
            and error_2 < POSITION_TOLERANCE
        ):
            print()
            return

        await asyncio.sleep(1.0 / RATE_HZ)


async def main():
    motor_1 = moteus.Controller(id=MOTEUS_ID_1)
    motor_2 = moteus.Controller(id=MOTEUS_ID_2)

    try:
        _, _, center_1 = await calibrate_motor(motor_1, "Motor 1")
        _, _, center_2 = await calibrate_motor(motor_2, "Motor 2")

        q1, q2 = inverse_kinematics(X_TARGET, Y_TARGET)

        print("\nInverse kinematics:")
        print(f"q1 = {math.degrees(q1):.3f} deg")
        print(f"q2 = {math.degrees(q2):.3f} deg")

        target_1 = joint_angle_to_motor_position(
            joint_angle=q1,
            center_joint_angle=JOINT_1_CENTER_ANGLE,
            motor_center=center_1,
            gear_ratio=MOTOR_1_GEAR_RATIO,
            motor_sign=MOTOR_1_SIGN,
        )

        target_2 = joint_angle_to_motor_position(
            joint_angle=q2,
            center_joint_angle=JOINT_2_CENTER_ANGLE,
            motor_center=center_2,
            gear_ratio=MOTOR_2_GEAR_RATIO,
            motor_sign=MOTOR_2_SIGN,
        )

        print("\nMotor commands:")
        print(f"Motor 1 target: {target_1:.6f} rev")
        print(f"Motor 2 target: {target_2:.6f} rev")

        print("\nMoving Motor 1 to IK target...")
        await motor_2.set_stop()
        await asyncio.sleep(0.1)

        await motor_1.set_stop()
        await asyncio.sleep(0.1)

        await move_to_position_tq(
            motor_1,
            target_1
        )

        print("\nMoving Motor 2 to IK target...")
        await move_to_position(
            motor_2,
            target_2
        )

        print("\nTarget reached.")


        q1, q2 = inverse_kinematics(X_TARGET_2, Y_TARGET_2)

        print("\nInverse kinematics:")
        print(f"q1 = {math.degrees(q1):.3f} deg")
        print(f"q2 = {math.degrees(q2):.3f} deg")

        target_1 = joint_angle_to_motor_position(
            joint_angle=q1,
            center_joint_angle=JOINT_1_CENTER_ANGLE,
            motor_center=center_1,
            gear_ratio=MOTOR_1_GEAR_RATIO,
            motor_sign=MOTOR_1_SIGN,
        )

        target_2 = joint_angle_to_motor_position(
            joint_angle=q2,
            center_joint_angle=JOINT_2_CENTER_ANGLE,
            motor_center=center_2,
            gear_ratio=MOTOR_2_GEAR_RATIO,
            motor_sign=MOTOR_2_SIGN,
        )

        print("\nMotor commands:")
        print(f"Motor 1 target: {target_1:.6f} rev")
        print(f"Motor 2 target: {target_2:.6f} rev")

        print("\nMoving Motor 1 to IK target...")
        await motor_2.set_stop()
        await asyncio.sleep(0.1)

        await motor_1.set_stop()
        await asyncio.sleep(0.1)

        await move_to_position_tq(
            motor_1,
            target_1
        )

        print("\nMoving Motor 2 to IK target...")
        await move_to_position(
            motor_2,
            target_2
        )

        print("\nTarget reached.")

        q1, q2 = inverse_kinematics(X_TARGET_3, Y_TARGET_3)

        print("\nInverse kinematics:")
        print(f"q1 = {math.degrees(q1):.3f} deg")
        print(f"q2 = {math.degrees(q2):.3f} deg")

        target_1 = joint_angle_to_motor_position(
            joint_angle=q1,
            center_joint_angle=JOINT_1_CENTER_ANGLE,
            motor_center=center_1,
            gear_ratio=MOTOR_1_GEAR_RATIO,
            motor_sign=MOTOR_1_SIGN,
        )

        target_2 = joint_angle_to_motor_position(
            joint_angle=q2,
            center_joint_angle=JOINT_2_CENTER_ANGLE,
            motor_center=center_2,
            gear_ratio=MOTOR_2_GEAR_RATIO,
            motor_sign=MOTOR_2_SIGN,
        )

        print("\nMotor commands:")
        print(f"Motor 1 target: {target_1:.6f} rev")
        print(f"Motor 2 target: {target_2:.6f} rev")

        print("\nMoving Motor 1 to IK target...")
        await motor_2.set_stop()
        await asyncio.sleep(0.1)

        await motor_1.set_stop()
        await asyncio.sleep(0.1)

        await move_to_position_tq(
            motor_1,
            target_1
        )

        print("\nMoving Motor 2 to IK target...")
        await move_to_position(
            motor_2,
            target_2
        )

        print("\nTarget reached.")

        q1, q2 = inverse_kinematics(X_TARGET_4, Y_TARGET_4)

        print("\nInverse kinematics:")
        print(f"q1 = {math.degrees(q1):.3f} deg")
        print(f"q2 = {math.degrees(q2):.3f} deg")

        target_1 = joint_angle_to_motor_position(
            joint_angle=q1,
            center_joint_angle=JOINT_1_CENTER_ANGLE,
            motor_center=center_1,
            gear_ratio=MOTOR_1_GEAR_RATIO,
            motor_sign=MOTOR_1_SIGN,
        )

        target_2 = joint_angle_to_motor_position(
            joint_angle=q2,
            center_joint_angle=JOINT_2_CENTER_ANGLE,
            motor_center=center_2,
            gear_ratio=MOTOR_2_GEAR_RATIO,
            motor_sign=MOTOR_2_SIGN,
        )

        print("\nMotor commands:")
        print(f"Motor 1 target: {target_1:.6f} rev")
        print(f"Motor 2 target: {target_2:.6f} rev")

        print("\nMoving Motor 1 to IK target...")
        await motor_2.set_stop()
        await asyncio.sleep(0.1)

        await motor_1.set_stop()
        await asyncio.sleep(0.1)

        await move_to_position_tq(
            motor_1,
            target_1
        )

        print("\nMoving Motor 2 to IK target...")
        await move_to_position(
            motor_2,
            target_2
        )

        print("\nTarget reached.")


        while True:
            await asyncio.gather(
                motor_1.set_position(
                    position=target_1,
                    velocity=0.0,
                    velocity_limit=MOVE_VELOCITY_LIMIT,
                    accel_limit=MOVE_ACCEL_LIMIT,
                    maximum_torque=MAX_TORQUE,
                ),
                motor_2.set_position(
                    position=target_2,
                    velocity=0.0,
                    velocity_limit=MOVE_VELOCITY_LIMIT,
                    accel_limit=MOVE_ACCEL_LIMIT,
                    maximum_torque=MAX_TORQUE,
                ),
            )

            await asyncio.sleep(1.0 / RATE_HZ)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        await asyncio.gather(
            motor_1.set_stop(),
            motor_2.set_stop(),
        )


if __name__ == "__main__":
    asyncio.run(main())