# Moteus Motor Test Utility

[https://mjbots.github.io/moteus/reference/](https://mjbots.github.io/moteus/reference/)

Python-based test and diagnostics utility for the mjbots `moteus` motor controller. 

## Requirements

Install dependencies:

```bash
pip install -r requirements.txt
```

## Moteus Python API 

Main controller object:

```python
c = moteus.Controller(id=1)
```

Primary control interface:

```python
await c.set_position(...)
```

This command is used for:
- Position control
- Velocity control
- Torque/feedforward control
- Query-only operations

Example:

```python
await c.set_position(
    position=1.0,
    velocity=0.0,
    maximum_torque=0.5,
    query=True,
)
```

Useful parameters:

| Parameter | Description |
|---|---|
| `position` | Target position in revolutions |
| `velocity` | Target velocity in rev/s |
| `maximum_torque` | Torque limit in Nm |
| `velocity_limit` | Maximum allowed velocity |
| `accel_limit` | Maximum allowed acceleration |
| `feedforward_torque` | Open-loop torque feedforward |
| `query` | Returns telemetry state |

Telemetry is accessed through:

```python
state.values[moteus.Register.POSITION]
```

Common registers:
- `POSITION`
- `VELOCITY`
- `TORQUE`
- `Q_CURRENT`
- `D_CURRENT`
- `VOLTAGE`
- `TEMPERATURE`
- `FAULT`

All calls made in this script are asynchronous. To make synchronous calls, refer to https://github.com/mjbots/moteus/blob/main/lib/python/examples/synchronized_movement.py.

## Usage

### Calibration

If you started from a bare moteus board, you will need to calibrate it for the attached motor before any control modes are possible. Calibration is required when:

- Using a new bare moteus controller for the first time
- Changing to a different motor
- Altering the mechanical registration between the controller and motor
- After certain configuration changes that affect motor characteristics

```bash
python3 -m moteus.moteus_tool --target 1 --calibrate
```
**Important:** Allow the motor to be able to spin freely during calibration.

### Query Controller State

```bash
python3 moteus_test.py --id 1 --mode query
```

Reads:
- Position
- Velocity
- Torque
- Voltage
- Temperature
- Fault state


### Hold Current Position

```bash
python3 moteus_test.py \
  --id 1 \
  --mode hold \
  --duration 5 \
  --max-torque 0.5
```


### Position Step Test

```bash
python3 moteus_test.py \
  --id 1 \
  --mode step \
  --step-rev 0.25 \
  --duration 4 \
  --max-torque 0.8 \
  --velocity-limit 5.0 \
  --accel-limit 20.0
```

### Velocity Test

```bash
python3 moteus_test.py \
  --id 1 \
  --mode velocity \
  --velocity 3.0 \
  --duration 5 \
  --max-torque 0.8
```

Velocity units are:

```text
rev/s
```

Examples:

| Velocity | RPM |
|---|---|
| 1.0 | 60 RPM |
| 5.0 | 300 RPM |
| 10.0 | 600 RPM |

### Torque Test

```bash
python3 moteus_test.py \
  --id 1 \
  --mode torque \
  --torque 0.2 \
  --duration 3
```


## Logging and Plotting

Telemetry is automatically saved to CSV:

```text
moteus_log.csv
```

Generated plots:
- Position
- Velocity
- Torque
- Voltage
- Temperature

Re-plot existing logs:

```bash
python3 moteus_test.py --mode plot --csv moteus_log.csv
```


## Useful Diagnostics

Read controller statistics:

```bash
moteus_tool -t 1 --read servo_stats
```

### Read encoder position

```bash
moteus_tool -t 1 --read motor_position
```

### Read DRV8323 status

```bash
moteus_tool -t 1 --read drv8323
```
Sample output: 
```bash
{
  "fault": false,
  "vds_ocp": false,
  "gdf": false,
  "uvlo": false,
  "otsd": false,
  "vds_ha": false,
  "vds_la": false,
  "vds_hb": false,
  "vds_lb": false,
  "vds_hc": false,
  "vds_lc": false,
  "fsr1": 0,
  "sa_oc": false,
  "sb_oc": false,
  "sc_oc": false,
  "otw": false,
  "cpuv": false,
  "vgs_ha": false,
  "vgs_la": false,
  "vgs_hb": false,
  "vgs_lb": false,
  "vgs_hc": false,
  "vgs_lc": false,
  "fsr2": 0,
  "fault_line": false,
  "power": false,
  "enabled": false,
  "fault_config": 0,
  "config_count": 10,
  "status_count": 3866
}
```