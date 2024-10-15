A python program that bridges the gap between fantasy and reality by animating a physical chandelier.

### Usage

Flags:

```bash
python main.py -e -m -c
```

- `-e` or `--environment` to specify the environment (development, testing, production)
- `-m` or `--mode` to specify the mode (normal, manual, scripts)
  - `normal` mode is the default mode and requires physical interaction
  - `manual` mode is for testing purposes and disables physical interaction and enables state switching via the console
  - `scripts` mode is for running scripts that interact with the chandelier
- `-c` or `--calibration` to specify whether to edit the default behavior of the calibration process
  - `default` calibration simply calibrates all motors that aren't already calibrated
  - `prompt` calibration allows the user to specify which motors to recalibrate or recalibrate all motors

Default flags:

```bash
python main.py -e=development -m=normal -c=default
```
