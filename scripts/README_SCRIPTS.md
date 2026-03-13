# About
Scripts are little demos or tests that we don't want to clog up the root directory with.

# Running
To run, use `python -m scripts.lidar_terminal` for the file `lidar_terminal.py`.

This allows `from lidar.api import Lidar` to still access the `lidar` module to work as if
the `lidar_terminal.py` is in the root directory. Note the lack of `.py` in the command.