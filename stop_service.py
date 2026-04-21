"""stops the headless service which boots up on startup"""

import os

if __name__ == "__main__":
    print("Stopping Pathfinder headless service ('sudo systemctl stop pathfinder.service')")
    os.system("sudo systemctl stop pathfinder.service")
