import subprocess
import time

import os
import json
import numpy as np



for i in np.arange(0.7, 0.8, 0.01):
    config = {

        "velocity": i,
        "lookback_length": 5.0,
        "t_step": 0.1
    }
    print(config)

    with open('robot_config.json', 'w') as f:
        f.write(json.dumps(config, indent=4))


    sim_run = subprocess.run(f"bash test_simulation.sh", shell=True, check=True, stderr=subprocess.DEVNULL)
    time.sleep(1)  # Wait for a second before the next iteration

    with open("results.txt", "r") as f:
            print(f.read())
