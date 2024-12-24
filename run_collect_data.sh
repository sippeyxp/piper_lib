#!/bin/bash
bash ./can_activate.sh
python3 collect_data.py --episode_log_dir /host/tmp/episode_log --config_file configs/home_config.json --task red-fish-into-bowl
