#!/bin/bash
bash ./can_activate.sh
# python3 collect_data.py --episode_log_dir /host/tmp/episode_log --config_file configs/home_config.json --task red-fish-into-bowl
python3 collect_data.py --episode_log_dir ${HOME}/work/episode_log --config_file configs/p620_collect_3cams.json \
    --task red-fish-into-bowl-v3
