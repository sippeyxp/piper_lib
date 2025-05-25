#!/bin/bash
bash ./can_activate.sh can0 1000000 $(./usb_address_from_iserial.sh 002E00495246571120393733) # left
bash ./can_activate.sh can1 1000000 $(./usb_address_from_iserial.sh 0040002B5746570D20323337) # right
# python3 collect_data.py --episode_log_dir /host/tmp/episode_log --config_file configs/home_config.json --task red-fish-into-bowl
python3 collect_data.py --episode_log_dir ${HOME}/work/episode_log --config_file configs/p620_collect_biarm_3cams.json \
    --task test_biarm_3cams
