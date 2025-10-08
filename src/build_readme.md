1. install deps
`dpkg -l | grep libmodbus`
`apt search libmodbus`
`sudo apt update && sudo apt install -y libmodbus-dev libmodbus5`
2. build msgs
`catkin_make --pkg cruise_msgs`
`catkin_make --pkg ranger_msgs`
