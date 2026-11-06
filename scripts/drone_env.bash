# Helpers of the ROS2_ardupilot container (sourced by ~/.bashrc)
#
# Every simulated drone runs in its own ROS domain (ROS_DOMAIN_ID = drone id),
# the ground station (GCS) in domain 0. A terminal only "sees" its domain:
#   drone_domain 2     -> this terminal works inside drone2 (all its topics: MAVROS, camera, TF...)
#   gcs_domain         -> back to the GCS domain (only what the Zenoh bridges share)

drone_domain() {
    export ROS_DOMAIN_ID="${1:-0}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

gcs_domain() {
    drone_domain 0
}

# QGroundControl (runs as the non-root user "ubuntu": QGC refuses to run as root)
alias qgc='qgroundcontrol > /dev/null 2>&1 &'

# MAVProxy on the extra GCS port of a drone: mavlink_console <id>
mavlink_console() {
    local id="${1:-1}"
    mavproxy.py --master "tcp:127.0.0.1:$((5762 + 10 * (id - 1)))" --console
}

# Show the ROS domain in the prompt
if [[ "${PS1}" != *ROS_DOMAIN_ID* ]]; then
    PS1='\[\e[1;33m\][dom ${ROS_DOMAIN_ID:-0}]\[\e[0m\] '"${PS1}"
fi
