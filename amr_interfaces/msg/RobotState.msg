# Operational state of one AMR, published at >= 1 Hz on /<robot_id>/robot_state.
# The fleet manager also uses this message as the robot's heartbeat.
#
# State values. Keep in sync with amr_fleet_manager/fleet_core.py (RobotStateCode).
uint8 UNKNOWN=0
uint8 IDLE=1            # parked in a ready-queue slot, no task, ready for work
uint8 TO_GTP=2          # driving from the queue to a GTP dock (includes precision docking)
uint8 DOCKED_AT_GTP=3   # aligned beside the GTP station, waiting for the operator
uint8 LOADING=4         # the operator is loading items onto the cargo deck
uint8 IN_TRANSIT=5      # driving loaded from the GTP station to a drop-off
uint8 UNLOADING=6       # at the drop-off, waiting for the worker to unload
uint8 RETURNING=7       # driving empty back to a ready-queue slot
uint8 ERROR=8           # needs human attention, see error_message

std_msgs/Header header      # header.frame_id is the frame of `pose` (normally "map")
string robot_id
uint8 state
string task_id              # empty when no task is running
geometry_msgs/Pose pose     # current pose estimate (AMCL / SLAM)
float32 battery_percent     # 0..100
string error_message
