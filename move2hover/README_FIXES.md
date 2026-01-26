# Move2Hover - Fixed and Working

## Summary of Fixes

The `move2hover.py` file has been updated to properly control a simulated drone via MAVROS topics and services.

### Key Changes

1. **Added MAVROS Support**
   - Added PoseStamped publisher to `/mavros/setpoint_position/local`
   - Added MAVROS service clients for `/mavros/cmd/arming` and `/mavros/set_mode`
   - Implemented OFFBOARD mode handshake

2. **Fixed QoS Issues**
   - Used `qos_profile_sensor_data` for IMU and odometry subscriptions
   - Added proper QoS profiles to match publisher settings
   - Fixed topic names to match simulation (e.g., `/sim1/imu/data`)

3. **State Machine**
   - PUBLISH_SP: Publishes initial setpoints for ~1 second
   - SET_OFFBOARD: Requests OFFBOARD mode
   - ARM: Requests arming
   - MOVE_TO_GOAL: Moves to target position
   - HOVER: Maintains position at goal

4. **Increased Control Rate**
   - Changed from 10 Hz to 20 Hz (50ms timer)
   - Required for reliable OFFBOARD setpoint streaming

## How to Use

### Configuration

Edit the goal position in the code:
```python
self.goal_x = 5.0     # target east position (meters)
self.goal_y = -3.0    # target north position (meters)
self.goal_z = 2.5     # target altitude (meters)
```

### Running

1. Start your SITL simulation and MAVROS
2. Run the node:
```bash
python3 /home/dfec/master_src/move2hover/move2hover/move2hover.py
```

3. Enable control by publishing to the `ctrl_relinq` topic:
```bash
ros2 topic pub /ctrl_relinq std_msgs/msg/Bool "{data: true}" -1
```

### Expected Behavior

Once control is enabled:
1. Node publishes current position setpoints for ~1 second
2. Requests OFFBOARD mode via MAVROS
3. Requests arming via MAVROS
4. Begins moving toward goal position
5. Hovers when within 0.3m of goal

### Monitoring

Watch the setpoint stream:
```bash
ros2 topic echo /mavros/setpoint_position/local
```

Check IMU data is received:
```bash
ros2 topic echo /sim1/imu/data -n 1
```

## Topic Configuration

Current topic subscriptions:
- `/smaug2/odometry/out` - Odometry (may need to update to `/sim1/local_position/pose`)
- `/sim1/imu/data` - IMU data
- `ctrl_relinq` - Control enable/disable

Current publishers:
- `/mavros/setpoint_position/local` - Position setpoints for MAVROS
- `/smaug2/setpoint_attitude/cmd_vel` - Legacy velocity commands

## Requirements

- ROS2 (Humble)
- mavros_msgs
- SITL with MAVROS running
- tf_transformations

## Known Limitations

1. **Service Response Checking**: Currently doesn't wait for SetMode/CommandBool service responses
2. **No Retries**: If arming/mode setting fails, no automatic retry
3. **Fixed Goal**: Goal position is hardcoded, not parameterized
4. **Topic Names**: May need adjustment based on your specific simulation setup

## Recommended Next Steps

1. **Add Service Response Checking**:
   ```python
   future = self.mode_client.call_async(req)
   rclpy.spin_until_future_complete(self, future)
   if future.result().mode_sent:
       # Success
   ```

2. **Parameterize Goal Position**:
   ```python
   self.declare_parameter('goal_x', 5.0)
   self.goal_x = self.get_parameter('goal_x').value
   ```

3. **Make Topic Names Parameters**:
   ```python
   self.declare_parameter('odom_topic', '/sim1/local_position/pose')
   ```

4. **Add Safety Timeouts**:
   - Timeout if OFFBOARD not accepted
   - Timeout if arming fails
   - Emergency land if position diverges

## Troubleshooting

### Node Starts But Doesn't Move
- Check that `ctrl_relinq` topic received True
- Verify MAVROS is connected: `ros2 topic echo /mavros/state`
- Check pre-arm conditions in simulator

### QoS Warnings
- Should be fixed with current code
- If warnings persist, verify publisher QoS settings

### No IMU/Odometry Data
- Verify topics exist: `ros2 topic list | grep imu`
- Check QoS compatibility
- Ensure simulation is running
