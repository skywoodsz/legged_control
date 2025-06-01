## Additional feature 

### 1. Foot forces observer

*reference:* Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains

*Output topic:* `/LFLeg_force /LHLeg_force /RFLeg_force /RHLeg_force`

### 2. State predeiction by the centrodial dynamics in body frame

*Output topic:* `/legged_robot_mpc_prediction`
data:
- state: predicted base velocity. The order is vx, vy, vz, wx, wy, wz.
- input predicted base acc. The order is acc_x, acc_y, acc_z, acc_wx, acc_wy, acc_wz.

### 3. IMU bias 
Adjusting the zero bias of the IMU Euler angle in `legged_controllers/config/imu.yaml`
