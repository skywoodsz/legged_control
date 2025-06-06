## 1. Additional feature 
- Foot forces observer: *reference: Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains*.
- State predeiction by the centrodial dynamics in body frame.
- IMU bias: Adjusting the zero bias of the IMU Euler angle in `legged_controllers/config/imu.yaml`.

### 2. Topic and data structure
#### 2.1 : Foot forces observer
*Output topic:* `/dog/leg_state` with `cheetah_msgs/LegsState.msg`
- bfoot_pos: foot position in body frame
- bfoot_vel: foot velocity in body frame
- foot_contact: foot contact flag
- foot_force_prediction: foot force predicted by mpc in body frame
- foot_force_observation: foot force predicted by observer in body frame

#### 2.2 : State predeiction by the centrodial dynamics in body frame 
*Output topic:* `/legged_robot_mpc_prediction` with `cheetah_msgs/RobotDynamicState.msg` 

- base_linear_velocity: predicted base linear velocity, with the order of vx, vy, vz.
- base_angular_velocity: predicted base angular velocity, with the order of wx, wy, wz.
- base_linear_acceleration: predicted base linear acceleration, with the order of acc_x, acc_y, acc_z.
- base_angular_acceleration: predicted base angular acceleration, with the order of acc_wx, acc_wy, acc_wz.


