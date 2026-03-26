import os, re

base = '/home/dell/smart_traffic_ws/src/smart_intersection/models'

# Updated wheel dimensions for heavy vehicles! Radius 0.25 for bus/truck
vehicles = {
    'car': {'x_f': 0.65, 'x_r': -0.65, 'y': 0.6, 'z': 0.15, 'r_t': 0.15, 'l_t': 0.10, 'r_r': 0.09, 'l_r': 0.11, 'lift': 0.0},
    'bike': {'x_f': 0.45, 'x_r': -0.45, 'y': 0.1, 'z': 0.15, 'r_t': 0.15, 'l_t': 0.05, 'r_r': 0.09, 'l_r': 0.06, 'lift': 0.0},
    'police': {'x_f': 0.7, 'x_r': -0.7, 'y': 0.6, 'z': 0.15, 'r_t': 0.15, 'l_t': 0.10, 'r_r': 0.09, 'l_r': 0.11, 'lift': 0.0},
    'ambulance': {'x_f': 0.8, 'x_r': -0.8, 'y': 0.6, 'z': 0.15, 'r_t': 0.15, 'l_t': 0.10, 'r_r': 0.09, 'l_r': 0.11, 'lift': 0.0},
    'bus': {'x_f': 1.7, 'x_r': -1.7, 'y': 0.7, 'z': 0.25, 'r_t': 0.25, 'l_t': 0.15, 'r_r': 0.15, 'l_r': 0.16, 'lift': 0.10},
    'fire_engine': {'x_f': 1.7, 'x_r': -1.7, 'y': 0.7, 'z': 0.25, 'r_t': 0.25, 'l_t': 0.15, 'r_r': 0.15, 'l_r': 0.16, 'lift': 0.10},
    'truck': {'x_f': 1.9, 'x_r': -1.9, 'y': 0.7, 'z': 0.25, 'r_t': 0.25, 'l_t': 0.15, 'r_r': 0.15, 'l_r': 0.16, 'lift': 0.10},
}

for v, d in vehicles.items():
    path = f"{base}/{v}/model.sdf"
    with open(path, 'r') as f:
        content = f.read()

    # Extract chassis ONLY
    chassis_match = re.search(r'(<link name="chassis">.*?</link>)', content, flags=re.DOTALL)
    chassis = chassis_match.group(1) if chassis_match else ""
    
    # Lift chassis if required (for larger wheels)
    if d['lift'] > 0:
        def lift_pose(m):
            parts = m.group(1).split()
            z = float(parts[2]) + d['lift']
            return f"<pose>{parts[0]} {parts[1]} {z:.2f} {parts[3]} {parts[4]} {parts[5]}</pose>"
        chassis = re.sub(r'<pose>(.*?)</pose>', lift_pose, chassis, count=1)
    
    # Extract top_camera if it exists
    camera_match = re.search(r'(<!-- TOP CAMERA -->.*?</joint>)', content, flags=re.DOTALL)
    camera = camera_match.group(1) if camera_match else ""

    wheels_xml = ""
    joints_xml = ""
    
    for prefix, sign_x in [('front', 1), ('rear', -1)]:
        x = d['x_f'] if prefix == 'front' else d['x_r']
        for side, sign_y in [('left', 1), ('right', -1)]:
            y = d['y'] * sign_y
            name = f"{prefix}_{side}_wheel"
            
            mu = "1.5" if prefix == 'rear' else "0.0"
            
            wheels_xml += f"""
    <link name="{name}">
      <pose>{x} {y} {d['z']} 1.5707963 0 0</pose>
      <inertial>
        <mass>8.0</mass>
        <inertia><ixx>0.04</ixx><iyy>0.04</iyy><izz>0.07</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="v"><geometry><cylinder><radius>{d['r_t']}</radius><length>{d['l_t']}</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <visual name="rim"><geometry><cylinder><radius>{d['r_r']}</radius><length>{d['l_r']}</length></cylinder></geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      <collision name="c"><geometry><cylinder><radius>{d['r_t']}</radius><length>{d['l_t']}</length></cylinder></geometry>
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
        <contact><ode><kp>10000000.0</kp><kd>1.0</kd><min_depth>0.002</min_depth></ode></contact></surface></collision>
    </link>"""
            
            joints_xml += f"""
    <joint name="{name}_joint" type="revolute">
      <parent>chassis</parent><child>{name}</child>
      <axis><xyz>0 0 -1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>"""

    # FIXED: rear_left_wheel_joint matches the joints defined above
    diff_drive_xml = f"""
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros></ros>
      <update_rate>50</update_rate>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>{d['y']*2}</wheel_separation>
      <wheel_diameter>{d['r_t']*2}</wheel_diameter>
      <max_wheel_torque>150</max_wheel_torque>
      <max_wheel_acceleration>2.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_source>1</odometry_source>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
    </plugin>"""

    new_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{v}">
    <static>false</static>
{chassis}
{wheels_xml}
{camera}
{joints_xml}
{diff_drive_xml}
  </model>
</sdf>"""

    with open(path, 'w') as f:
        f.write(new_sdf)

print("Wheel fix applied!")
