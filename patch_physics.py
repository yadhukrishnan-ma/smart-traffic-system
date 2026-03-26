import os
import re

vehicles = ['ambulance', 'bike', 'bus', 'fire_engine', 'police', 'truck']
base = '/home/dell/smart_traffic_ws/src/smart_intersection/models'

for v in vehicles:
    path = os.path.join(base, v, 'model.sdf')
    with open(path, 'r') as f:
        content = f.read()

    # Fix backwards movement
    content = content.replace('<xyz>0 0 1</xyz>', '<xyz>0 0 -1</xyz>')
    content = content.replace('<xyz> 0 0 1 </xyz>', '<xyz>0 0 -1</xyz>')
    
    # Fix chassis mass
    content = re.sub(r'(<link name="chassis">.*?)<mass>[\d\.]+</mass>', r'\1<mass>800.0</mass>', content, flags=re.DOTALL|re.IGNORECASE)
    
    # Fix wheel mass
    content = re.sub(r'(<link name="[^"]*wheel">.*?)<mass>[\d\.]+</mass>', r'\1<mass>8.0</mass>', content, flags=re.DOTALL|re.IGNORECASE)
    
    # Fix castor mass
    content = re.sub(r'(<link name="castor">.*?)<mass>[\d\.]+</mass>', r'\1<mass>0.5</mass>', content, flags=re.DOTALL|re.IGNORECASE)
    
    # Fix torque and acceleration
    content = re.sub(r'<max_wheel_torque>[\d\.]+</max_wheel_torque>', '<max_wheel_torque>150</max_wheel_torque>', content)
    content = re.sub(r'<max_wheel_acceleration>[\d\.]+</max_wheel_acceleration>', '<max_wheel_acceleration>2.0</max_wheel_acceleration>', content)

    # Ensure kp and kd are set
    content = re.sub(r'<kp>[\d\.]+</kp>', '<kp>10000000.0</kp>', content)
    content = re.sub(r'<kd>[\d\.]+</kd>', '<kd>1.0</kd>', content)

    with open(path, 'w') as f:
        f.write(content)

print("Patch applied to all vehicles.")
