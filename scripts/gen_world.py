#!/usr/bin/env python3
"""Generate 50m×50m factory world with rich obstacles"""
import math, random
random.seed(42)

# Fixed header + car model (from current working world)
header = '''<sdf version="1.6">
  <world name="factory_50x50">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.9 0.9</diffuse><direction>-0.3 0.15 -0.9</direction></light>
    <model name="ground"><static>true</static>
      <link name="l"><collision name="c"><geometry><box><size>100 100 0.1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>100 100 0.1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5</ambient><diffuse>0.65 0.65 0.65</diffuse></material></visual></link></model>
'''

# Helper: pillar
def pillar(name, x, y, r=0.45, h=5, z=2.5, color="0.55 0.5 0.5"):
    c = random.random()
    if c < 0.3: r2, g2, b2 = 0.55, 0.5, 0.5
    elif c < 0.6: r2, g2, b2 = 0.5, 0.55, 0.5
    else: r2, g2, b2 = 0.5, 0.5, 0.55
    return f'''    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} {z:.1f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><cylinder><radius>{r:.2f}</radius><length>{h:.0f}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r:.2f}</radius><length>{h:.0f}</length></cylinder></geometry>
          <material><ambient>{r2:.2f} {g2:.2f} {b2:.2f}</ambient><diffuse>{r2+0.1:.2f} {g2+0.1:.2f} {b2+0.1:.2f}</diffuse></material></visual></link></model>'''

# Helper: wall section
def wall(name, x, y, sx, sy, color="0.7 0.4 0.3"):
    return f'''    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} 2.5 0 0 {math.atan2(sy,sx):.4f}</pose>
      <link name="l"><collision name="c"><geometry><box><size>{max(sx,sy):.1f} 0.3 5</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{max(sx,sy):.1f} 0.3 5</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color.replace('0.','0.')}</diffuse></material></visual></link></model>'''

# Helper: box/warehouse
def crate(name, x, y, sx, sy, sz, z=0.5, yaw=0):
    r2, g2, b2 = random.uniform(0.3,0.7), random.uniform(0.3,0.7), random.uniform(0.3,0.7)
    return f'''    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} {z:.2f} 0 0 {yaw:.2f}</pose>
      <link name="l"><collision name="c"><geometry><box><size>{sx:.1f} {sy:.1f} {sz:.1f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.1f} {sy:.1f} {sz:.1f}</size></box></geometry>
          <material><ambient>{r2:.2f} {g2:.2f} {b2:.2f}</ambient><diffuse>{r2+0.1:.2f} {g2+0.1:.2f} {b2+0.1:.2f}</diffuse></material></visual></link></model>'''

# Helper: barrel
def barrel(name, x, y, r=0.4, h=1.6, z=0.8):
    return f'''    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} {z:.2f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r:.2f}</radius><length>{h:.2f}</length></cylinder></geometry>
          <material><ambient>0.65 0.45 0.2</ambient><diffuse>0.7 0.5 0.25</diffuse></material></visual></link></model>'''

# Helper: ramp
def ramp(name, x, y, pitch=0.35):
    return f'''    <model name="{name}"><static>true</static><pose>{x:.1f} {y:.1f} 0.35 0 {pitch:.2f} 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>6 2 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>6 2 0.3</size></box></geometry>
          <material><ambient>0.45 0.45 0.45</ambient><diffuse>0.55 0.55 0.55</diffuse></material></visual></link></model>'''

models = []

# ==== Outer perimeter walls (50m sides) ====
for side in ['n','s','e','w']:
    if side == 'n':
        for x in range(-25, 26, 5):
            models.append(wall(f"outer_{side}_{x}", x, 25, 5, 0.4, "0.75 0.35 0.25"))
    elif side == 's':
        for x in range(-25, 26, 5):
            models.append(wall(f"outer_{side}_{x}", x, -25, 5, 0.4, "0.3 0.4 0.75"))
    elif side == 'e':
        for y in range(-25, 26, 5):
            models.append(wall(f"outer_{side}_{y}", 25, y, 5, 0.4, "0.35 0.7 0.35"))
    elif side == 'w':
        for y in range(-25, 26, 5):
            models.append(wall(f"outer_{side}_{y}", -25, y, 5, 0.4, "0.75 0.65 0.2"))

# ==== Internal walls → rooms + corridors ====
# Horizontal partitions
for x in [-12, 0, 12]:
    models.append(wall(f"hw_{x}_a", x, -8, 8, 0.3, "0.55 0.55 0.65"))
    models.append(wall(f"hw_{x}_b", x, 8, 8, 0.3, "0.55 0.55 0.65"))

# Vertical partitions  
for y in [-12, 0, 12]:
    models.append(wall(f"vw_a_{y}", -8, y, 8, 0.3, "0.55 0.55 0.55"))
    models.append(wall(f"vw_b_{y}", 8, y, 8, 0.3, "0.55 0.55 0.55"))

# Short walls creating complex corridors
for (x,y,yaw) in [(6,-4,0),(-6,4,0),(4,6,1.57),(-4,-6,1.57),
                   (18,3,0),(-18,-3,0),(3,18,1.57),(-3,-18,1.57),
                   (20,-10,0),(-20,10,0),(10,20,1.57),(-10,-20,1.57)]:
    models.append(wall(f"sw_{x}_{y}", x, y, 5, 0.2, "0.5 0.5 0.5"))

# ==== Pillars — grid + scattered ====
for x in range(-20, 22, 8):
    for y in range(-20, 22, 8):
        if abs(x) > 2 or abs(y) > 2:  # avoid spawn point
            models.append(pillar(f"p{x}_{y}", x, y, r=random.uniform(0.3,0.5), h=5))

# Extra pillars near corridors
for _ in range(30):
    x = random.uniform(-22, 22)
    y = random.uniform(-22, 22)
    if abs(x) > 2 or abs(y) > 2:
        models.append(pillar(f"px{_}", x, y, r=random.uniform(0.2,0.45), h=4.5))

# ==== Crates & shelves (60 pieces) ====
for i in range(60):
    x = random.uniform(-22, 22)
    y = random.uniform(-22, 22)
    if abs(x) < 3 and abs(y) < 3: continue  # keep spawn area clear
    sx = random.uniform(0.8, 2.5)
    sy = random.uniform(0.6, 2.0)
    sz = random.uniform(0.8, 2.4)
    models.append(crate(f"cr{i}", x, y, sx, sy, sz, z=sz/2, yaw=random.uniform(0,1.57)))

# ==== Barrels (20) ====
for i in range(20):
    x = random.uniform(-22, 22)
    y = random.uniform(-22, 22)
    if abs(x) < 3 and abs(y) < 3: continue
    models.append(barrel(f"br{i}", x, y))

# ==== Ramps (8) ====
for i in range(8):
    x = random.uniform(-20, 20)
    y = random.uniform(-20, 20)
    if abs(x) < 4 and abs(y) < 4: continue
    models.append(ramp(f"rm{i}", x, y, pitch=random.choice([0.3, -0.3, 0.35, -0.35])))

# ==== Small decorative objects ====
for i in range(20):
    x = random.uniform(-22, 22)
    y = random.uniform(-22, 22)
    if abs(x) < 3 and abs(y) < 3: continue
    models.append(f'''    <model name="cone{i}"><static>true</static><pose>{x:.1f} {y:.1f} 0.3 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><box><size>0.4 0.4 0.6</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.4 0.4 0.6</size></box></geometry>
          <material><ambient>0.7 0.5 0.2</ambient><diffuse>0.75 0.55 0.25</diffuse></material></visual></link></model>''')

# ==== Car model (same as before) ====
car = '''
    <model name="diff_car">
      <pose>0 0 0.55 0 0 0</pose>
      <link name="chassis">
        <inertial><mass>8.0</mass><inertia><ixx>1.5</ixx><iyy>1.5</iyy><izz>1.5</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>0.9 0.6 0.3</size></box></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>0.9 0.6 0.3</size></box></geometry><material><ambient>0.2 0.25 0.45</ambient><diffuse>0.3 0.35 0.6</diffuse></material></visual>
        <sensor name="imu_sensor" type="imu"><topic>/imu0</topic><always_on>1</always_on><update_rate>400</update_rate></sensor>
        <visual name="imu_tag"><pose>0.25 0.12 0.02 0 0 0</pose><geometry><box><size>0.05 0.04 0.03</size></box></geometry><material><ambient>0.1 0.3 0.9</ambient><diffuse>0.1 0.4 1.0</diffuse></material></visual>
      </link>
      <link name="lidar_tower">
        <pose>0 0 0.28 0 0 0</pose>
        <inertial><mass>0.3</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <visual name="v"><geometry><cylinder><radius>0.1</radius><length>0.1</length></cylinder></geometry><material><ambient>0.1 0.1 0.1</ambient><diffuse>0.15 0.15 0.15</diffuse></material></visual>
        <sensor name="lidar_sensor" type="gpu_lidar"><topic>/lidar0</topic><update_rate>10</update_rate>
          <lidar><scan><horizontal><samples>900</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
          <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.261799</min_angle><max_angle>0.261799</max_angle></vertical></scan>
          <range><min>0.3</min><max>60.0</max><resolution>0.02</resolution></range><noise><type>gaussian</type><mean>0</mean><stddev>0.03</stddev></noise></lidar>
          <always_on>1</always_on><visualize>true</visualize></sensor>
      </link>
      <joint name="lidar_joint" type="fixed"><parent>chassis</parent><child>lidar_tower</child></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>rear_left_joint</left_joint><right_joint>rear_right_joint</right_joint>
        <wheel_separation>0.5</wheel_separation><wheel_radius>0.2</wheel_radius>
        <max_linear_acceleration>2.0</max_linear_acceleration><max_angular_acceleration>3.0</max_angular_acceleration>
        <odom_publish_frequency>30</odom_publish_frequency><topic>/cmd_vel</topic></plugin>
      <link name="rear_left_wheel"><pose>-0.25 -0.25 -0.35 0 0 0</pose>
        <inertial><mass>0.8</mass><inertia><ixx>0.008</ixx><iyy>0.008</iyy><izz>0.008</izz></inertia></inertial>
        <collision name="c"><geometry><sphere><radius>0.2</radius></sphere></geometry><surface><friction><ode><mu>100</mu><mu2>100</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><sphere><radius>0.2</radius></sphere></geometry><material><ambient>0.1 0.1 0.1</ambient><diffuse>0.15 0.15 0.15</diffuse></material></visual></link>
      <joint name="rear_left_joint" type="revolute"><parent>chassis</parent><child>rear_left_wheel</child><axis><xyz>0 1 0</xyz></axis></joint>
      <link name="rear_right_wheel"><pose>-0.25 0.25 -0.35 0 0 0</pose>
        <inertial><mass>0.8</mass><inertia><ixx>0.008</ixx><iyy>0.008</iyy><izz>0.008</izz></inertia></inertial>
        <collision name="c"><geometry><sphere><radius>0.2</radius></sphere></geometry><surface><friction><ode><mu>100</mu><mu2>100</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><sphere><radius>0.2</radius></sphere></geometry><material><ambient>0.1 0.1 0.1</ambient><diffuse>0.15 0.15 0.15</diffuse></material></visual></link>
      <joint name="rear_right_joint" type="revolute"><parent>chassis</parent><child>rear_right_wheel</child><axis><xyz>0 1 0</xyz></axis></joint>
      <link name="front_left_caster"><pose>0.32 -0.25 -0.38 0 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz></inertia></inertial>
        <collision name="c"><geometry><sphere><radius>0.08</radius></sphere></geometry></collision>
        <visual name="v"><geometry><sphere><radius>0.08</radius></sphere></geometry><material><ambient>0.05 0.05 0.05</ambient><diffuse>0.1 0.1 0.1</diffuse></material></visual></link>
      <joint name="front_left_caster_joint" type="ball"><parent>chassis</parent><child>front_left_caster</child></joint>
      <link name="front_right_caster"><pose>0.32 0.25 -0.38 0 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz></inertia></inertial>
        <collision name="c"><geometry><sphere><radius>0.08</radius></sphere></geometry></collision>
        <visual name="v"><geometry><sphere><radius>0.08</radius></sphere></geometry><material><ambient>0.05 0.05 0.05</ambient><diffuse>0.1 0.1 0.1</diffuse></material></visual></link>
      <joint name="front_right_caster_joint" type="ball"><parent>chassis</parent><child>front_right_caster</child></joint>
    </model>
'''

# ==== Assemble ====
with open('/home/c/fastlio_ws/src/fastlio_sim/worlds/ball_robot.sdf', 'w') as f:
    f.write(header)
    for m in models:
        f.write(m + '\n')
    f.write(car)
    f.write('\n  </world>\n</sdf>\n')

print(f"Generated world with {len(models)} obstacles + car")
print("done")
