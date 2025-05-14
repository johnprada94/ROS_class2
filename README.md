
# ROS Noetic - Class 2 Exercises 

**Focus**: TurtleBot3 Navigation, PID Control, Advanced ROS Nodes  
**Programming**: Python-only  
**Simulation**: Gazebo + RViz  
**Robot**: TurtleBot3 Burger

---

## 🧱 Step-by-Step: Create and Prepare the Package

Open a terminal and run:

```bash
cd ~/catkin_ws/src
catkin_create_pkg class2 std_msgs geometry_msgs sensor_msgs nav_msgs rospy tf2_ros tf
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Then:

```bash
cd ~/catkin_ws/src/class2
mkdir src
cd src
```

Place all exercise scripts in this `src` directory. After adding each script, make it executable:

```bash
chmod +x <script_name>.py
```

---

## ✏️ Exercise 1: Random Navigation Goal Sender

### 📘 Topic
- Actionlib in ROS
- Autonomous goal sending using `/move_base`

### 💡 Summary
Creates a node that sends random (x,y) navigation goals. Shuts down after 3 failures.

### 📄 Code: `random_goal_sender.py`
### 📄 Code

```python
#!/usr/bin/env python3
import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def random_goal():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    success_count = 0
    fail_count = 0

    while not rospy.is_shutdown() and fail_count < 3:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = random.uniform(-1.5, 1.5)
        goal.target_pose.pose.position.y = random.uniform(-1.5, 1.5)
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal to x={goal.target_pose.pose.position.x:.2f}, y={goal.target_pose.pose.position.y:.2f}")
        client.send_goal(goal)
        client.wait_for_result()

        if client.get_result():
            rospy.loginfo("Goal reached.")
            success_count += 1
        else:
            rospy.logwarn("Goal failed.")
            fail_count += 1

if __name__ == '__main__':
    rospy.init_node('random_goal_sender')
    random_goal()
```

### 🧪 How to Run

1. Terminal 1 – Launch simulation with map:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

2. Terminal 2 – Launch navigation:
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=~/maps/my_map.yaml
```

3. Terminal 3 – Run the node:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun class2 random_goal_sender.py
```

### 🎯 Expected Result
- TurtleBot moves toward random goals in the map.
- Node exits after 3 navigation failures.

---

## ✏️ Exercise 2: SLAM Mapping and Map Saving

### 📘 Topic
- SLAM with `gmapping`
- Saving map with `map_server`

### 💡 Summary
Use teleop to explore, build a map, and save it.

### 🧪 How to Run

1. Terminal 1:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

2. Terminal 2:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

3. Terminal 3:
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

4. Terminal 4 – After exploring:
```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/my_map
```

### 🎯 Expected Result
- Saved files: `my_map.yaml`, `my_map.pgm`

---

## ✏️ Exercise 3: TF Debugging and Broadcasting

### 📘 Topic
- TF frame management

### 💡 Summary
Inject a fake TF frame using `tf2_ros`.

### 📄 Code: `fake_tf_broadcaster.py`

```python
#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

def broadcast():
    rospy.init_node('fake_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    broadcast()
```

### 🧪 How to Run

1. Terminal 1 – Run simulation:
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

2. Terminal 2 – Run node:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun class2 fake_tf_broadcaster.py
```

3. Terminal 3 – Check TF tree:
```bash
rosrun tf view_frames
evince frames.pdf
```

### 🎯 Expected Result
- `odom -> base_footprint` frame appears.

---

## ✏️ Exercise 4: PID Heading Controller

### 📘 Topic
- PID control for heading correction

### 💡 Summary
Control robot to rotate to 90° using P or PI controller.

### 📄 Code: `pid_heading_control.py`
```python
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PIDController:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.target_yaw = math.radians(90)
        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.error_sum = 0.0
        self.last_time = rospy.Time.now()

    def callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(orientation_q)
        error = self.target_yaw - yaw
        self.error_sum += error
        dt = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()
        control = self.Kp * error + self.Ki * self.error_sum * dt

        twist = Twist()
        twist.angular.z = max(min(control, 0.5), -0.5)
        self.pub.publish(twist)

        if abs(error) < 0.01:
            rospy.loginfo("Target angle reached.")
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rospy.signal_shutdown("Done")

    def get_yaw_from_quaternion(self, q):
        import tf
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

if __name__ == '__main__':
    rospy.init_node('pid_heading_controller')
    PIDController()
    rospy.spin()
``` 

### 🧪 How to Run

1. Terminal 1 – Start empty Gazebo world:
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

2. Terminal 2 – Run controller node:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun class2 pid_heading_control.py _Kp:=2.0 _Ki:=0.1
```

### 🎯 Expected Result
- Robot rotates to face 90° (East) and stops.

---

## ✅ Final Checklist

| Step | Description |
|------|-------------|
| ✅ | All scripts in `~/catkin_ws/src/class2/src/` |
| ✅ | Each script made executable with `chmod +x` |
| ✅ | `catkin_make` run from `~/catkin_ws` |
| ✅ | Workspace sourced: `source ~/catkin_ws/devel/setup.bash` |
| ✅ | ROS_MASTER_URI and TurtleBot model properly set |

---

