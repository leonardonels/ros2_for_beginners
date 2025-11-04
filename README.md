# ros2_for_beginners

This repository holds a concise step-by-step checklist and tips for getting started with ROS 2 (Humble) on Ubuntu 22.04 and for creating and testing simple example nodes (Python and C++).

The goal: give a beginner clear commands, verification steps, and small examples to bootstrap learning.

## Quick overview

- OS: Ubuntu 22.04 LTS (recommended for ROS 2 Humble)
- ROS 2 distribution: humble
- Build tool: colcon for building packages
- Example packages: turtlesim (used for testing / exploring topics and remapping)

---

## Prerequisites

- A machine running Ubuntu 22.04 or a VM with sufficient resources.
- Internet access to install packages.
- A terminal (bash). If you like terminal multiplexing, `tmux` is handy but optional.

Install tmux (optional, but recommended):

```bash
sudo apt update
sudo apt install -y tmux
```

## Step 1 — Install ROS 2 Humble (high level)

Follow the official ROS 2 installation instructions for Ubuntu 22.04. High-level steps are:

1. Configure the package repository and keys.
2. Install `ros-humble-desktop` (or another subset such as `ros-humble-ros-base`).
3. Source the setup script.

Example (shortened / typical commands):

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && apt upgrade -y
sudo apt install ros-humble-desktop ros-dev-tools
```

Note: always check the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for the most up-to-date commands.

## Step 2 — Source the environment

After installation, source the ROS 2 setup script to get the ROS environment variables in your shell session:

```bash
source /opt/ros/humble/setup.bash
# Make it persistent for new shells:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Verify the environment variables:

```bash
printenv | grep -i ROS
```

Expected output lines (at least):

- ROS_VERSION=2
- ROS_PYTHON_VERSION=3
- ROS_DISTRO=humble

If any are missing, try sourcing again or opening a new terminal.

## Step 3 — Install and run turtlesim (learning tool)

`turtlesim` is a small graphical simulator used for learning ROS 2. Install and run it to verify ROS 2 works.

Install:

```bash
sudo apt install -y ros-humble-turtlesim
```

Run the simulator (in one terminal):

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

In another terminal, run the keyboard teleop node:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key
```

You should be able to move the turtle with arrow keys.

## Useful ROS 2 CLI checks

- List nodes:
    - `ros2 node list`
- Get node info:
    - `ros2 node info /<node_name>`
- List topics (short):
    - `ros2 topic list`
- List topics with type:
    - `ros2 topic list -t`
- Inspect topic type and data:
    - `ros2 topic echo /topic_name` (or `--once` to print a single message)
- Find who publishes/subscribes to a topic:
    - `ros2 topic info /topic_name`
- Publish a single Twist message to control the turtle:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

This command sends one velocity command to the `turtle1` topic and the turtle should move.

## Step 4 — Remapping topics (example)

Remapping lets you redirect topics at runtime. Example: run the teleop node but remap the topic so its commands go to `turtle2` instead of `turtle1`.

```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

Tip: remapping is powerful for testing multiple nodes without changing source code.

## Step 5 — Install colcon (build tool)

`colcon` is the standard build tool used to build ROS 2 workspace packages.

```bash
sudo apt install -y python3-colcon-common-extensions
```

Create a workspace layout (if you don't have one yet):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Step 6 — Create a simple Python node (publisher)

Create a minimal `ament_python` package inside `src/` and build with `colcon`.

High-level steps:

1. From your workspace root:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pub
```

2. Implement a minimal node in `py_pub/py_pub/publisher_member_function.py` (template):

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. Verify all dependecies inside the package.xml:

```xml
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>

<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

4. Add an entry point with the setup.py file:

```python
entry_points={
        'console_scripts': [
                'talker = py_pub.publisher_member_function:main',
        ],
},
```

5. Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select py_pub
source install/setup.bash
ros2 run py_pub talker
```

This runs your Python node and you should see the logger message printed.

## Step 7 — Create a simple C++ node (subscribe)

Use `ros2 pkg create --build-type ament_cmake` to scaffold a C++ package, add a small subscriber, then build with `colcon`.

High-level steps:

1. From your workspace root:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_sub
```

2. Implement a minimal node in `cpp_sub/src/subscriber_member_function.cpp` (template):

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

3. Complete the CMakeLists.txt file:

```CMake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
```

4. Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select cpp_sub
source install/setup.bash
ros2 run cpp_sub listener
```

## Step 8 — Topic performance and inspection

- Measure publishing frequency:
    - `ros2 topic hz /topic_name`
- Measure bandwidth (approx):
    - `ros2 topic bw /topic_name`

These are handy to validate message rates and approximate throughput.

Since the publishig node works with a timer_callback you can change the frequnecy of publising with:

```python
timer_period = 0.5  # seconds
```

And measure again the publishing frequency.

## Troubleshooting & tips

- If GUI apps (like `turtlesim`) don't open in a VM, ensure X11 forwarding or use a native desktop environment.
- If `ros2` commands say `command not found`, make sure you ran `source /opt/ros/humble/setup.bash` or restarted your shell.
- For permission errors on serial devices or hardware, check udev rules or run with correct user permissions.
- If building with `colcon` fails, run `colcon build --event-handlers console_cohesion+` to get fuller logs.

## Next steps and learning path

1. Work through the official ROS 2 tutorials to understand publishers, subscribers, services, actions, and parameters.
2. Create a complete sample package (publisher + subscriber) in both Python and C++ and write small tests.
3. Learn launch files (`ros2 launch`) to start multiple nodes and handle remappings easily.

---

If you'd like, I can:

- Add a minimal `my_py_pkg` skeleton into this workspace (create files, build and run a smoke test).
- Add step-by-step C++ example files for `my_cpp_pkg`.

Tell me which of the above you'd like next and I'll create the package files and verify they build.
