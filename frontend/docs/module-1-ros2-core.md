# Chapter 3: Fundamentals of ROS 2

## Chapter Overview

This chapter introduces the core concepts of the Robot Operating System 2 (ROS 2), focusing on its modular architecture and essential communication mechanisms. You will learn about nodes, topics, services, and actions, which are fundamental building blocks for creating distributed robotics applications.

## Learning Objectives

Upon completing this chapter, you will be able to:
*   Define a ROS 2 node and its purpose within a robotics system.
*   Understand the publish/subscribe communication model using topics.
*   Explain the request/response pattern facilitated by services.
*   Describe the goal/feedback/result interaction of actions for long-running tasks.
*   Implement simple ROS 2 nodes for basic communication.

## 3.1 What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that aim to simplify the task of building complex and robust robot applications. ROS 2 is designed to be production-ready, supporting multiple platforms, real-time control, and a robust communication system.

Key features of ROS 2 include:
*   **Distributed System:** Enables multiple processes (nodes) to run on different machines and communicate seamlessly.
*   **Modular Design:** Encourages breaking down complex robot functionalities into smaller, manageable nodes.
*   **Language Agnostic:** Supports clients in C++, Python, and other languages.
*   **Middleware Independent:** Can use different underlying communication protocols (e.g., DDS, Zenoh).

## 3.2 ROS 2 Nodes: The Worker Bees

A **Node** is an executable process that performs computation. In a ROS 2 system, multiple nodes can run concurrently, each responsible for a specific task (e.g., a node for reading sensor data, a node for controlling motors, a node for path planning).

Nodes communicate with each other using various ROS 2 communication mechanisms.

**Example: A Simple Python Node**

```python
# my_robot_controller/my_robot_controller/minimal_publisher.py
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
        msg.data = 'Hello, ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This node (`minimal_publisher`) publishes a "Hello, ROS 2!" message to a topic every 0.5 seconds.

## 3.3 ROS 2 Topics: Publish/Subscribe Communication

**Topics** are the most common way for nodes to exchange data. They implement a *publish/subscribe* model:
*   A **publisher** node sends messages to a topic.
*   One or more **subscriber** nodes receive messages from that topic.
Topics are unidirectional and asynchronous.

**Example: A Simple Python Subscriber Node**

```python
# my_robot_controller/my_robot_controller/minimal_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This node (`minimal_subscriber`) subscribes to the 'topic' and prints any messages it receives.

## 3.4 ROS 2 Services: Request/Response Interaction

**Services** provide a request/response communication model, similar to a client-server interaction. A *client* node sends a request to a *server* node, and the server processes the request and returns a single response. Services are synchronous and bidirectional. They are ideal for tasks that require an immediate, one-time result, like querying a robot's current state or triggering a specific action.

**Example: A Simple Python Service Server**

First, define a service interface (e.g., `AddTwoInts.srv`):
```
# my_robot_controller/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Then, the server implementation:
```python
# my_robot_controller/my_robot_controller/add_two_ints_server.py
import rclpy
from rclpy.node import Node
from my_robot_controller.srv import AddTwoInts # Import your service type

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts Service is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Example: A Simple Python Service Client**

```python
# my_robot_controller/my_robot_controller/add_two_ints_client.py
import sys
import rclpy
from rclpy.node import Node
from r_robot_controller.srv import AddTwoInts # Import your service type

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        node.get_logger().info('Usage: ros2 run my_robot_controller add_two_ints_client A B')
        sys.exit(1)
    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.5 ROS 2 Actions: Long-Running Goal-Oriented Tasks

**Actions** are designed for long-running, goal-oriented tasks that may be preempted (cancelled) and require continuous feedback. They build upon topics and services, providing a more complex communication pattern:
*   A **client** sends a *goal* to an action *server*.
*   The action *server* provides continuous *feedback* as it progresses toward the goal.
*   The action *server* eventually sends a final *result* when the goal is achieved or aborted.
*   The client can also *cancel* the goal.

**Example: A Simple Python Action Server (Conceptual)**

First, define an action interface (e.g., `CountUp.action`):
```
# my_robot_controller/action/CountUp.action
int32 target_number
---
int32 sequence_sum
---
int32 current_number
```

Then, the server implementation (simplified):
```python
# my_robot_controller/my_robot_controller/count_up_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_robot_controller.action import CountUp # Import your action type
import time

class CountUpActionServer(Node):

    def __init__(self):
        super().__init__('count_up_action_server')
        self._action_server = ActionServer(
            self,
            CountUp,
            'count_up',
            self.execute_callback)
        self.get_logger().info('CountUp Action Server is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = CountUp.Feedback()
        feedback_msg.current_number = 0
        sequence_sum = 0

        for i in range(1, goal_handle.request.target_number + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled!')
                return CountUp.Result()

            feedback_msg.current_number = i
            sequence_sum += i
            self.get_logger().info(f'Feedback: {feedback_msg.current_number}, Sum: {sequence_sum}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate work

        goal_handle.succeed()
        result = CountUp.Result()
        result.sequence_sum = sequence_sum
        self.get_logger().info('Goal succeeded!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUpActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab: Creating a "Hello, World!" ROS 2 Publisher and Subscriber

### Introduction
This lab will guide you through creating and running your first ROS 2 publisher and subscriber nodes in Python. You will see how two independent nodes can communicate using a ROS 2 topic.

### Prerequisites
*   Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed.
*   A basic understanding of Python programming.
*   Familiarity with the Linux command line.

### Step-by-Step Instructions

1.  **Set up your ROS 2 environment:**
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2.  **Create a new ROS 2 workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
3.  **Create a new Python package (e.g., `py_pubsub`):**
    ```bash
    ros2 pkg create --build-type ament_python py_pubsub
    ```
4.  **Navigate into the package's Python directory:**
    ```bash
    cd py_pubsub/py_pubsub
    ```
5.  **Create `publisher_member_function.py`:**
    ```python
    # publisher_member_function.py
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
            msg.data = 'Hello, ROS 2! %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)
        minimal_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
6.  **Create `subscriber_member_function.py`:**
    ```python
    # subscriber_member_function.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MinimalSubscriber(Node):
        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

    def main(args=None):
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
7.  **Update `setup.py` and `package.xml`:**
    *   **`py_pubsub/setup.py`**: Add the entry points for your executables.
        ```python
        from setuptools import find_packages, setup

        package_name = 'py_pubsub'

        setup(
            name=package_name,
            version='0.0.0',
            packages=find_packages(exclude=['test']),
            data_files=[
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/resource', ['resource/' + package_name]),
            ],
            install_requires=['setuptools'],
            zip_safe=True,
            maintainer='your_name', # Replace with your name
            maintainer_email='your_email@example.com', # Replace with your email
            description='A minimal publisher/subscriber package',
            license='Apache-2.0', # Or your chosen license
            tests_require=['pytest'],
            entry_points={
                'console_scripts': [
                    'talker = py_pubsub.publisher_member_function:main',
                    'listener = py_pubsub.subscriber_member_function:main',
                ],
            },
        )
        ```
    *   **`py_pubsub/package.xml`**: Add `std_msgs` dependency.
        ```xml
        <?xml version="1.0"?>
        <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
        <package format="3">
          <name>py_pubsub</name>
          <version>0.0.0</version>
          <description>A minimal publisher/subscriber package</description>
          <maintainer email="your_email@example.com">your_name</maintainer>
          <license>Apache-2.0</license>

          <depend>rclpy</depend>
          <depend>std_msgs</depend>

          <test_depend>ament_copyright</test_depend>
          <test_depend>ament_flake8</test_depend>
          <test_depend>ament_pep257</test_depend>
          <test_depend>python3-pytest</test_depend>

          <export>
            <build_type>ament_python</build_type>
          </export>
        </package>
        ```
8.  **Build your workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select py_pubsub
    ```
9.  **Source the setup files:**
    ```bash
    source install/setup.bash
    ```
10. **Run the publisher node:**
    ```bash
    ros2 run py_pubsub talker
    ```
11. **In a new terminal, source and run the subscriber node:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub listener
    ```
    You should see the subscriber receiving messages from the publisher.

### Expected Output
In the publisher terminal, you will see messages like: `[INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! 0"`
In the subscriber terminal, you will see messages like: `[INFO] [minimal_subscriber]: I heard: "Hello, ROS 2! 0"`

## Summary

*   **Nodes** are modular executable units in ROS 2.
*   **Topics** facilitate asynchronous, many-to-many data streaming (publish/subscribe).
*   **Services** provide synchronous, one-to-one request/response communication.
*   **Actions** handle long-running, preemptable tasks with continuous feedback.

## Assessment / Self-Check

1.  What is the primary difference between a ROS 2 topic and a service?
    *   *Hint: Consider the communication pattern and whether a response is expected.*
2.  When would you choose to use an action instead of a service for a task?
    *   *Hint: Think about task duration and the need for feedback/cancellation.*
3.  How can you inspect the active ROS 2 nodes and topics from the command line?
    *   *Hint: There are `ros2` commands for listing these.*

## Further Reading

*   ROS 2 Documentation: [rclpy Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Publisher-And-Subscriber-Python.html)
*   ROS 2 Documentation: [Understanding ROS 2 interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
