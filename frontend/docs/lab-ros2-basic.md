# Lab: Your First ROS 2 Publisher and Subscriber

## Lab Overview

This lab will guide you through the process of creating, building, and running your first ROS 2 publisher and subscriber nodes using Python. You will create a simple "talker" node that broadcasts a message and a "listener" node that receives it, demonstrating the fundamental publish/subscribe communication model in ROS 2.

## Learning Objectives

Upon completing this lab, you will be able to:
*   Create a ROS 2 package.
*   Write a simple publisher node in Python.
*   Write a simple subscriber node in Python.
*   Build and run ROS 2 nodes.
*   Verify communication between nodes using ROS 2 command-line tools.

## Prerequisites

*   **Operating System:** Ubuntu 22.04 LTS
*   **ROS 2 Distribution:** ROS 2 Humble Hawksbill (Desktop Install)
*   **Development Tools:** A text editor (like VS Code) and a terminal.
*   **Basic Skills:** Familiarity with the Linux command line and basic Python programming.

---

## Step 1: Set Up Your ROS 2 Workspace

A ROS 2 workspace is a directory where you store and build your ROS 2 packages.

1.  **Open a terminal.**
2.  **Source your ROS 2 setup file.** This makes ROS 2 commands available in your terminal.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
3.  **Create a workspace directory.**
    ```bash
    mkdir -p ~/ros2_ws/src
    ```
4.  **Navigate into the `src` directory.** This is where you will create your new package.
    ```bash
    cd ~/ros2_ws/src
    ```

## Step 2: Create a New ROS 2 Package

Now, you will create a Python package named `py_pubsub` to hold your lab code.

1.  **Use the `ros2 pkg create` command:**
    ```bash
    ros2 pkg create --build-type ament_python py_pubsub
    ```
    This command creates a `py_pubsub` directory with all the necessary boilerplate files for a Python package.

2.  **Inspect the created files.** You will see a `package.xml`, `setup.py`, `setup.cfg`, and a directory named `py_pubsub`.

## Step 3: Write the Publisher Node

The publisher node (the "talker") will send a `String` message to a topic named `topic` every 0.5 seconds.

1.  **Navigate into your package's Python module directory:**
    ```bash
    cd py_pubsub/py_pubsub
    ```

2.  **Create a new file named `publisher_member_function.py`** and add the following code:
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

## Step 4: Write the Subscriber Node

The subscriber node (the "listener") will subscribe to the `topic` and print any messages it receives.

1.  **In the same directory (`py_pubsub/py_pubsub`), create a file named `subscriber_member_function.py`** with the following code:
    ```python
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

## Step 5: Configure the Package

Now, you must configure the package so ROS 2 knows about your new nodes and their dependencies.

1.  **Navigate back to the root of your package:**
    ```bash
    cd ~/ros2_ws/src/py_pubsub
    ```

2.  **Modify `package.xml`** to add a dependency on `std_msgs`, which contains the `String` message type. Open `package.xml` and add the following line within the `<depend>` tags:
    ```xml
    <depend>std_msgs</depend>
    ```

3.  **Modify `setup.py`** to create the executables for your nodes. Open `setup.py` and add the `entry_points` to the `setup` function:
    ```python
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
    ```

## Step 6: Build and Run the Nodes

1.  **Navigate to the root of your workspace:**
    ```bash
    cd ~/ros2_ws
    ```
2.  **Build your package:**
    ```bash
    colcon build --packages-select py_pubsub
    ```
3.  **Source the workspace's setup file.** This step adds your newly created executables to your path.
    ```bash
    source install/setup.bash
    ```
4.  **Run the publisher node.** In your current terminal, run:
    ```bash
    ros2 run py_pubsub talker
    ```
    You should see the publisher sending messages.

5.  **Run the subscriber node.** Open a **new terminal**, and remember to source the setup file again:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_pubsub listener
    ```

## Expected Output

*   In the **talker** terminal, you will see:
    ```
    [INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! 0"
    [INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! 1"
    ...
    ```
*   In the **listener** terminal, you will see:
    ```
    [INFO] [minimal_subscriber]: I heard: "Hello, ROS 2! 0"
    [INFO] [minimal_subscriber]: I heard: "Hello, ROS 2! 1"
    ...
    ```

Congratulations! You have successfully created and run a simple publisher/subscriber system in ROS 2.
