# Quickstart for Module 4: Vision-Language-Action (VLA)

This guide provides the steps to set up the environment for running the VLA module examples.

## 1. Prerequisites

- Python 3.10+
- ROS 2 Humble or Iron
- Gazebo Fortress or Ignition (if using Gazebo for simulation)
- NVIDIA Isaac Sim (if using Isaac Sim for simulation)
- An OpenAI API key

## 2. Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Install Python dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
    A `requirements.txt` will need to be created containing:
    - `rclpy`
    - `openai`
    - `opencv-python`
    - `sounddevice`
    - `numpy`

3.  **Set up environment variables**:
    Create a `.env` file in the root of the project and add your OpenAI API key:
    ```
    OPENAI_API_KEY="your-api-key"
    ```

4.  **Build the ROS 2 packages**:
    ```bash
    colcon build
    ```

## 3. Running the examples

Source the ROS 2 setup file and run the launch file for the desired example:
```bash
source install/setup.bash
ros2 launch <package_name> <launch_file_name>
```
