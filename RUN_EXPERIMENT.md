# Running the Experiment on Ubuntu (ROS1)

Follow these steps to pull the latest code and run the experiment on your robot/VM.

## 1. Update Code
Open a terminal and go to your project folder:
```bash
cd ~/aig240project1
git pull origin main
```
*If you have local changes that conflict, you can force reset (WARNING: loses local changes):*
```bash
git fetch --all
git reset --hard origin/main
```

## 2. Setup Workspace
Run the setup script to install the package into your `~/catkin_ws`:
```bash
chmod +x setup.sh
./setup.sh
```
*This will copy `lab3_turtlesim` to `~/catkin_ws/src`, run `catkin_make`, and source the setup file.*

## 3. Run the Experiment

### A. Basic Control (One Turtle)
```bash
roslaunch lab3_turtlesim turtle_controller.launch
```
-   Use **w/a/s/d** to move.
-   Use **q/e/z/c** to move diagonally.

### B. Multi-Turtle Test (REQUIRED)
To prove you can control multiple turtles independently:

1.  **Launch ROS** (Terminal 1):
    ```bash
    roscore
    ```

2.  **Launch Simulator** (Terminal 2):
    ```bash
    rosrun turtlesim turtlesim_node
    ```

3.  **Spawn Turtle 2** (Terminal 3):
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun lab3_turtlesim spawn_turtle
    ```

4.  **Control Turtle 1** (Terminal 4):
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun lab3_turtlesim turtle_controller turtle1
    ```

5.  **Control Turtle 2** (Terminal 5):
    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun lab3_turtlesim turtle_controller turtle2
    ```

**Goal**: Show that Terminal 4 ONLY moves Turtle 1, and Terminal 5 ONLY moves Turtle 2.
