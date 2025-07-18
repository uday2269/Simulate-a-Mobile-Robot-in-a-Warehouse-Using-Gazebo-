Project Overview
This project simulates a mobile robot navigating a 3D warehouse environment using Gazebo 11 and ROS Noetic. The robot employs Dijkstra’s algorithm for optimal path planning, dynamically avoiding obstacles while reaching its goal position. The simulation showcases real-time robot movement, intelligent pathfinding, and obstacle avoidance in a structured warehouse setup.

Project Highlights
✅ 3D Warehouse Simulation – Built using Gazebo 11 for realistic environment modeling.
✅ Path Planning – Uses Dijkstra’s algorithm to compute the shortest path.
✅ Dynamic Obstacle Avoidance – Implements costmaps for real-time navigation adjustments.
✅ Visualization Features:

Shortest path (highlighted in red)

Robot’s movement trail (displayed as a green transparent line)
✅ Realistic Navigation – Avoids blocked paths and recalculates routes dynamically.
✅ Simulation Environment – Runs on MATLAB R2024b and Ubuntu 20.04 (Oracle VM).

Project Structure
Simulate-a-Mobile-Robot-in-a-Warehouse-Using-Gazebo/
├── **AIR_Report_Final.docx**       - Final project report (Microsoft Word)  
├── **air_pptNew.pptx**             - PowerPoint presentation summarizing the project  
├── **SIMULATION VIDEO.mp4**        - Demo video of the warehouse robot simulation  
├── **matlabcode.m**                - MATLAB script for path planning/algorithm testing  
├── **Gazebo simulation.jpg**       - Screenshot of the Gazebo warehouse environment  
├── **Bidirectional Navigation.jpg** - Visualization of robot's bidirectional pathfinding  
├── **Validated Paths.jpg**         - Image showing collision-free validated paths  
├── **Ware Occupancy GRID.jpg**     - Warehouse grid map for navigation  

Technologies Used

Gazebo 11 – High-fidelity robot simulation

ROS Noetic – Robot Operating System for middleware

MATLAB R2024b – Algorithm development and visualization

RViz – Real-time robot visualization

Python – Path planning and control scripts

URDF/SDF – Robot and environment modeling

Installation and Setup
Prerequisites
Ubuntu 20.04 LTS (or a VM)

ROS Noetic (installed and configured)

Gazebo 11

MATLAB R2024b (optional for visualization)

Steps
Clone the repository:

bash
git clone https://github.com/<your-username>/Simulate-a-Mobile-Robot-in-a-Warehouse-Using-Gazebo.git  
cd Simulate-a-Mobile-Robot-in-a-Warehouse-Using-Gazebo  
Set up the ROS workspace:

bash
mkdir -p ~/catkin_ws/src  
cp -r Simulate-a-Mobile-Robot-in-a-Warehouse-Using-Gazebo ~/catkin_ws/src/  
cd ~/catkin_ws  
catkin_make  
source devel/setup.bash  
Launch the simulation:

bash
roslaunch warehouse_navigation simulation.launch  
Run path planning (optional):

bash
rosrun warehouse_navigation dijkstra_path_planner.py  
Visualize in MATLAB (optional):
Open matlab/dijkstra_3D_simulation.m and execute the script.

Results
✔ Successful Navigation – The robot reaches its goal while avoiding obstacles.
✔ Optimal Pathfinding – Dijkstra’s algorithm ensures the shortest path.
✔ Dynamic Re-routing – Adjusts path if obstacles move or the goal changes.

Future Improvements
🔹 SLAM Integration – For autonomous warehouse mapping.
🔹 Interactive Goal Setting – Real-time goal updates via RViz.
🔹 Reinforcement Learning – Advanced navigation strategies.

Author
👨‍💻 Uday Kiran
📧 Email: udayuday2269@gmail.com
🔗 LinkedIn: https://www.linkedin.com/in/uday-kiran-tirumalasetty-92b6512b1/
