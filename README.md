# Introspecter_ros2
Web-based tool for live monitoring of ROS 2 nodes, topics, and system state.

## Requirements
- Python 3.10+
- Flask
- UVicorn
- ROS 2 Humble or later

## Starting the Server
1. Clone the repository:
   ```bash
    git clone https://github.com/MannavaVivek/Introspecter_ros2.git
    ```
2.  Navigate to the project directory:
    ```bash
    cd Introspecter_ros2
    ```
3. Install the required python package dependencies. Activate your virtual environment if needed:
   ```bash
    pip install -r requirements.txt
   ```
4. Start the server:
   ```bash
    uvicorn backend:app --host 0.0.0.0 --port 8000
   ```
5. Open your web browser and navigate to `http://localhost:8000` to access the monitoring dashboard.

## Usage instructions
- Use the dashboard to view real-time data on ROS 2 nodes and topics.
- Double-click on nodes or topics to see detailed information.
- Press space on any node or topic to toggle monitoring. Monitored items will have their data refreshed automatically.
- Press enter on any topic to set an expected frequency for monitoring. When monitoring, the rate is displayed in green if within 10% of expected, red otherwise.
- You can search for nodes or topics using the search bar at the top.
- Use the toggle at the top right to show/hide unmonitored items.