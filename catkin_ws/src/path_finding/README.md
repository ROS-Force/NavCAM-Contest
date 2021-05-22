# Path Finding

The [path_finding_node.py](script/path_finding_node.py) uses a service that retrieves the updated map given by RTABMAP and implements [this](https://pypi.org/project/pathfinding/) python library to compute the fastest path to the current goal.

# Subscribers

- **`goal`** ([geometry_msgs/PoseStamped])

  The current goal.

- **`get_map`** ([geometry_msgs.srv/GetMap])

  Service to request the map.

# Publishers

- **`path_image`** ([sensor_msgs/Image])

  The image to vizualize the map.

- **`path`** ([nav_msgs/Path])

  The path to the current goal

# Parameters

- **`algorithm`** ([String])

  Path finding algorithm (A\*, dijkstra, etc ...)
