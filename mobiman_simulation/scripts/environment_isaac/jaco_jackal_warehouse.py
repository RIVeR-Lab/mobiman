#!/home/alpharomeo911/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh

import os
from omni.isaac.kit import SimulationApp

# Simple example showing how to start and stop the helper
simulation_app = SimulationApp({"headless": True})
simulation_app.update()  # Render a single frame
simulation_app.close()  # Cleanup application
