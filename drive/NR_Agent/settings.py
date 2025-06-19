import os
import platform
"""
Robocar AI Project - Global Settings Module

This file centralizes all configuration constants used across the project.
"""

CONFIG_PATH = "agent_config.json" # Path to agent configuration JSON (FOV, number of rays)
FOV = 180                         # Field of view in degrees
NUM_RAYS = 24                     # Number of LiDAR rays

###############################################################################
#                                                                             #
#                          AGENT NETWORK SETTINGS                             #
#                                                                             #
###############################################################################

INPUT_SIZE = NUM_RAYS                  # Neural network input size
MODEL_SAVE_PATH = "model/nr_model.pth" # Model save/load path
EPOCHS = 300                           # Total number of training epochs
LEARNING_RATE = 0.0001                 # Learning rate for the optimizer
WEIGHT_DECAY = 1e-4                    # L2 penalty for regularization
BATCH_SIZE = 64                        # Number of samples per batch
LR_STEP_SIZE = 50                      # Step interval before reducing learning rate
LR_GAMMA = 0.5                         # Learning rate reduction factor
NUM_THREADS = 8                        # CPU parallelism for training

###############################################################################
#                                                                             #
#                         DATA COLLECTION SETTINGS                            #
#                                                                             #
###############################################################################

SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080
DEFAULT_CSV_PATH = "data_rn/" # Default path for storing or reading collected training data
LIDAR_CONFIG = {
    "agents": [
        {
            "fov": FOV,
            "nbRay": NUM_RAYS
        }
    ]
}
