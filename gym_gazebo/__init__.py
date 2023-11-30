import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# cart pole
register(
    id='GazeboCartPole-v0',
    entry_point='gym_gazebo.envs.gazebo_cartpole:GazeboCartPolev0Env',
)

register(
    id='GazeboWheel-v0',
    entry_point='gym_gazebo.envs.gazebo_wheel:GazeboWheelv0Env',
    # max_episode_steps=3000,
)

register(
    id='GazeboWheel-v1',
    entry_point='gym_gazebo.envs.gazebo_wheel:GazeboWheelv1Env',
    # max_episode_steps=3000,
)