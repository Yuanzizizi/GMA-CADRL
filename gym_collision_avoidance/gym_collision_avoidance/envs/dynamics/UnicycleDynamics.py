import numpy as np
from gym_collision_avoidance.envs import Config
from gym_collision_avoidance.envs.dynamics.Dynamics import Dynamics
from gym_collision_avoidance.envs.util import wrap, find_nearest
import math

class UnicycleDynamics(Dynamics):
    """ Convert a speed & heading to a new state according to Unicycle Kinematics model.
    """

    def __init__(self, agent):
        Dynamics.__init__(self, agent)

    def step(self, action, dt):
        """ 
        In the global frame, assume the agent instantaneously turns by :code:`heading`
        and moves forward at :code:`speed` for :code:`dt` seconds.  
        
        Args:
            action (list): [delta heading angle, speed] command for this agent
            dt (float): time in seconds to execute :code:`action`
        """
        
        selected_speed = action[0]
        selected_heading = wrap(action[1] + self.agent.heading_global_frame)

        dx = selected_speed * np.cos(selected_heading) * dt
        dy = selected_speed * np.sin(selected_heading) * dt
        self.agent.length += np.linalg.norm(np.array([dx, dy]))

        temp_pos = self.agent.pos_global_frame + np.array([dx, dy])
        limit_x, limit_y = Config.MAP_XW/2-0.05, Config.MAP_YW/2-0.05
        self.agent.pos_global_frame = np.array([min(limit_x, max(-limit_x, temp_pos[0])), min(limit_y, max(-limit_y, temp_pos[1]))])
        self.agent.vel_global_frame[0] = selected_speed * np.cos(selected_heading)
        self.agent.vel_global_frame[1] = selected_speed * np.sin(selected_heading)
        self.agent.speed_global_frame = selected_speed
        self.agent.delta_heading_global_frame = wrap(selected_heading -
                                               self.agent.heading_global_frame)
        self.agent.heading_global_frame = selected_heading
