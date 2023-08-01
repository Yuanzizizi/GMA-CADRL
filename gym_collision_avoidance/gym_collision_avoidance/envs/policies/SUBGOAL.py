from gym_collision_avoidance.envs import Config
import gym_collision_avoidance.envs.util as util
import numpy as np
import matplotlib.pyplot as plt


class SUBGOAL(): 
    def __init__(self, static_map, d):
        self.num_angles = Config.NUM_RAYS
        self.num_pieces_unitlen = 5

        self.dist = d
        self.map = static_map

    def get_fmm_dist(self, pos, info):
        pos_index = self.map.world_coordinates_to_map_indices(pos)[0]
        try:
            d = self.dist[pos_index[0], pos_index[1]]
            assert(type(d)==np.float64) 
            return d*Config.GRID_CELL_SIZE
        except:
            return None

    def get_feasible_action_subgoal(self, pos, heading_global_frame, radius):
        min_dist = 99999
        sg = None

        distance_rays = [-1.5]*self.num_angles

        pos_index = self.map.world_coordinates_to_map_indices(pos)[0]
        # posssssssssssssssssssssssssss_X = [pos_index[0]]*self.num_angles
        # posssssssssssssssssssssssssss_Y = [pos_index[1]]*self.num_angles
        base_dist = self.dist[pos_index[0], pos_index[1]]


        for angle_idx in range(self.num_angles):
            selected_heading = angle_idx/self.num_angles * (2*np.pi) - np.pi
            selected_heading = util.wrap(selected_heading + heading_global_frame)

            for length_idx in range(self.num_pieces_unitlen * Config.SUBGOAL_RATIO):  ##
                length = (length_idx+1)/(self.num_pieces_unitlen)  ##

                dx = length * np.cos(selected_heading) 
                dy = length * np.sin(selected_heading)
                pos_new = pos + np.array([dx, dy])

                subgoal_index = self.map.world_coordinates_to_map_indices(pos_new)[0]

                try:
                    d = self.dist[subgoal_index[0], subgoal_index[1]]
                    assert(type(d)==np.float64)
                except:
                    break

                new_distcut = (base_dist-d)*Config.GRID_CELL_SIZE/Config.SUBGOAL_RATIO   
                if new_distcut > distance_rays[angle_idx]:
                    distance_rays[angle_idx] = new_distcut
                    # posssssssssssssssssssssssssss_X[angle_idx] = pos_new[0]
                    # posssssssssssssssssssssssssss_Y[angle_idx] = pos_new[1]
                if d < min_dist:
                    min_dist = d
                    sg = pos_new 
        # print(posssssssssssssssssssssssssss_X, posssssssssssssssssssssssssss_Y)
        # assert(0) 
        return sg, min_dist*Config.GRID_CELL_SIZE, distance_rays       
