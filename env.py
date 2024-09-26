from gymnasium.envs.mujoco import MujocoEnv
from gymnasium import utils
from gymnasium.spaces import Box
import numpy as np



class Environment(MujocoEnv, utils.EzPickle):

    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 50,
    }

    def __init__(
        self,
        model_path,
        frame_skip = 10,
        **kwargs
    ):
        utils.EzPickle.__init__(
            self,
            frame_skip,
            **kwargs
        )
  
        self.frame_skip = frame_skip
        observation_space = Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float64)
        MujocoEnv.__init__(self, model_path, 10, observation_space=observation_space, **kwargs)
        

    
    def get_terminated(self):        
        desiredTimesteps = self.taskDuration / (self.del_t)
        if self.timesteps >= desiredTimesteps:
            return True
        else:
            return False
        
        
    def get_truncated(self):
        return not self.is_healthy


    def _get_obs(self):
        observation = np.array([0])
        return observation


    def reset_model(self, seed=None):
        qpos = self.model.keyframe("Init_pos").qpos.copy()
        qvel = self.model.keyframe("Init_pos").qvel.copy()
        self.set_state(qpos, qvel)
        info = {}
        return self._get_obs()

    
    # def set_state_manual(self, mode):
    #     if mode == 'stand':
    #         qpos = self


    
    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        observation = [0]
        total_reward = 0
        terminated = False
        truncated = False
        info = {}
        return observation, total_reward, terminated, truncated, info


    def viewer_setup(self):
        assert self.viewer is not None
        for key, value in DEFAULT_CAMERA_CONFIG.items():
            if isinstance(value, np.ndarray):
                getattr(self.viewer.cam, key)[:] = value
            else:
                setattr(self.viewer.cam, key, value)
