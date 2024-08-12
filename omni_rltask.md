# RL Task training tips

### *Before you start:*

- ==**Warning**==: this doc is written based on ***[OmniIsaacGymEnvs](https://github.com/isaac-sim/OmniIsaacGymEnvs)*** RL Task usage in ***[Isaac Sim](https://developer.nvidia.com/isaac/sim)***.
  - The default function calls may be **different** in lab even if they share the same names. Check carefully if you are trying to use this for training in Isaac lab.
  - See [here](https://isaac-sim.github.io/IsaacLab/source/migration/migrating_from_isaacgymenvs.html) for **Isaac Lab** official documentation
- Official docs:
  - Isaac Sim API [doc](https://docs.omniverse.nvidia.com/py/isaacsim/)
  - OmniIsaacGymEnvs [RL framework](https://github.com/isaac-sim/OmniIsaacGymEnvs/blob/main/docs/framework/framework.md)



## RLTask

- `from omniisaacgymenvs.tasks.base.rl_task import RLTask`

  

### Scene initialization:

- Register prims in world (customized in task script)
  - Prims are loaded from USD file ([guidance](#urdf2usd) for converting URDF toUSD) 
- Clone env_0 to all environments (built-in func at `RLTask.set_up_scene(self, scene)`)
- Add views on stage (customized in task script)



#### Built-in prims (check Isaac Sim [API]((https://docs.omniverse.nvidia.com/py/isaacsim/)) for details):

- `omni.isaac.core`
  - Simple prims: e.g.`objects.FixedCuboid`, `objects.DynamicSphere`
  - Articulations: 
    - `articulations.Articulation`
    - `robots.robot.Robot`

- Core prop/func in prim ***view***:
  - Get position and rotation:`get_world_poses()`
  - Articulated prim: 
    - info: `dof_names`,  `get_dof_index()`,  `get_dof_limits()`
    - action: `apply_action()` or `set_joint_position_targets()`, `set_joint_positions()`, `set_joint_velocities()`



#### Core functions:

- `omni.isaac.core.utils.stage.add_reference_to_stage(usd_path, prim_path)`

- `set_up_scene()`: cloner that clone env_0 to all envs

- `initialize_views()`: set `_env_pos`, start renderer

  ```python
      def initialize_views(self, scene):
          """Optionally implemented by individual task classes to initialize views used in the task.
              This API is required for the extension workflow, where tasks are expected to train on a pre-defined stage.
  
          Args:
              scene (Scene): Scene to remove existing views and initialize/add new views.
          """
          self._cloner = GridCloner(spacing=self._env_spacing)
          pos, _ = self._cloner.get_clone_transforms(self._num_envs)
          self._env_pos = torch.tensor(np.array(pos), device=self._device, dtype=torch.float)
          if self._env.render_enabled:
              # initialize capturer for viewport recording
              if self._cfg.get("enable_recording", False) and not self._dr_randomizer.randomize:
                  self._env.create_viewport_render_product(resolution=(self.viewport_camera_width, self.viewport_camera_height))
  
  ```



### Env reset:

- Reset robot and objects

- Reset built-in properties

- Reset customized properties

  

#### Property:

- `reset_buf`

  - Initialized as `torch.ones(self._num_envs, device=self._device, dtype=torch.long)`

  - 0 or 1, 1 flags the env that requires reset

    

#### Core functions

- `cleanup()`: clean buffer

  ```python
      def cleanup(self) -> None:
          """Prepares torch buffers for RL data collection."""
  
          # prepare tensors
          self.obs_buf = torch.zeros((self._num_envs, self.num_observations), device=self._device, dtype=torch.float)
          self.states_buf = torch.zeros((self._num_envs, self.num_states), device=self._device, dtype=torch.float)
          self.rew_buf = torch.zeros(self._num_envs, device=self._device, dtype=torch.float)
          self.reset_buf = torch.ones(self._num_envs, device=self._device, dtype=torch.long)
          self.progress_buf = torch.zeros(self._num_envs, device=self._device, dtype=torch.long)
          self.extras = {}
  ```

- `post_reset()`:

  - Function called after the first reset at initialization
  - Overwritten in task script 

- `reset_idx()`: 

  - Reset robot, objects and related properties in given env idx
  - Overwritten in task script 

- `reset()`:

  - Reset `reset_buf` to all zeros 

  

#### Step actions:

- `step()`:

  - render&record

  - clip actions

  - `self._task.pre_physics_step(actions)`

  - `self._task.post_physics_step()`

  - `return obs_dict, self._rew, self._resets, self._extras`

    

- `pre_physics_step()`:

  - Reset env according to reset_buf and apply actions

  - Overwritten in task script

  - Default template:

    ```python
    def pre_physics_step(self, actions: torch.Tensor) -> None:
        # implement logic to be performed before physics steps
        self.perform_reset()
        self.apply_action(actions)
    ```

    

  

- `post_physics_step()`:

  ```python
      def post_physics_step(self):
          """Processes RL required computations for observations, states, rewards, resets, and extras.
              Also maintains progress buffer for tracking step count per environment.
  
          Returns:
              obs_buf(torch.Tensor): Tensor of observation data.
              rew_buf(torch.Tensor): Tensor of rewards data.
              reset_buf(torch.Tensor): Tensor of resets/dones data.
              extras(dict): Dictionary of extras data.
          """
  
          self.progress_buf[:] += 1
  
          if self._env.world.is_playing():
              self.get_observations()
              self.get_states()
              self.calculate_metrics()
              self.is_done()
              self.get_extras()
  
          return self.obs_buf, self.rew_buf, self.reset_buf, self.extras
  ```





### URDF to USD<a name="urdf2usd"></a>

*Convert with:*

- **Isaac Sim GUI**: official doc [here](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html)

- **Python** function:

  ```python
  omni.kit.commands.execute(
              "URDFParseAndImportFile",
              import_config=cfg,
              urdf_path=fp,
              dest_path=dp,
          )
  ```

  Example:

  - ```python
    def init_cfg():
        from omni.importer.urdf import _urdf
    
        cfg = _urdf.ImportConfig()
        cfg.merge_fixed_joints = False
        cfg.convex_decomp = False
        cfg.import_inertia_tensor = True
        cfg.make_default_prim = True
        cfg.self_collision = False
        cfg.create_physics_scene = True
        cfg.default_drive_strength = 1047.19751
        cfg.default_position_drive_damping = 52.35988
        cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_NONE
        cfg.distance_scale = 1
        cfg.density = 0.0
    
        cfg.fix_base = False  # Object will be fixed to world origin if True
        cfg.parse_mimic = False  # Set to False if robot has mimic joints
        
        return cfg
    
    
    def urdf2usd(urdf_path, dest_path, ucfg={}):
        from omni.isaac.core.utils.stage import clear_stage
        import omni.kit.commands
    
        cfg = init_cfg()
        for k, v in list(ucfg.items()):
            setattr(cfg, k, v)
    
        urdf_path = urdf_path if isinstance(urdf_path, list) else [urdf_path]
        dest_path = dest_path if isinstance(dest_path, list) else [dest_path]
    
        for fp, dp in zip(urdf_path, dest_path):
            clear_stage()
            omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=fp,
                import_config=cfg,
                dest_path=dp,
            )
            
    ```

  



### Pipeline brief

1. Load config : `hydra` 
2. Initialize env: `VecEnvRLGames`
3. initialize task: `initialize_task()` - `env.set_task()` 
4. Start customized trainer: `Trainer` 
   1. Register env config
   2. Initialize agent
   3. Train

