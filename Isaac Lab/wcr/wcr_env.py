from __future__ import annotations

import torch
from collections.abc import Sequence
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils import configclass
from .waypoint import WAYPOINT_CFG
from isaaclab_assets.robots.wcr import WCR_CFG
from isaaclab.markers import VisualizationMarkers

@configclass
class WcrEnvCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 60.0
    action_space = 3
    observation_space = 8
    state_space = 0
    sim: SimulationCfg = SimulationCfg(dt=1 / 60, render_interval=decimation)
    robot_cfg: ArticulationCfg = WCR_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    waypoint_cfg = WAYPOINT_CFG

    throttle_dof_name = [
        "BL_wheel",
        "BR_wheel",
        "FL_wheel",
        "FR_wheel",
    ]
    steering_dof_name = [
        "BL_steering",
        "BR_steering",
        "FL_steering",
        "FR_steering",
    ]

    wheel_positions_xy = [
        [-0.1125,  0.1125],  # BL
        [-0.1125,  -0.1125],  # BR
        [ 0.1125, 0.1125],  # FL
        [ 0.1125, -0.1125],  # FR
    ]

    env_spacing = 16.0
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=env_spacing, replicate_physics=True)

class WcrEnv(DirectRLEnv):
    cfg: WcrEnvCfg

    def __init__(self, cfg: WcrEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._throttle_dof_idx, _ = self.wcr.find_joints(self.cfg.throttle_dof_name)
        self._steering_dof_idx, _ = self.wcr.find_joints(self.cfg.steering_dof_name)
        self._throttle_state = torch.zeros((self.num_envs, 4), device=self.device, dtype=torch.float32)
        self._steering_state = torch.zeros((self.num_envs, 4), device=self.device, dtype=torch.float32)
        self._goal_reached = torch.zeros((self.num_envs), device=self.device, dtype=torch.int32)
        self.task_completed = torch.zeros((self.num_envs), device=self.device, dtype=torch.bool)
        self._num_goals = 40
        self._target_positions = torch.zeros((self.num_envs, self._num_goals, 2), device=self.device, dtype=torch.float32)
        self._markers_pos = torch.zeros((self.num_envs, self._num_goals, 3), device=self.device, dtype=torch.float32)
        self._position_error = torch.zeros((self.num_envs), device=self.device, dtype=torch.float32)
        self._previous_position_error = torch.zeros((self.num_envs), device=self.device, dtype=torch.float32)
        self._target_index = torch.zeros((self.num_envs), device=self.device, dtype=torch.int32)
        self.env_spacing = self.cfg.env_spacing
        self.course_length_coefficient = 4.5
        self.course_width_coefficient = 4.0
        self.position_tolerance = 0.1
        self.goal_reached_bonus = 10.0
        self.position_progress_weight = 1.0
        self.heading_coefficient = 0.25
        self.heading_progress_weight = 0.05




    def _setup_scene(self):
        spawn_ground_plane(
            prim_path="/World/ground",
            cfg=GroundPlaneCfg(
                size=(500.0, 500.0),
                color=(0.2, 0.2, 0.2),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    friction_combine_mode="average",
                    restitution_combine_mode="average",
                    static_friction=0.5,
                    dynamic_friction=0.5,
                    restitution=0.1,
                ),
            ),
        )
        self.wcr = Articulation(self.cfg.robot_cfg)
        self.waypoints = VisualizationMarkers(self.cfg.waypoint_cfg)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])
        self.scene.articulations["wcr"] = self.wcr
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)
        self._wheel_positions_xy = torch.tensor(self.cfg.wheel_positions_xy, device=self.device, dtype=torch.float32)




    def _pre_physics_step(self, actions: torch.Tensor) -> None:
    # cmd_vel interpretation: actions = [vx, vy, omega] per env
        vx = actions[:, 0].unsqueeze(1)  # (num_envs, 1)
        vy = actions[:, 1].unsqueeze(1)
        omega = actions[:, 2].unsqueeze(1)


    # Get wheel positions
        x = self._wheel_positions_xy[:, 0].unsqueeze(0).expand(self.num_envs, -1)  # (num_envs, 4)
        y = self._wheel_positions_xy[:, 1].unsqueeze(0).expand(self.num_envs, -1)

    # Compute individual wheel velocities
        vx_i = vx - y * omega
        vy_i = vy + x * omega

        v_i = torch.sqrt(vx_i**2 + vy_i**2)
        v_i = torch.where(vx_i < 0, -v_i, v_i)
        delta_i = torch.atan2(vy_i, vx_i)


        
    # ✅ Now apply your scaling/clamping logic AFTER calculations
        throttle_scale = 10
        throttle_max = 4 # [rad/s]
        steering_scale = 1.0
        steering_max = 1.0 # [rad]

        self._throttle_action = torch.clamp(v_i * throttle_scale, -throttle_max, throttle_max) 
        self._steering_action = torch.clamp(delta_i, -steering_max, steering_max) * steering_scale

        self._throttle_state = self._throttle_action
        self._steering_state = self._steering_action

       # print(f"Throttle action: {self._throttle_state}")
        #print(f"Steering action: {self._steering_state}")


    def _apply_action(self) -> None:
        self.wcr.set_joint_velocity_target(self._throttle_state, joint_ids=self._throttle_dof_idx)
        self.wcr.set_joint_position_target(self._steering_state, joint_ids=self._steering_dof_idx)
        #self.wcr.write_joint_position_to_sim(self._steering_state, self._steering_dof_idx)
        #self.wcr.write_joint_velocity_to_sim(self._throttle_state, self._throttle_dof_idx)
        



    def _get_observations(self) -> dict:
        current_target_positions = self._target_positions[self.wcr._ALL_INDICES, self._target_index]
        self._position_error_vector = current_target_positions - self.wcr.data.root_pos_w[:, :2]
        self._previous_position_error = self._position_error.clone()
        self._position_error = torch.norm(self._position_error_vector, dim=-1)

        heading = self.wcr.data.heading_w
        target_heading_w = torch.atan2(
            self._target_positions[self.wcr._ALL_INDICES, self._target_index, 1] - self.wcr.data.root_link_pos_w[:, 1],
            self._target_positions[self.wcr._ALL_INDICES, self._target_index, 0] - self.wcr.data.root_link_pos_w[:, 0],
        )
        self.target_heading_error = torch.atan2(torch.sin(target_heading_w - heading), torch.cos(target_heading_w - heading))

        obs = torch.cat(
            (
                self._position_error.unsqueeze(dim=1),
                torch.cos(self.target_heading_error).unsqueeze(dim=1),
                torch.sin(self.target_heading_error).unsqueeze(dim=1),
                self.wcr.data.root_lin_vel_b[:, 0].unsqueeze(dim=1),
                self.wcr.data.root_lin_vel_b[:, 1].unsqueeze(dim=1),
                self.wcr.data.root_ang_vel_w[:, 2].unsqueeze(dim=1),
                self._throttle_state[:, 0].unsqueeze(dim=1),
                self._steering_state[:, 0].unsqueeze(dim=1),
            ),
            dim=-1,
        )
        print(obs)
        if torch.any(obs.isnan()):
            raise ValueError("Observations cannot be NAN")

        return {"policy": obs}
    
    def _get_rewards(self) -> torch.Tensor:
        position_progress_rew = self._previous_position_error - self._position_error
        target_heading_rew = torch.exp(-torch.abs(self.target_heading_error) / self.heading_coefficient)
        goal_reached = self._position_error < self.position_tolerance
        self._target_index = self._target_index + goal_reached
        self.task_completed = self._target_index > (self._num_goals -1)
        self._target_index = self._target_index % self._num_goals

        composite_reward = (
            position_progress_rew * self.position_progress_weight +
            target_heading_rew * self.heading_progress_weight +
            goal_reached * self.goal_reached_bonus
        )

        one_hot_encoded = torch.nn.functional.one_hot(self._target_index.long(), num_classes=self._num_goals)
        marker_indices = one_hot_encoded.view(-1).tolist()
        self.waypoints.visualize(marker_indices=marker_indices)

        if torch.any(composite_reward.isnan()):
            raise ValueError("Rewards cannot be NAN")

        return composite_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        task_failed = self.episode_length_buf > self.max_episode_length
        return task_failed, self.task_completed
    
    def _generate_figure8_waypoints(self, num_points: int, device: torch.device):
        t_vals = torch.linspace(0, 2 * torch.pi, num_points, device=device)
        x_vals = torch.sin(t_vals)
        y_vals = torch.sin(t_vals) * torch.cos(t_vals)

    # Rotate (same as x_rot = y, y_rot = x)
        x_rot = y_vals
        y_rot = x_vals

    # Scale
        x_scaled = x_rot * (self.env_spacing / self.course_length_coefficient / 2.0)
        y_scaled = y_rot * (self.env_spacing / self.course_width_coefficient / 2.0)

        return x_scaled, y_scaled
    
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.wcr._ALL_INDICES
        super()._reset_idx(env_ids)

        num_reset = len(env_ids)

        # --- Reset root pose and joint states ---
        default_state = self.wcr.data.default_root_state[env_ids]
        wcr_pose = default_state[:, :7]
        wcr_velocities = default_state[:, 7:]
        joint_positions = self.wcr.data.default_joint_pos[env_ids]
        joint_velocities = self.wcr.data.default_joint_vel[env_ids]

    # Position offset for each env
        wcr_pose[:, :3] += self.scene.env_origins[env_ids]
        wcr_pose[:, 0] -= self.env_spacing / 2
        wcr_pose[:, 1] += 2.0 * torch.rand((num_reset), dtype=torch.float32, device=self.device) * self.course_width_coefficient

    # Random heading
        angles = torch.pi / 6.0 * torch.rand((num_reset), dtype=torch.float32, device=self.device)
        wcr_pose[:, 3] = torch.cos(angles * 0.5)
        wcr_pose[:, 6] = torch.sin(angles * 0.5)

    # Apply reset
        self.wcr.write_root_pose_to_sim(wcr_pose, env_ids)
        self.wcr.write_root_velocity_to_sim(wcr_velocities, env_ids)
        self.wcr.write_joint_state_to_sim(joint_positions, joint_velocities, None, env_ids)

    # --- Generate and assign waypoints ---
        x_vals, y_vals = self._generate_figure8_waypoints(self._num_goals, self.device)
        trajectory = torch.stack([x_vals, y_vals], dim=-1)  # shape [num_goals, 2]
        trajectory = trajectory.unsqueeze(0).expand(num_reset, -1, -1).clone()  # shape [num_reset, num_goals, 2]
        env_origins = self.scene.env_origins[env_ids, :2].unsqueeze(1)  # [num_reset, 1, 2]
        trajectory += env_origins

    # Assign
        self._target_positions[env_ids] = trajectory
        self._markers_pos[env_ids, :, :2] = trajectory
        self._target_index[env_ids] = 0

    # Visualize
        visualize_pos = self._markers_pos.view(-1, 3)
        self.waypoints.visualize(translations=visualize_pos)

    # --- Recompute errors for reward/obs ---
        current_target_positions = self._target_positions[self.wcr._ALL_INDICES, self._target_index]
        self._position_error_vector = current_target_positions[:, :2] - self.wcr.data.root_pos_w[:, :2]
        self._position_error = torch.norm(self._position_error_vector, dim=-1)
        self._previous_position_error = self._position_error.clone()

        heading = self.wcr.data.heading_w[:]
        target_heading_w = torch.atan2(
            self._target_positions[:, 0, 1] - self.wcr.data.root_pos_w[:, 1],
            self._target_positions[:, 0, 0] - self.wcr.data.root_pos_w[:, 0],
        )
        self._heading_error = torch.atan2(torch.sin(target_heading_w - heading), torch.cos(target_heading_w - heading))
        self._previous_heading_error = self._heading_error.clone()


