# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the wcr robot."""

import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

# Get absolute path to workspace root
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))

# USD path with proper resolution for cross-platform compatibility
USD_PATH = os.path.join(WORKSPACE_ROOT, "source", "isaaclab_tasks", "isaaclab_tasks", "direct", "wcr", "custom_assets", "wcr_isaac9.usd")

WCR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_angular_velocity=229.0,           #[deg/s] = 4 rad/s = 0.1016 m/s
            enable_gyroscopic_forces=None,

        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            fix_root_link=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(5, -5, 0.05), rot=(1,0,0,0),
        joint_pos={
            "BL_wheel": 0.0,
            "BR_wheel": 0.0,
            "FL_wheel": 0.0,
            "FR_wheel": 0.0,
            "BL_steering": 0.0,
            "BR_steering": 0.0,
            "FL_steering": 0.0,
            "FR_steering": 0.0,

        },
    ),
        actuators={
        "throttle": ImplicitActuatorCfg(
            joint_names_expr=["BL_wheel", "BR_wheel", "FL_wheel", "FR_wheel"],
            effort_limit_sim=40000.0,
            velocity_limit_sim=1030.0,
            stiffness=0.0,               #[kg*m^2/s^2*deg]
            damping=0.5,                 #[kg*m^2/s*deg]
            friction=0.0000002,
            armature=0.00000006,
        ),
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["BL_steering", "BR_steering", "FL_steering", "FR_steering"],
            effort_limit_sim=40000.0,
            velocity_limit_sim=1030.0,
            stiffness=10,                #[kg*m^2/s^2*deg]
            damping=0.4,                 #[kg*m^2/s*deg]
            friction=0.0000002,
            armature=0.00000006,
        ),
    },
)
