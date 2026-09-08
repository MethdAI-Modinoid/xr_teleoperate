"""
Robot-less Dex3-1 hand controller for headset-only recording.

`Dex3_1_Controller` blocks in its constructor waiting for hand-state DDS traffic::

    while True:
        if any(self.left_hand_state_array) and any(self.right_hand_state_array):
            break
        time.sleep(0.01)
        logger_mp.warning("[Dex3_1_Controller] Waiting to subscribe dds...")

With no Dex3 hardware present that never returns. This is the hand-side twin of the blocker
described in `robot_arm_virtual.py`. See `../../XR_SWAP_PLAN.md` work item A.

The retargeting itself is completely robot-independent: `HandRetargeting` maps XR hand
skeleton data to Dex3 joint angles via dex-retargeting, using only the hand model. This
class keeps that path exactly as-is and drops only the DDS publish/subscribe, so the
recorded `*_ee.qpos` values are produced by the identical code that would run on a real
robot.

Because there are no encoders, the reported hand *state* is the last commanded target. That
is a statement of fact rather than a simulation: the recorded `actions` are what the
retargeter produced, and on a robot-less station `states` carries no independent
information. For the headset-only workflow the trajectory of record is `actions` -- see
`phantom/phantom/xr_episode.py`'s `TRAJECTORY_SOURCE`.
"""

import os
import sys
import time
from multiprocessing import Array, Process

import numpy as np

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

import logging_mp

logger_mp = logging_mp.getLogger(__name__)

Dex3_Num_Motors = 7


class Dex3_1_VirtualController:
    """
    In-memory stand-in for `Dex3_1_Controller`.

    Matches the real constructor's signature and its output contract: it fills
    `dual_hand_state_array_out` and `dual_hand_action_array_out` with 14 values each
    (left 7, then right 7) in the Dex3 API joint order -- thumb_0..2, middle_0..1,
    index_0..1 -- exactly as `hand_retargeting.py`'s `*_dex3_api_joint_names` defines.

    Args:
        left_hand_array_in / right_hand_array_in: [input] 75-float XR hand skeleton arrays
            (25 joints x 3), written by the main loop from televuer.
        dual_hand_data_lock: guards the two output arrays.
        dual_hand_state_array_out: [output] 14 floats, left then right hand state.
        dual_hand_action_array_out: [output] 14 floats, left then right hand action.
        fps: retargeting rate.
        Unit_Test: selects the unit-test retargeting config (different relative asset path).
        simulation_mode: accepted for signature compatibility; no effect.
        xr_motion_data_ready_in: [input] gate; retargeting is skipped until XR data arrives,
            so the hands hold their last pose instead of snapping to a garbage target.
    """

    def __init__(self, left_hand_array_in, right_hand_array_in, dual_hand_data_lock=None,
                 dual_hand_state_array_out=None, dual_hand_action_array_out=None,
                 fps: float = 100.0, Unit_Test: bool = False, simulation_mode: bool = False,
                 xr_motion_data_ready_in=None):
        logger_mp.info("Initialize Dex3_1_VirtualController (no robot, no DDS)...")

        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        self.running = True

        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3)
        else:
            self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3_Unit_Test)

        # Kept for interface parity with the real controller, which populates these from
        # DDS. Here they mirror the last command.
        self.left_hand_state_array = Array('d', Dex3_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Dex3_Num_Motors, lock=True)

        self.hand_control_process = Process(
            target=self.control_process,
            args=(left_hand_array_in, right_hand_array_in, self.left_hand_state_array,
                  self.right_hand_state_array, dual_hand_data_lock,
                  dual_hand_state_array_out, dual_hand_action_array_out,
                  xr_motion_data_ready_in),
        )
        self.hand_control_process.daemon = True
        self.hand_control_process.start()

        logger_mp.info("Initialize Dex3_1_VirtualController OK!")

    def control_process(self, left_hand_array_in, right_hand_array_in,
                        left_hand_state_array, right_hand_state_array,
                        dual_hand_data_lock=None, dual_hand_state_array_out=None,
                        dual_hand_action_array_out=None, xr_motion_data_ready_in=None):
        """
        Retarget XR hand skeletons to Dex3 joint targets at `fps`.

        Identical to the real `control_process` except that there is no `ctrl_dual_hand`
        publish and no encoder read -- the achieved state is the commanded target.
        """
        self.running = True
        left_q_target = np.zeros(Dex3_Num_Motors)
        right_q_target = np.zeros(Dex3_Num_Motors)

        try:
            while self.running:
                start_time = time.time()

                with left_hand_array_in.get_lock():
                    left_hand_data = np.array(left_hand_array_in[:]).reshape(25, 3).copy()
                with right_hand_array_in.get_lock():
                    right_hand_data = np.array(right_hand_array_in[:]).reshape(25, 3).copy()

                if xr_motion_data_ready_in is not None:
                    with xr_motion_data_ready_in.get_lock():
                        xr_motion_data_ready = xr_motion_data_ready_in.value
                else:
                    xr_motion_data_ready = True

                if xr_motion_data_ready:
                    # Retargeting consumes inter-joint VECTORS, not absolute positions, so
                    # it is invariant to where the hand sits in the XR world frame.
                    ref_left_value = (left_hand_data[self.hand_retargeting.left_indices[1, :]]
                                      - left_hand_data[self.hand_retargeting.left_indices[0, :]])
                    ref_right_value = (right_hand_data[self.hand_retargeting.right_indices[1, :]]
                                       - right_hand_data[self.hand_retargeting.right_indices[0, :]])

                    left_q_target = self.hand_retargeting.left_retargeting.retarget(
                        ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(
                        ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                # No encoders: the achieved state IS the last command.
                left_hand_state_array[:] = left_q_target
                right_hand_state_array[:] = right_q_target

                state_data = np.concatenate((left_q_target, right_q_target))
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array_out and dual_hand_action_array_out:
                    with dual_hand_data_lock:
                        dual_hand_state_array_out[:] = state_data
                        dual_hand_action_array_out[:] = action_data

                time.sleep(max(0, (1 / self.fps) - (time.time() - start_time)))
        finally:
            logger_mp.info("Dex3_1_VirtualController control_process exiting.")

    def close(self):
        """Stop the retargeting process."""
        self.running = False
        if self.hand_control_process.is_alive():
            self.hand_control_process.terminate()
            self.hand_control_process.join(timeout=1.0)
