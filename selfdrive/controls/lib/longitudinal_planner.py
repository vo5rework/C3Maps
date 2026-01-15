#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.events import Events

# --- SUNNYPILOT MAP IMPORT ---
# We try to import the controller. If the files aren't found, it falls back to vision-only.
try:
  from openpilot.selfdrive.controls.lib.sunnypilot.speed_limit_controller import SpeedLimitController
except ImportError:
  cloudlog.warning("Map Data: SpeedLimitController not found! Running in Vision-Only mode.")
  SpeedLimitController = None

# Physics and MPC Constants
LON_MPC_STEP = 0.2
A_CRUISE_MIN = -4.0
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Thresholds for Vision Logic
CURVE_PEAK_THRESHOLD = 0.0020
AREA_THRESHOLD = 0.012

def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """Prevents acceleration when lateral Gs are high"""
  a_total_max = interp(v_ego, [20., 40.], [1.5, 2.8])
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  return [a_target[0], min(a_target[1], a_x_allowed)]

class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 1.0, DT_MDL)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.param_read_counter = 0
    self.read_param()
    self.personality = log.LongitudinalPersonality.standard
    
    # Custom Turn Logic States
    self.v_turn_filter = FirstOrderFilter(init_v, 0.2, DT_MDL)
    self.curve_detected = False

    # --- MAP CONTROLLER INITIALIZATION ---
    self.events = Events()
    self.slc = None
    if SpeedLimitController is not None:
      # Sunnypilot's SLC requires CP to be passed during init
      self.slc = SpeedLimitController(CP)

  def read_param(self):
    try:
      p = self.params.get('LongitudinalPersonality')
      self.personality = int(p) if p is not None else log.LongitudinalPersonality.standard
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

  @staticmethod
  def parse_model(model_msg, model_error, v_ego, v_turn_filter, roll, lead_status, lead_v):
    """
    YOUR CUSTOM LOGIC:
    Combines Vision Anticipation + Roll Compensation + Lead Probing
    """
    x = np.zeros(len(T_IDXS_MPC))
    v = np.zeros(len(T_IDXS_MPC))
    a = np.zeros(len(T_IDXS_MPC))
    j = np.zeros(len(T_IDXS_MPC))
    
    if (len(model_msg.position.x) == 33 and len(model_msg.velocity.x) == 33):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v_raw = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      
      v_corrected = np.maximum(v_raw, v_ego - 1.5) - model_error
      raw_curv = np.abs(np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)) / np.clip(v_corrected, 0.3, 100.0)
      
      # 1. FAR LOOKAHEAD (Vision Anticipation)
      num_idx = len(raw_curv)
      far_curv_peak = np.max(raw_curv[min(10, num_idx-1):min(32, num_idx)])
      anticipatory_slowdown = interp(far_curv_peak, [0.0008, 0.003], [1.0, 0.78])

      # 2. LEAD PROBING (Use lead car behavior)
      lead_slowdown = 1.0
      if lead_status and lead_v < (v_ego * 0.85):
         lead_slowdown = 0.85 
      
      # 3. ROLL COMPENSATION (Superelevation Check for Tesla EPS)
      roll_penalty = 1.0
      if abs(roll) < 0.02 and far_curv_peak > 0.001: 
          roll_penalty = 0.90

      curve_area = np.sum(raw_curv[:25]) * 0.2
      max_curv_ahead = np.max(raw_curv[2:20]) 
      
      if curve_area > AREA_THRESHOLD or max_curv_ahead > CURVE_PEAK_THRESHOLD:
        lat_stress_factor = (v_ego ** 2) * max_curv_ahead
        torque_multiplier = interp(lat_stress_factor, [0.015, 0.05], [1.0, 0.65])
        dynamic_multiplier = interp(max_curv_ahead, [0.0015, 0.008], [1.0, 0.70])
        
        final_multiplier = dynamic_multiplier * torque_multiplier * anticipatory_slowdown * roll_penalty * lead_slowdown
        
        max_v_curve = final_multiplier * np.sqrt(2.1 / (max_curv_ahead + 1e-4))
        v_turn_filter.k = 15.0 if max_v_curve < v_turn_filter.x else 0.3
        v = np.minimum(v_turn_filter.update(max_v_curve), v_corrected)
        return x, v, a, j, True
      
      # Non-curve state: still apply subtle penalties for smoother transitions
      v = v_corrected
      combined_slowdown = anticipatory_slowdown * roll_penalty * lead_slowdown
      if combined_slowdown < 1.0:
        v = np.minimum(v * combined_slowdown, v)
        
      return x, v, a, j, (combined_slowdown < 0.98)
      
    return x, v, a, j, False

  def update(self, sm):
    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_cruise = sm['controlsState'].vCruise * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    # --- DATA GATHERING ---
    roll = sm['liveParameters'].roll
    lead_status = sm['radarState'].leadOne.status
    lead_v = sm['radarState'].leadOne.vLead
    enabled = sm['controlsState'].enabled
    a_ego = sm['carState'].aEgo

    # 1. UPDATE MAP LOGIC (Sunnypilot Integration)
    map_limit = 0.0
    if self.slc:
      # We create a new Events object to capture alerts from the SLC
      self.events = Events()
      # Pass the required data to the Sunnypilot controller
      self.slc.update(enabled, v_ego, a_ego, sm, v_cruise, self.events)
      
      if self.slc.is_active:
        # Use the "offseted" limit (includes user +/- settings if ported) or raw
        map_limit = self.slc.speed_limit_offseted
    
    # 2. RUN VISION LOGIC (Your Custom Tuning)
    x, v, a, j, self.curve_detected = self.parse_model(sm['modelV2'], self.v_model_error, v_ego, self.v_turn_filter, roll, lead_status, lead_v)

    # 3. BLEND: Take the lowest of Vision vs. Map
    # Only if the map has a valid limit (usually > 1.0 m/s)
    if map_limit > 1.0:
        v = np.minimum(v, map_limit)

    # --- TESLA REGEN WEIGHTS ---
    # We apply your custom MPC weights if *either* vision sees a curve 
    # *OR* the map is enforcing a limit (like a sharp map curve or drop in speed limit)
    is_map_slowing = (map_limit > 1.0 and map_limit < v_cruise)
    
    if self.curve_detected or is_map_slowing:
      # Aggressive weights: Priority on V (2.0), low cost on A (5.0) to use regen
      self.mpc.set_weights(prev_accel_constraint, personality=self.personality, weights=[2.0, 5.0, 40.0])
      self.mpc.set_accel_limits(accel_limits_turns[0], min(self.a_desired - 0.3, -0.1))
    else:
      self.mpc.set_weights(prev_accel_constraint, personality=self.personality)
      self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])

    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    v_target = v # v has already been clamped by vision and map
    
    # Final check to ensure we don't exceed cruise set speed
    v_target = np.minimum(v_target, v_cruise + 1.0) # +1.0 buffer for rounding

    self.mpc.update(sm['carState'], sm['radarState'], v_cruise, x, v_target, a, j, personality=self.personality)

    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(DT_MDL, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    lp = plan_send.longitudinalPlan
    lp.modelMonoTime = sm.logMonoTime['modelV2']
    lp.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    lp.speeds = self.v_desired_trajectory.tolist()
    lp.accels = self.a_desired_trajectory.tolist()
    lp.jerks = self.j_desired_trajectory.tolist()
    lp.hasLead = sm['radarState'].leadOne.status
    lp.longitudinalPlanSource = self.mpc.source
    lp.fcw = self.fcw
    lp.solverExecutionTime = self.mpc.solve_time
    lp.personality = self.personality
    
    # Publish Map Data if available (Useful for UI if you port that later)
    if self.slc:
       lp.visionTurnControllerState = self.slc.state # Optional: Map state info
       
    pm.send('longitudinalPlan', plan_send)