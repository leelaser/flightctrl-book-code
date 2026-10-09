#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
卷 2 第 9 章 · ArduPilot EKF3 对比分析 —— 配套实验

    实验 1  ★★ 状态向量并排对照表：同一个状态，两套协方差组织方式
    实验 2  ★★ 门限单位对照：sigma（PX4）vs 百分数（AP）+ 新息拒绝率
    实验 3  ★  融合顺序并排对照：controlFusionModes vs UpdateFilter
    实验 4  ★★ lane 选择器仿真：AP 的 errorScore/相对误差 vs PX4 的 selector

用法：
    python lesson15_ap_ekf3.py 1
    python lesson15_ap_ekf3.py 2
    python lesson15_ap_ekf3.py 3
    python lesson15_ap_ekf3.py 4
    python lesson15_ap_ekf3.py          # 全部

只需 numpy（matplotlib 可选，缺失时只打印数值不画图）。

【本脚本的原则】

  这一课是「读源码课」+「对比课」。所以本脚本里**每一个数字、每一条公式、
  每一行顺序，都是从源码里读出来的**，脚本里用 SRC 字典统一登记出处。

  源码根目录（本机，2026-09 核对过）：

      AP  = $SRC_APM
      PX4 = $SRC_PX4

  四个实验的主线都是同一句话：

      **同一件事，两种做法。**

  实验 1 是本章最核心的发现 —— AP 和 PX4 的状态向量**逐项一一对应**，
  但一套把四元数当成 4 个槽位存进 24x24 的 P，另一套把姿态误差压成 3 个
  自由度存进 23x23 的 P。**信息量完全相同，索引方式完全不同。**

  实验 2 揭示一个极容易踩的坑：同样的物理门限，
  PX4 的 EKF2_GPS_V_GATE 写 5.0（单位是 sigma），
  AP  的 EK3_VEL_I_GATE 写 500（单位是百分数）。
  两者在源码里的**除法形式**也不同，本脚本逐字对照。

  实验 4 会挖出一个「两个团队其实是同一个算法」的事实：
  AP 的 errorScore() 和 PX4 的 combined_test_ratio 是**同一个公式**，
  连相对误差阈值（0.2）、限幅（+-1.0）、切换阈值（0.5）都一模一样。

参考：
    AP/libraries/AP_NavEKF3/          （本章主对象）
    AP/libraries/AP_NavEKF/EKFGSF_yaw.cpp
    PX4-Autopilot/src/modules/ekf2/EKF/
    PX4-Autopilot/src/modules/ekf2/EKF2Selector.cpp
"""

import math
import sys
import unicodedata

import numpy as np

# ---------------------------------------------------------------------------
# 输出辅助（和本教程其它脚本一致）
# ---------------------------------------------------------------------------

W = 74


def head(title):
    print()
    print("=" * W)
    print("  " + title)
    print("=" * W)
    print()


def sub(title):
    print()
    print("  " + title)
    print("  " + "-" * (W - 4))
    print()


def tab(rows, headers, width=None):
    """打印表格。rows 是二维列表。"""
    cols = len(headers)
    if width is None:
        width = [max(len(str(r[i])) for r in [headers] + rows) + 2
                 for i in range(cols)]
    line = "  " + "".join("%-*s" % (width[i], headers[i]) for i in range(cols))
    print(line)
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join("%-*s" % (width[i], str(r[i])) for i in range(cols)))
    print()


def cw(s):
    """字符串的显示宽度：CJK 全角字符算 2 列。"""
    n = 0
    for ch in str(s):
        n += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
    return n


def wtab(rows, headers):
    """和 tab() 一样，但按显示宽度对齐。

    本章表格里中文很多，而 tab() 用的是 len()（CJK 只算 1 列），
    会错位。tab() 本身保持与本教程其它课一字不差，这里包一层。
    """
    cols = len(headers)
    width = [max(cw(r[i]) for r in [headers] + rows) + 2 for i in range(cols)]
    line = "  " + "".join(headers[i] + " " * (width[i] - cw(headers[i]))
                          for i in range(cols))
    print(line)
    print("  " + "-" * (sum(width) - 2))
    for r in rows:
        print("  " + "".join(str(r[i]) + " " * (width[i] - cw(r[i]))
                             for i in range(cols)))
    print()


# ---------------------------------------------------------------------------
# 源码出处登记表
#
#   ★ 本脚本的规矩：凡是在这里登记过的行，都是【真的打开文件看过】的。
#     没有登记的行号，脚本里一个都不许出现。
# ---------------------------------------------------------------------------

AP = "AP/libraries/AP_NavEKF3"      # EKF3 本体
APD = "AP/libraries"                # EKF-GSF 等在 AP_NavEKF/ 下
SRC = {
    # --- 状态向量 ---
    "ap_vec24":      (AP + "/AP_NavEKF3_core.h", 501,
                      "typedef VectorN<ftype,24> Vector24;"),
    "ap_state_struct": (AP + "/AP_NavEKF3_core.h", 536,
                        "struct state_elements { QuaternionF quat; ... }"),
    "ap_union":      (AP + "/AP_NavEKF3_core.h", 547,
                      "union { Vector24 statesArray; struct state_elements stateStruct; }"),
    "ap_matrix24":   (AP + "/AP_NavEKF3_core.h", 505,
                      "typedef VectorN<VectorN<ftype,24>,24> Matrix24;"),
    "ap_lim_init":   (AP + "/AP_NavEKF3_core.cpp", 286, "stateIndexLim = 23;"),
    "ap_lim_func":   (AP + "/AP_NavEKF3_Control.cpp", 191,
                      "void NavEKF3_core::updateStateIndexLim()"),
    "ap_lim_body":   (AP + "/AP_NavEKF3_Control.cpp", 193, "if (inhibitWindStates) {...}"),
    "ap_quat_comment": (AP + "/AP_NavEKF3_core.cpp", 758,
                        "the quaternion is not used by the EKF equations ..."),
    "ap_strapdown":  (AP + "/AP_NavEKF3_core.cpp", 765,
                      "void NavEKF3_core::UpdateStrapdownEquationsNED()"),
    "ap_covpred":    (AP + "/AP_NavEKF3_core.cpp", 1030,
                      "void NavEKF3_core::CovariancePrediction(Vector3F *rotVarVecPtr)"),
    "ap_deriv_copy": (AP + "/derivation/main.py", 2,
                      "# Copied from https://github.com/PX4/ecl/commit/264c8c4e8681704e4719d0a03b848df8617c0863"),
    "ap_paramgen":   (APD + "/AP_NavEKF3/../../Tools/scripts/build_parameters.sh", 1,
                      "AP 用 Tools/scripts/build_parameters.sh 扫 @Param 注释生成参数元数据"),
    "ap_deriv_p00":  (AP + "/derivation/generated/covariance_generated.cpp", 191,
                      "nextP[0][0] = ..."),
    "px4_state_h":   ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h", 12,
                      "struct StateSample { Quaternionf quat_nominal; ... }"),
    "px4_state_data": ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h", 22,
                       "Vector<float,24> Data() const"),
    "px4_state_idx": ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h", 43,
                      "namespace State { IdxDof quat_nominal{0,3}; ... size{23}; }"),
    "px4_state_size": ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h", 52,
                       "static constexpr uint8_t size{23};"),
    "px4_cov_getter": ("PX4/src/modules/ekf2/EKF/ekf.h", 239,
                       "const matrix::SquareMatrix<float, State::size> &covariances()"),
    "px4_aidsrc_msg": ("PX4/src/modules/ekf2/EKF/ekf.h", 670,
                       "estimator_aid_source2d_s _aid_src_gnss_pos{}; ..."),
    "px4_aidsrc_cnt": ("PX4/src/modules/ekf2/EKF/ekf.h", 585,
                       "estimator_aid_source2d_s _aid_src_drag{};（这类成员共 21 个）"),
    "px4_velaid":    ("PX4/src/modules/ekf2/EKF/velocity_fusion.cpp", 55,
                      "updateVelocityAidSrcStatus(...) 填 innovation / test_ratio"),
    "px4_velaid_h":  ("PX4/src/modules/ekf2/EKF/velocity_fusion.cpp", 36,
                      "void Ekf::updateHorizontalVelocityAidSrcStatus(time_us, obs, obs_var, innov_gate, aid_src)"),
    "px4_pub_aidsrc": ("PX4/src/modules/ekf2/EKF2.cpp", 864,
                       "void EKF2::PublishAidSourceStatus(const hrt_abstime &timestamp)"),
    "px4_pub_helper": ("PX4/src/modules/ekf2/EKF2.hpp", 252,
                       "template <typename T> PublishAidSourceStatus(status, last, pub)"),
    "px4_pub_gnssvel": ("PX4/src/modules/ekf2/EKF2.cpp", 905,
                        "PublishAidSourceStatus(_ekf.aid_src_gnss_vel(), ...)"),
    "px4_deriv_sola": ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py", 35,
                       "Sola, Joan. Quaternion kinematics for the error-state Kalman filter."),
    "px4_deriv_theta": ("PX4/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py", 151,
                        "state_t[quat_nominal] = Rot3(Quaternion(xyz=theta/2, w=1)) * q"),

    # --- 融合顺序 ---
    "px4_ctrl_fusion": ("PX4/src/modules/ekf2/EKF/control.cpp", 46,
                        "void Ekf::controlFusionModes(const imuSample &imu_delayed)"),
    "px4_ctrl_seq":  ("PX4/src/modules/ekf2/EKF/control.cpp", 105,
                      "controlMagFusion() ... updateDeadReckoningStatus()"),
    "ap_update_filter": (AP + "/AP_NavEKF3_core.cpp", 646,
                         "void NavEKF3_core::UpdateFilter(bool predict)"),
    "ap_update_seq": (AP + "/AP_NavEKF3_core.cpp", 664,
                      "controlFilterModes() ... updateFilterStatus()"),
    "ap_5s_reset":   (AP + "/AP_NavEKF3_core.cpp", 736,
                      "filterStatus.value == 0 && ... > 5000 && !armed -> forced reset"),
    "ap_frames_pred": (AP + "/AP_NavEKF3.cpp", 767,
                       "_framesPerPrediction = uint8_t(EKF_TARGET_DT / frameTime + 0.5);"),
    "ap_cpu_budget": (AP + "/AP_NavEKF3.cpp", 917,
                      "allow_state_prediction = false（CPU 预算不够时跳过预测）"),
    "ap_target_dt":  (AP + "/AP_NavEKF3_core.h", 59, "#define EKF_TARGET_DT 0.012f"),
    "px4_predict_us": ("PX4/src/modules/ekf2/module.yaml", 10,
                       "EKF2_PREDICT_US default 10000"),
    "px4_of_gate":   ("PX4/src/modules/ekf2/module.yaml", 733,
                      "EKF2_OF_GATE default 3.0 SD"),
    "px4_of_ctrl":   ("PX4/src/modules/ekf2/module.yaml", 692,
                      "EKF2_OF_CTRL default 0"),
    "px4_hdg_gate":  ("PX4/src/modules/ekf2/module.yaml", 362,
                      "EKF2_HDG_GATE default 2.6 SD"),
    "px4_mag_gate":  ("PX4/src/modules/ekf2/module.yaml", 372,
                      "EKF2_MAG_GATE default 3.0 SD"),
    "px4_baro_gate": ("PX4/src/modules/ekf2/module.yaml", 432,
                      "EKF2_BARO_GATE default 5.0 SD"),
    "px4_gpsp_gate": ("PX4/src/modules/ekf2/module.yaml", 464,
                      "EKF2_GPS_P_GATE default 5.0 SD"),
    "px4_gpsv_gate": ("PX4/src/modules/ekf2/module.yaml", 474,
                      "EKF2_GPS_V_GATE default 5.0 SD"),
    "px4_beta_gate": ("PX4/src/modules/ekf2/module.yaml", 343,
                      "EKF2_BETA_GATE default 5.0 SD"),
    "px4_flow_gate_c": ("PX4/src/modules/ekf2/EKF/common.h", 454,
                        "flow_innov_gate{3.0f}"),
    "px4_gates_c":   ("PX4/src/modules/ekf2/EKF/common.h", 294,
                      "baro_innov_gate{5.0f} ... tas_innov_gate{5.0f}"),

    # --- 门限 ---
    "px4_gate_1d":   ("PX4/src/modules/ekf2/EKF/ekf.h", 1188,
                      "status.test_ratio = sq(innovation) / (sq(gate) * innov_var)"),
    "px4_gate_3d":   ("PX4/src/modules/ekf2/EKF/ekf.h", 1207,
                      "status.test_ratio[i] = sq(innovation[i]) / (sq(gate) * innov_var[i])"),
    "px4_gates":     ("PX4/src/modules/ekf2/EKF/common.h", 322,
                      "gps_pos_innov_gate{5.0f}; gps_vel_innov_gate{5.0f}; ..."),
    "ap_vel_gate":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 844,
                      "velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01*gate,1.0)))"),
    "ap_vel_imax":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 826,
                      "uint8_t imax = 2; ... if (...) imax = 1;"),
    "ap_vel_idx":    (AP + "/AP_NavEKF3_PosVelFusion.cpp", 838,
                      "stateIndex = i + 4;   // 速度状态 4/5/6"),
    "ap_fault_bits": (AP + "/AP_NavEKF3_PosVelFusion.cpp", 1084,
                      "faultStatus.bad_nvel/bad_evel/.../bad_dpos"),
    "ap_healthy2":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 1065,
                      "if (healthyFusion) { P -= KHP; ForceSymmetry(); ConstrainVariances(); }"),
    "ap_pos_gate":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 767,
                      "maxPosInnov2 = sq(MAX(0.01*_gpsPosInnovGate, 1.0)) * (var3+var4)"),
    "ap_hgt_gate":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 887,
                      "hgtTestRatio = sq(innov[5]) / (sq(MAX(0.01*_hgtInnovGate,1.0)) * var[5])"),
    "ap_mag_gate":   (AP + "/AP_NavEKF3_MagFusion.cpp", 561,
                      "magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f*_magInnovGate,1.0f)) * var)"),
    "ap_gate_defaults": (AP + "/AP_NavEKF3.cpp", 109,
                         "VEL_I_GATE 500 / POS_I_GATE 500 / HGT_I_GATE 500 / MAG_I_GATE 300"),

    # --- 序贯融合 ---
    "ap_seq_loop":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 951,
                      "for (obsIndex=0; obsIndex<=5; obsIndex++)"),
    "ap_gyro_obs":   (AP + "/AP_NavEKF3_PosVelFusion.cpp", 997,
                      "poorObservability = fabsF(prevTnb.a.z) > M_SQRT1_2;"),
    "ap_healthy_fusion": (AP + "/AP_NavEKF3_PosVelFusion.cpp", 1059,
                          "if (KHP[i][i] > P[i][i]) healthyFusion = false;"),
    "px4_direct_fuse": ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 827,
                        "bool Ekf::fuseDirectStateMeasurement(...)"),
    "px4_joseph":    ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 851,
                      "Efficient implementation of the Joseph stabilized covariance update"),
    "px4_inhibit":   ("PX4/src/modules/ekf2/EKF/ekf.h", 904,
                      "void clearInhibitedStateKalmanGains(VectorState &K)"),
    "px4_cov_limit": ("PX4/src/modules/ekf2/EKF/covariance.cpp", 212,
                      "void Ekf::constrainStateVariances()"),
    "ap_cov_limit":  (AP + "/AP_NavEKF3_core.cpp", 1887,
                      "void NavEKF3_core::ConstrainVariances()"),
    "ap_state_limit": (AP + "/AP_NavEKF3_core.cpp", 2006,
                       "void NavEKF3_core::ConstrainStates()"),

    # --- 位置重置 / 超时 ---
    "ap_glitch":     (AP + "/AP_NavEKF3_PosVelFusion.cpp", 791,
                      "posVarianceIsTooLarge = (_gpsGlitchRadiusMax>0) && (P77+P88) > sq(GLITCH_RAD)"),
    "ap_zero_rows":  (AP + "/AP_NavEKF3_PosVelFusion.cpp", 800,
                      "zeroRows(P,7,8); zeroCols(P,7,8);"),
    "ap_reset_pos":  (AP + "/AP_NavEKF3_PosVelFusion.cpp", 794,
                      "ResetPosition(resetDataSource::DEFAULT);"),
    "ap_aiding":     (AP + "/AP_NavEKF3_Control.cpp", 231,
                      "void NavEKF3_core::setAidingMode()"),
    "ap_aid_switch": (AP + "/AP_NavEKF3_Control.cpp", 283,
                      "switch (PV_AidingMode) { case AID_NONE: ... }"),
    "ap_aid_loss":   (AP + "/AP_NavEKF3_Control.cpp", 374,
                      "if (attAidLossCritical) PV_AidingMode = AID_NONE;"),
    "ap_aid_enum":   (AP + "/AP_NavEKF3_core.h", 1272,
                      "AID_ABSOLUTE=0, AID_NONE=1, AID_RELATIVE=2"),
    "ap_retry":      (AP + "/AP_NavEKF3.h", 465,
                      "posRetryTimeUseVel_ms=10000; 466: posRetryTimeNoVel_ms=7000; 464: tiltDriftTimeMax_ms=15000"),
    "ap_magcal_def": (AP + "/AP_NavEKF3.cpp", 35,
                      "Copter 分支（第 18 行 #if APM_BUILD_COPTER_OR_HELI）里 MAG_CAL_DEFAULT = 3"),
    "px4_deadreck":  ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 580,
                      "void Ekf::updateDeadReckoningStatus()"),
    "px4_deadreck_h": ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 586,
                       "void Ekf::updateHorizontalDeadReckoningstatus()"),
    "px4_noreck":    ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 609,
                      "_control_status.flags.inertial_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding"),
    "px4_deadreck_w": ("PX4/src/modules/ekf2/EKF/ekf_helper.cpp", 622,
                       "ECL_WARN(dead reckon time exceeded)"),
    "px4_noaid_tout": ("PX4/src/modules/ekf2/module.yaml", 583,
                       "EKF2_NOAID_TOUT default 5000000 us"),

    # --- 磁力计 ---
    "ap_mag_health": (AP + "/AP_NavEKF3_MagFusion.cpp", 565,
                      "magHealth = (magTestRatio[0]<1 && [1]<1 && [2]<1)"),
    "ap_mag_bad":    (AP + "/AP_NavEKF3_MagFusion.cpp", 525,
                      "if (varInnovMag[0] >= R_MAG) bad_xmag=false; else CovarianceInit(); return;"),
    "ap_mag_cal":    (AP + "/AP_NavEKF3.cpp", 252,
                      "AP_GROUPINFO(MAG_CAL, 14, ...) —— 默认值按机型：copter 3(第 35 行) / plane 0(第 87 行) / rover 2(第 61 行)"),
    "ap_mag_ef_lim": (AP + "/AP_NavEKF3.cpp", 604,
                      "EK3_MAG_EF_LIM default 50 mGauss"),
    "px4_magfield":  ("PX4/src/modules/ekf2/EKF/mag_control.cpp", 278,
                      "bool Ekf::checkMagField(const Vector3f &mag_sample)"),
    "px4_mag_dist":  ("PX4/src/modules/ekf2/EKF/mag_control.cpp", 293,
                      "_control_status.flags.mag_field_disturbed = true;"),
    "px4_wmm":       ("PX4/src/modules/ekf2/EKF/mag_control.cpp", 301,
                      "average_earth_mag_field_strength = 0.45f Gauss, gate 0.40f"),
    "px4_magcheck":  ("PX4/src/modules/ekf2/EKF/mag_control.cpp", 297,
                      "MagCheckMask::FORCE_WMM"),

    # --- GSF 偏航 ---
    "ap_gsf_pred":   (AP + "/AP_NavEKF3_Control.cpp", 765,
                      "void NavEKF3_core::runYawEstimatorPrediction()"),
    "ap_gsf_corr":   (AP + "/AP_NavEKF3_Control.cpp", 787,
                      "void NavEKF3_core::runYawEstimatorCorrection()"),
    "ap_gsf_models": (APD + "/AP_NavEKF/EKFGSF_yaw.cpp", 550,
                      "yaw_increment = M_2PI / N_MODELS_EKFGSF; weights = 1/N"),
    "ap_gsf_n":      (APD + "/AP_NavEKF/AP_Nav_Common.h", 83,
                      "#define N_MODELS_EKFGSF 5U"),
    "ap_gsf_fuse":   (APD + "/AP_NavEKF/EKFGSF_yaw.cpp", 136,
                      "void EKFGSF_yaw::fuseVelData(const Vector2F &vel, const ftype velAcc)"),
    "ap_gsf_update": (AP + "/AP_NavEKF3_Control.cpp", 784,
                      "yawEstimator->update(delAng, delVel, ...) 每个 IMU 步都跑"),
    "ap_gsf_cont":   (AP + "/AP_NavEKF3_MagFusion.cpp", 258,
                      "fuseEulerYaw(yawFusionMethod::GSF) —— GSF 偏航被当成常规测量融合"),
    "ap_gsf_throt":  (AP + "/AP_NavEKF3_MagFusion.cpp", 254,
                      "if (imuSampleTime_ms - lastSynthYawTime_ms > 140) —— 140 ms 节流"),
    "ap_gsf_ymeth":  (AP + "/AP_NavEKF3_core.h", 675,
                      "enum class yawFusionMethod { MAGNETOMETER=0, GPS=1, GSF=2, STATIC=3, PREDICTED=4, EXTNAV=5 }"),
    "ap_gsf_rst":    (AP + "/AP_NavEKF3_MagFusion.cpp", 1544,
                      "bool NavEKF3_core::EKFGSF_resetMainFilterYaw(bool emergency_reset)"),
    "ap_gsf_usemask": (AP + "/AP_NavEKF3_MagFusion.cpp", 1548,
                      "if ((yawEstimator == nullptr) || !(frontend->_gsfUseMask & (1U<<core_index)))"),
    "ap_gsf_rstmax": (AP + "/AP_NavEKF3.cpp", 631,
                      "AP_GROUPINFO(GSF_RST_MAX, 60, NavEKF3, _gsfResetMaxCount, 2)"),
    "ap_gsf_txt":    (AP + "/AP_NavEKF3_MagFusion.cpp", 1573,
                      "GCS_SEND_TEXT(WARNING, EKF3 IMU%u emergency yaw reset)"),
    "ap_gsf_validh": (AP + "/AP_NavEKF3_core.h", 80,
                      "#define GSF_YAW_VALID_HISTORY_THRESHOLD 5"),
    "ap_srcyaw":     (APD + "/AP_NavEKF/AP_NavEKF_Source.h", 39,
                      "enum class SourceYaw { NONE=0, COMPASS=1, GPS=2, GPS_COMPASS_FALLBACK=3, EXTNAV=6, GSF=8 }"),
    "ap_src1yaw":    (APD + "/AP_NavEKF/AP_NavEKF_Source.cpp", 59,
                      "AP_GROUPINFO(1_YAW, 5, AP_NavEKF_Source, _source_set[0].yaw, SourceYaw::COMPASS)"),
    "px4_gsf_reset": ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 483,
                      "bool Ekf::resetYawToEKFGSF()"),
    "px4_gsf_avail": ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 460,
                      "isYawEmergencyEstimateAvailable(): yawVar < sq(EKFGSF_yaw_err_max)"),
    "px4_gsf_pred":  ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 54,
                      "_yawEstimator.predict(imu_delayed.delta_ang, ...) 每个 IMU 步都跑"),
    "px4_gsf_gyro":  ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 50,
                      "_yawEstimator.setGyroBias(getGyroBias(), vehicle_at_rest)"),
    "px4_gsf_call1": ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 239,
                      "if (resetYawToEKFGSF()) { ECL_INFO(GPS yaw aligned using IMU); }"),
    "px4_gsf_call2": ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 246,
                      "bool Ekf::tryYawEmergencyReset()（内部 255 行调 resetYawToEKFGSF，256 行 ECL_WARN）"),
    "px4_gsf_magf":  ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 261,
                      "_control_status.flags.mag_fault = true; 应急重置后停用磁力计"),
    "px4_gsf_fail":  ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 480,
                      "return fabsf(yaw_error) > math::radians(25.f);"),
    "px4_gsf_gate":  ("PX4/src/modules/ekf2/EKF/gps_control.cpp", 489,
                      "// don't allow reset if there's just been a yaw reset"),

    # --- lane / selector ---
    "ap_score":      (AP + "/AP_NavEKF3_Outputs.cpp", 42,
                      "score = MAX(score, 0.5f*(velTestRatio + posTestRatio)); score = MAX(score, hgtTestRatio);"),
    "ap_score_mag":  (AP + "/AP_NavEKF3_Outputs.cpp", 57,
                      "score = MAX(score, 0.3f * (magTestRatio.x+y+z))"),
    "ap_rel_err":    (AP + "/AP_NavEKF3.cpp", 1076,
                      "void NavEKF3::updateCoreRelativeErrors()"),
    "ap_rel_body":   (AP + "/AP_NavEKF3.cpp", 1083,
                      "if (error > 0 || error < -MAX(_err_thresh,0.05)) coreRelativeErrors[i] += error;"),
    "ap_err_lim":    (AP + "/AP_NavEKF3.cpp", 1085,
                      "constrain_ftype(coreRelativeErrors[i], -CORE_ERR_LIM, CORE_ERR_LIM)"),
    "ap_err_thresh": (AP + "/AP_NavEKF3.cpp", 639,
                      "AP_GROUPINFO(ERR_THRESH, 61, ..., 0.2)"),
    "ap_ok":         (AP + "/AP_NavEKF3.cpp", 1,
                      "（本章主要引用文件之一：AP_NavEKF3.cpp）"),
    "ap_better":     (AP + "/AP_NavEKF3.cpp", 962,
                      "betterCore = altCoreError <= -BETTER_THRESH;"),
    "ap_switch":     (AP + "/AP_NavEKF3.cpp", 979,
                      "altCoreAvailable && (primaryErrorScore > 1.0f || !healthy || betterCore)"),
    "ap_better_th":  (AP + "/AP_NavEKF3.h", 522,
                      "#define CORE_ERR_LIM 1 / BETTER_THRESH 0.5"),
    "ap_corescore":  (AP + "/AP_NavEKF3.cpp", 873,
                      "bool NavEKF3::coreBetterScore(uint8_t new_core, uint8_t current_core)"),
    "ap_laneswitch": (AP + "/AP_NavEKF3.cpp", 1014,
                      "void NavEKF3::checkLaneSwitch(void) -- altErrorScore < lowestErrorScore && < 0.9"),
    "ap_ls_guard":   (AP + "/AP_NavEKF3.cpp", 1019,
                      "if (lastLaneSwitch_ms != 0 && now - lastLaneSwitch_ms < 5000) return;  // don't switch twice in 5 seconds"),
    "ap_ls_cond":    (AP + "/AP_NavEKF3.cpp", 1033,
                      "if (altCoreAvailable && altErrorScore < lowestErrorScore && altErrorScore < 0.9)"),
    "ap_ls_avail":   (AP + "/AP_NavEKF3.cpp", 1031,
                      "altCoreAvailable = newCore.healthy() && have_aligned_yaw() && have_aligned_tilt()"),
    "ap_altloop":    (AP + "/AP_NavEKF3.cpp", 947,
                      "for (coreIndex...) { coreBetterScore(coreIndex, newPrimaryIndex) && >= 10 s }"),
    "ap_altavail":   (AP + "/AP_NavEKF3.cpp", 972,
                      "altCoreAvailable = newPrimaryIndex != primary;"),
    "ap_userprim":   (AP + "/AP_NavEKF3.cpp", 991,
                      "const uint8_t user_primary = ...; if (primary != user_primary && healthy && !armed) primary = user_primary;"),
    "ap_better_yaw": (AP + "/AP_NavEKF3.cpp", 967,
                      "betterCore |= newCore.have_aligned_yaw() && !oldCore.have_aligned_yaw();"),
    "px4_sel_prim_upd": ("PX4/src/modules/ekf2/EKF2Selector.cpp", 339,
                         "if (primary_updated) { ... }  // 只在 primary 更新时累加"),
    "px4_sel_prim_set": ("PX4/src/modules/ekf2/EKF2Selector.cpp", 278,
                         "if (i == _selected_instance) { primary_updated = true; }"),
    "px4_sel_gyro2": ("PX4/src/modules/ekf2/EKF2Selector.cpp", 352,
                      "relative_test_ratio < 1.f 时提高 selector_status 发布率"),
    "ap_imumask":    (AP + "/AP_NavEKF3.cpp", 408,
                      "AP_GROUPINFO(IMU_MASK, 33, NavEKF3, _imuMask, HAL_EKF_IMU_MASK_DEFAULT)"),
    "ap_imumask_def": (AP + "/AP_NavEKF3.cpp", 130,
                       "#define HAL_EKF_IMU_MASK_DEFAULT 3  // Default to using two IMUs"),
    "ap_alloc":      (AP + "/AP_NavEKF3.cpp", 803,
                      "available_memory() < sizeof(NavEKF3_core)*num_cores + 4096 -> 0 cores"),
    "px4_sel_comb":  ("PX4/src/modules/ekf2/EKF2Selector.cpp", 294,
                      "combined_test_ratio = fmaxf(0.5f*(vel_test_ratio+pos_test_ratio), hgt_test_ratio)"),
    "px4_sel_rel":   ("PX4/src/modules/ekf2/EKF2Selector.cpp", 343,
                      "error_delta = combined[i] - combined[selected]"),
    "px4_sel_rel2":  ("PX4/src/modules/ekf2/EKF2Selector.cpp", 346,
                      "threshold = _gyro_fault_detected ? 0.0f : fmaxf(EKF2_SEL_ERR_RED, 0.05f)"),
    "px4_sel_lim":   ("PX4/src/modules/ekf2/EKF2Selector.hpp", 146,
                      "_rel_err_score_lim{1.0f}; _rel_err_thresh{0.5f}"),
    "px4_sel_switch": ("PX4/src/modules/ekf2/EKF2Selector.cpp", 757,
                       "if (!_instance[_selected_instance].healthy.get_state()) { ... }"),
    "px4_sel_else":  ("PX4/src/modules/ekf2/EKF2Selector.cpp", 765,
                      "} else if (lower_error_available && (...10_s... || warning...)) { SelectInstance(best_ekf_alternate); }"),
    "px4_sel_req":   ("PX4/src/modules/ekf2/EKF2Selector.cpp", 778,
                      "} else if (_request_instance.load() != INVALID_INSTANCE) { // 手动指定实例"),
    "px4_sel_red":   ("PX4/src/modules/ekf2/params_selector.yaml", 5,
                      "EKF2_SEL_ERR_RED default 0.2"),
    "px4_sel_max":   ("PX4/src/modules/ekf2/EKF2Selector.hpp", 60,
                      "EKF2_MAX_INSTANCES 2 (CONSTRAINED_MEMORY) / 9"),
}


def src(key):
    """返回一个源码出处的短字符串，形如 path:line。"""
    p, ln, _ = SRC[key]
    return "%s:%d" % (p, ln)


def src_banner(keys, title):
    """把一组出处打印成一个小清单。"""
    sub(title)
    for k in keys:
        p, ln, what = SRC[k]
        print("    %s" % ("%s:%d" % (p, ln)))
        print("        %s" % what)
    print()


# ---------------------------------------------------------------------------
# 实验 1：状态向量并排对照表
# ---------------------------------------------------------------------------

# (AP 起始索引, PX4 协方差索引, 名字, 浮点槽位, 自由度, 含义)
STATES = [
    (0,  0,  "quat",        4, 3, "姿态四元数 (body<-NED)"),
    (4,  3,  "velocity",    3, 3, "NED 速度 (m/s)"),
    (7,  6,  "position",    3, 3, "NED 位置 (m)"),
    (10, 9,  "gyro_bias",   3, 3, "机体陀螺零偏 (rad)"),
    (13, 12, "accel_bias",  3, 3, "机体加计零偏 (m/s)"),
    (16, 15, "earth_mag",   3, 3, "地球磁场 NED (Gauss)"),
    (19, 18, "body_mag",    3, 3, "机体磁场 (Gauss)"),
    (22, 21, "wind_vel",    2, 2, "水平风 NE (m/s)"),
]


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_from_rotvec(v):
    """exp(v/2) 的 Hamilton 四元数形式。v 是旋转向量（rad）。"""
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    ax = v / th
    return np.array([math.cos(th / 2.0),
                     ax[0] * math.sin(th / 2.0),
                     ax[1] * math.sin(th / 2.0),
                     ax[2] * math.sin(th / 2.0)])


def quat_G(q):
    """G = d(q (x) exp(dtheta/2)) / d(dtheta) 在 dtheta = 0 处的值（4x3）。

    推导（不跳步）：
        q (x) [1, d1, d2, d3]  （d = dtheta/2）
         0 分量: q0 - q1 d1 - q2 d2 - q3 d3
         1 分量: q1 + q0 d1 + q2 d3 - q3 d2
         2 分量: q2 - q1 d3 + q0 d2 + q3 d1
         3 分量: q3 + q1 d2 - q2 d1 + q0 d3
    对 d 求偏导再乘 1/2：

        G = 1/2 * [[-q1, -q2, -q3],
                   [ q0, -q3,  q2],
                   [ q3,  q0, -q1],
                   [-q2,  q1,  q0]]

    ★ 关键性质：G^T q = 0 —— 四元数的【径向】方向不在 G 的列空间里。
      这就是「4 个 float 只有 3 个自由度」的数学表述。
    """
    q0, q1, q2, q3 = q
    return 0.5 * np.array([
        [-q1, -q2, -q3],
        [q0, -q3, q2],
        [q3, q0, -q1],
        [-q2, q1, q0],
    ])


def exp1():
    head("实验 1  ★★ 状态向量并排对照表：同一份状态，两套协方差")

    print("""  本实验做三件事：

    ① 把 AP EKF3 和 PX4 EKF2 的状态向量逐项摆在一起
    ② 用数值证明「四元数 4 个 float 只有 3 个自由度」
    ③ 算清两套协方差组织方式的存储代价，以及 AP 的 stateIndexLim 机制
""")

    src_banner(["ap_vec24", "ap_state_struct", "ap_union",
                "px4_state_h", "px4_state_data", "px4_state_idx"],
               "① 先看两边的源码怎么定义的")

    # ---- ① 并排表 ----
    sub("② 逐项并排对照（索引 / 槽位 / 自由度 / 含义）")

    rows = []
    for ap_i, px4_i, name, slots, dof, desc in STATES:
        rows.append([ap_i, slots, px4_i, slots, name, dof, desc])
    wtab(rows, ["AP起点", "槽位", "PX4起点", "槽位", "名字", "DOF", "含义"])

    ap_total_slots = sum(s for _, _, _, s, _, _ in STATES)
    ap_total_dof = sum(d for _, _, _, _, d, _ in STATES)
    px4_cov_idx = [p for _, p, _, _, _, _ in STATES]
    px4_cov_n = max(px4_cov_idx) + STATES[-1][3]     # = 23，协方差阶数
    # ★★ 注意别把「状态向量存储」和「协方差阶数」混为一谈：
    #    PX4 的 state.h:22 是 matrix::Vector<float, 24> Data()，存储也是 24 个 float
    #    （四元数占 4 个槽位），只是协方差里折成了 3 个自由度。
    px4_slots = ap_total_slots                       # = 24，和 AP 完全一样
    px4_total_dof = ap_total_dof

    print("    AP  EKF3 : 状态向量 %d 个 float，协方差 %d x %d，自由度 %d" %
          (ap_total_slots, ap_total_slots, ap_total_slots, ap_total_dof))
    print("    PX4 EKF2 : 状态向量 %d 个 float，协方差 %d x %d，自由度 %d" %
          (px4_slots, px4_cov_n, px4_cov_n, px4_total_dof))
    print()

    sub("③ 这个表最该看出的一件事：逐项一一对应")

    print("""    两个团队独立写的两套代码，状态向量【完全一一对应】：

        quat / velocity / position / gyro_bias / accel_bias
        / earth_mag(I 系) / body_mag(B 系) / wind_vel

    连【顺序】都一样。差别的只有索引编号：
""")
    rows = []
    for ap_i, px4_i, name, slots, dof, _ in STATES:
        rows.append([name, ap_i, px4_i, ap_i - px4_i,
                     "是" if ap_i - px4_i == 1 else "起点"])
    wtab(rows, ["名字", "AP 索引", "PX4 协方差索引", "差", "是否恰好差 1"])

    print("""    ⇒ 从 velocity 往后，**AP 索引 = PX4 协方差索引 + 1**，一个不差。

       ★★ 原因就是实验 1 开头那张表的第 1 行：
            AP  给四元数留了 4 个【槽位】(0..3)
            PX4 给姿态误差只留了 3 个【自由度】(0..2)
          于是 AP 后面的所有状态索引都整体后移 1 位。

       ★ 这就是本章最有价值的发现：
         **同一个状态向量，两套完全不同的协方差组织方式。**
         而这两种组织方式的【信息量完全相同】—— 见下面 ④ 的数值证明。
""")

    # ---- ④ 4 个 float 只有 3 个自由度 ----
    sub("④ 数值证明：四元数 4 个 float 只有 3 个自由度")
    src_banner(["ap_quat_comment", "px4_deriv_sola", "px4_deriv_theta"],
               "先看两边的源码注释怎么说")

    q = np.array([math.cos(0.35), 0.0, math.sin(0.35), 0.0])
    q = q / np.linalg.norm(q)
    P3 = np.diag([0.010, 0.020, 0.030])       # 姿态误差 3x3 协方差 (rad^2)

    G = quat_G(q)
    P4 = G @ P3 @ G.T                          # 4x4 —— AP 的存储方式

    print("    取一个单位四元数 q = [%.4f, %.4f, %.4f, %.4f]" % tuple(q))
    print("    取姿态误差 3x3 协方差 P3 = diag(0.010, 0.020, 0.030) rad^2")
    print()

    print("    G = dq/d(dtheta) 的 4x3 矩阵（推导见本函数 docstring）：")
    for r in range(4):
        print("        [%+8.4f %+8.4f %+8.4f ]" % tuple(G[r]))
    print()

    gtq = G.T @ q
    print("    ★ 关键性质 G^T q = [%.2e, %.2e, %.2e]" % tuple(gtq))
    print("      ⇒ 四元数的【径向】方向（沿 q 本身）不在 G 的列空间里。")
    print("      ⇒ 单位范数约束把 4 维压成了 3 维。")
    print()

    ev = np.linalg.eigvalsh(P4)
    print("    P4 = G P3 G^T 的 4x4 特征值（升序）：")
    print("        [%+.6e, %+.6e, %+.6e, %+.6e]" % tuple(ev))
    print("        det(P4) = %+.6e" % float(np.linalg.det(P4)))
    print("        条件数  = %.6e" % float(np.linalg.cond(P4)))
    print()

    # 对应的（近）零特征向量
    w, V = np.linalg.eigh(P4)
    null_vec = V[:, 0]
    if float(np.dot(null_vec, q)) < 0:
        null_vec = -null_vec
    print("    最小特征值对应的特征向量 = [%+.6f, %+.6f, %+.6f, %+.6f]"
          % tuple(null_vec))
    print("    q 本身                  = [%+.6f, %+.6f, %+.6f, %+.6f]"
          % tuple(q))
    print("    两者夹角 = %.4f 度" %
          math.degrees(math.acos(min(1.0, abs(float(np.dot(null_vec, q)))))))
    print()
    print("""    ★★ 结论：P4 是【奇异】的（det = 0、特征值有一个 ≈ 0），
       而它的零空间方向正好是 q 自己 —— 也就是「四元数的模长」。

       四元数的模长不是自由度（归一化约束已经把它钉死了），
       所以 P4 里那一行一列的信息是【冗余】的。
""")

    # ---- ⑤ 蒙特卡洛验证 ----
    sub("⑤ 蒙特卡洛验证：直接采样，看经验协方差的秩")

    rng = np.random.default_rng(20260915)
    n = 200000

    print("    采样 %d 个 dtheta ~ N(0, s^2 * P3)，构造 q_i = q (x) exp(dtheta/2)。" % n)
    print("    中心取 q 本身（不是样本均值），这样和线性化点一致。")
    print()
    print("    然后看经验协方差 P4_mc 的 4 个特征值：")
    print("      如果 P4_mc 的秩真的是 3，第 4 个特征值应该【远小于】第 3 个。")
    print()

    rows = []
    ratios = {}
    for s in (1.0, 0.3, 0.1, 0.03):
        dth = rng.multivariate_normal(np.zeros(3), (s * s) * P3, size=n)
        qs = np.array([quat_mul(q, quat_from_rotvec(d)) for d in dth])
        dq = qs - q
        P4_mc = (dq.T @ dq) / (n - 1)
        ev_mc = np.linalg.eigvalsh(P4_mc)
        ratios[s] = ev_mc[0] / ev_mc[3]
        rows.append(["%.2f" % s, "%.3e" % ev_mc[3], "%.3e" % ev_mc[2],
                     "%.3e" % ev_mc[1], "%.3e" % ev_mc[0],
                     "%.3e" % ratios[s]])
    wtab(rows, ["扰动尺度 s", "lam1", "lam2", "lam3", "lam4（最小）",
                "lam4/lam3"])

    print("""    ★★ 看最后两列：lam4/lam3 随着扰动变小而【单调下降】：

        s = 1.00  ->  %.3e
        s = 0.30  ->  %.3e
        s = 0.10  ->  %.3e   （真实飞行时的量级）
        s = 0.03  ->  %.3e   （缩小了约 %.0f 倍）

      原因是四元数乘法是【非线性】的：
        q (x) exp(dtheta/2) = q + G dtheta + O(|dtheta|^2)
      那个 O(|dtheta|^2) 的二阶项正好落在 q 的径向
      （也就是 lam4 对应的方向上），大小正比于 s^2。
      s 缩小 %.0f 倍，二阶项缩小约 %.0f 倍。

      ⇒ 所以 lam4 不是"样本噪声"，而是**二阶非线性项**。
        把 s 推向 0，lam4/lam3 -> 0，
        **P4 的秩精确地是 3，不是 4。**

    ★ 顺便一个工程含义：
      真实 EKF 里姿态不确定度通常 < 10 度（s ~ 0.1），
      这时 lam4/lam3 已经小到 %.1e ——
      **在正常飞行条件下，AP 那个 4x4 姿态块几乎就是奇异的。**
      这解释了为什么两边都要不停地
      ConstrainVariances() + ForceSymmetry() 把 P 拉回正定：
      那一个冗余维度在数值上非常容易翻车。

      ⇒ AP 用 4x4 存储姿态块，代价不只是多 24 个浮点，
        还有一个【恒定的数值稳定性负担】。
        这是 PX4 后来改用 3 自由度 theta 误差状态的一个直接动机。
""" % (ratios[1.0], ratios[0.3], ratios[0.1], ratios[0.03],
       ratios[1.0] / ratios[0.03], 1.0 / 0.03, (1.0 / 0.03) ** 2,
       ratios[0.1]))

    # ---- ⑥ 存储代价 ----
    sub("⑥ 存储代价：两套组织方式到底差多少")

    n_ap = 24      # AP 的协方差阶数：四元数占 4 行 4 列
    n_px4 = 23     # PX4 的协方差阶数：四元数只占 3 行 3 列
    rows = [
        ["状态向量存储槽位（float）", 24, 24, "0"],
        ["协方差阶数 n", n_ap, n_px4, "-1"],
        ["稠密存储 n x n", n_ap * n_ap, n_px4 * n_px4, "-%d" % (n_ap**2 - n_px4**2)],
        ["对称存储 n(n+1)/2", n_ap * (n_ap + 1) // 2, n_px4 * (n_px4 + 1) // 2,
         "-%d" % (n_ap * (n_ap + 1) // 2 - n_px4 * (n_px4 + 1) // 2)],
        ["字节数（float32, 稠密）", 4 * n_ap * n_ap, 4 * n_px4 * n_px4,
         "-%d" % (4 * n_ap**2 - 4 * n_px4**2)],
        ["有效自由度", ap_total_dof, px4_total_dof, "0"],
    ]
    wtab(rows, ["项目", "AP EKF3", "PX4 EKF2", "差"])

    print("""    看前两行：
      · 状态向量【存储】两边都是 24 个 float（四元数占 4 个槽位）；
      · 协方差阶数差 1（24 vs 23）——这是唯一的差别。
      · 最后一行：两边的有效自由度完全一样，都是 23。

    多出来的那 24 个对称元素 = 一个额外的行 + 一个额外的列 = 24 个数，
    正好是「4 个槽位 vs 3 个自由度」多出来的那一份。

    ★ 这不是"谁抄谁"的问题 —— 两边的原作者是同一个人
      （Paul Riseborough）。差别来自**代码生成器的时代**：
""")
    src_banner(["ap_deriv_copy", "ap_deriv_p00"], "AP 的协方差预测是代码生成的")
    print("""    ⇒ AP 的 covariance_generated.cpp 里明文出现 nextP[0][0] .. nextP[3][3]，
      **姿态块占 4 行 4 列**。

    ⇒ PX4 在 2019 年前后用 SymForce 重写了整条推导链，
      derivation.py 里直接写 state_error[theta]（一个 3 向量），
      并且引用 Sola 的 ESKF 论文。**姿态块只占 3 行 3 列。**

    ★★ 但两边【估计的是同一件东西】：姿态误差（3 自由度）。
      AP 用 4 个槽位存这个 3 自由度量的做法，是从老版 ECL 继承下来的；
      代价是多算 24 个协方差元素，好处是索引改写时改动最小。
""")

    # ---- ⑦ stateIndexLim ----
    sub("⑦ AP 的 stateIndexLim：运行时可收缩的协方差窗口")
    src_banner(["ap_lim_init", "ap_lim_func", "ap_lim_body"],
               "AP 独有的一套优化：不用的状态就不算")

    rows = []
    for wind, mag, dvel, dang, lim in [
            (True,  True,  True,  True,  9),
            (True,  True,  True,  False, 12),
            (True,  True,  False, None,  15),
            (True,  False, None,  None,  21),
            (False, None,  None,  None,  23)]:
        rows.append(["是" if wind else "否",
                     "是" if mag else ("否" if mag is False else "—"),
                     "是" if dvel else ("否" if dvel is False else "—"),
                     "是" if dang else ("否" if dang is False else "—"),
                     lim, lim + 1, lim])
    wtab(rows, ["抑制风", "抑制磁", "抑制加计偏", "抑制陀螺偏",
               "stateIndexLim", "活跃槽位", "有效 DOF"])

    print("""    ★ stateIndexLim 是【闭区间上界】：循环写的是 i <= stateIndexLim。
      所以 stateIndexLim = 23 时覆盖 0..23 共 24 个槽位（全部）。

    ★★ 真正的信息是：**AP 的协方差矩阵大小是运行时可变的。**
      没有磁力计时就缩到 21（22 个槽位），
      连陀螺零偏都不估时缩到 9（10 个槽位）。
      协方差预测里那一堆 if (stateIndexLim > 9) / > 12 / > 15 / > 21
      就是这些收缩点在起作用。

    ★ PX4 走的是另一条路：协方差阶数在【编译期】就钉死为 23，
      不活跃的状态靠 clearInhibitedStateKalmanGains() 把对应的
      卡尔曼增益【置零】来处理，而不是缩矩阵。
""")
    src_banner(["ap_covpred", "px4_inhibit"], "两种「不估计某个状态」的做法")

    rows = [
        ["AP EKF3", "运行时收缩 stateIndexLim（9/12/15/21/23）",
         "矩阵变小，乘法次数减少", "索引要小心越界"],
        ["PX4 EKF2", "编译期固定 23；增益置零",
         "代码路径单一，好优化", "白算 23 行乘法的代价"],
    ]
    wtab(rows, ["", "机制", "好处", "代价"])


# ---------------------------------------------------------------------------
# 实验 2：门限单位对照
# ---------------------------------------------------------------------------

def norm_cdf(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def chi2_cdf(gate, dof):
    """自由度为 dof 的卡方分布，在 gate^2 处的 CDF。

    只用初等函数，不依赖 scipy：
        dof=1: P(X < g^2) = erf(g/sqrt2)
        dof=2: P(X < g^2) = 1 - exp(-g^2/2)
        dof=3: P(X < g^2) = erf(g/sqrt2) - sqrt(2/pi) * g * exp(-g^2/2)
    """
    g2 = gate * gate
    if dof == 1:
        return math.erf(gate / math.sqrt(2.0))
    if dof == 2:
        return 1.0 - math.exp(-0.5 * g2)
    if dof == 3:
        return (math.erf(gate / math.sqrt(2.0))
                - math.sqrt(2.0 / math.pi) * gate * math.exp(-0.5 * g2))
    raise ValueError("本脚本只推了 1/2/3 维")


def exp2():
    head("实验 2  ★★ 门限单位对照：sigma vs 百分数")

    print("""  这是本章最容易踩的一个坑：

      PX4 的 EKF2_GPS_V_GATE  默认 5.0   —— 单位是【sigma】
      AP  的 EK3_VEL_I_GATE   默认 500   —— 单位是【百分数】

    两个数字看起来差了 100 倍，**物理上其实是同一个门限**。
    本实验把这件事算清楚，再算一下门限对应的实际拒绝率。
""")

    # ---- ① 源码逐字对照 ----
    src_banner(["px4_gate_1d", "px4_gate_3d", "ap_vel_gate",
                "ap_pos_gate", "ap_hgt_gate", "ap_mag_gate"],
               "① 两边源码里的除法形式（逐字）")

    print("""    PX4（ekf.h，1 维与 N 维两个重载）：

        status.test_ratio = sq(status.innovation) / (sq(innovation_gate) * status.innovation_variance);
        status.test_ratio[i] = sq(status.innovation[i]) / (sq(innovation_gate) * status.innovation_variance[i]);

    AP（PosVelFusion / MagFusion，注意 (ftype) 强制转换是源码里就有的）：

        velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01 * (ftype)frontend->_gpsVelInnovGate, 1.0)));
        maxPosInnov2 = sq(MAX(0.01 * (ftype)frontend->_gpsPosInnovGate, 1.0))*(varInnovVelPos[3] + varInnovVelPos[4]);
        hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01 * (ftype)frontend->_hgtInnovGate, 1.0)) * varInnovVelPos[5]);
        magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (ftype)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);

    ★★ 那个【0.01 *】就是「百分数 -> sigma」的换算。

    ★ 注意 PX4 那个 N 维重载里的写法（ekf.h:1197-1215）：

        bool innovation_rejected = false;
        for (size_t i = 0; i < (sizeof(status.test_ratio) / sizeof(status.test_ratio[0])); i++) {
            ...
            status.test_ratio[i] = sq(status.innovation[i]) / (sq(innovation_gate) * status.innovation_variance[i]);
            if (status.test_ratio[i] > 1.f) {
                innovation_rejected = true;
            }
            ...
        }

      => PX4 是【逐轴判、任一轴超限则整体拒绝】。
         AP 是把各轴合成一个标量再判一次（见 ⑦ 和 ⑧）。
""")

    sub("② 数值对照：同一个门限，两种写法")
    rows = []
    for gate_pct in (100, 200, 300, 500, 1000):
        sigma = 0.01 * gate_pct
        rows.append([gate_pct, "%.2f" % sigma, "%.4f" % sigma ** 2])
    wtab(rows, ["AP 百分数", "换算成 sigma", "gate^2"])

    print("""    ⇒ 对于 5σ，两种写法分别是 500（AP）和 5.0（PX4）。
      对于 3σ，分别是 300 和 3.0。

    ★ AP 的百分数写法有一个隐藏好处：
      参数是【整数】。AP 的参数系统里整数参数不会因为
      EEPROM 里的浮点舍入而漂移，跨固件版本也稳定。
      PX4 用的是浮点参数（YAML 生成），5.0 就是 5.0。
""")

    # ---- ③ 除零保护 ----
    sub("③ 两边都有「下限保护」，但写法不同")

    print("""    两边的源码（逐字）：

      PX4/src/modules/ekf2/EKF/gps_control.cpp:191
          math::max(_params.gps_vel_innov_gate, 1.f),                 // innovation gate
      PX4/src/modules/ekf2/EKF/gps_control.cpp:218
          math::max(_params.gps_pos_innov_gate, 1.f), // innovation gate
      PX4/src/modules/ekf2/EKF/baro_height_control.cpp:62
          const float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);
      AP/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:844
          ... * sq(MAX(0.01 * (ftype)frontend->_gpsVelInnovGate, 1.0)));
      AP/libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp:561
          ... / (sq(MAX(0.01f * (ftype)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);

    ★ PX4 的地板是【分散写】的：每个融合源在自己那一行里
      math::max(..., 1.f)。grep 一下就发现 gps_control.cpp、mag_fusion.cpp、
      airspeed_fusion.cpp、optflow_fusion.cpp 各自都写了一遍。
      AP 的地板是【跟着百分数换算一起写】的：MAX(0.01*gate, 1.0)。

      => 两边都把地板压在 1.0 sigma，但一个写在「取参数」处，
         一个写在「百分数换 sigma」处。
""")

    rows = []
    for g in (0.0, 0.5, 5.0):
        rows.append(["PX4  math::max(gate, 1.f)", "%.1f" % g,
                     "%.1f" % max(g, 1.0)])
    for g in (0, 50, 500):
        rows.append(["AP   MAX(0.01*gate, 1.0)", "%d" % g,
                     "%.1f" % max(0.01 * g, 1.0)])
    wtab(rows, ["哪一边", "用户设的门限", "实际生效的 sigma"])

    print("""    ★ 两边的【地板值都是 1.0 sigma】—— 也就是不允许用户把门限
      设得比 1σ 还小。原因很直白：1σ 门限本来就会拒掉 31.7% 的
      好测量（见下面 ④），再小就没有工程意义了。

    ★ 差别在【类型】：
      PX4 的参数本身就是浮点 sigma（module.yaml 里 unit: SD / min: 1.0），
      代码里的 max(..., 1.f) 是双保险；
      AP  的参数是整数百分数（@Range: 100 1000 / @Increment: 25），
      必须先在代码里乘 0.01 才是 sigma。
""")

    # ---- ④ 新息拒绝率：1 维 ----
    sub("④ 新息拒绝率：一维到底拒掉多少好测量")

    gates = [1.0, 2.0, 3.0, 5.0]
    rows = []
    for g in gates:
        acc = chi2_cdf(g, 1)
        rows.append(["%.0f sigma" % g, "%.8f" % acc, "%.6e" % (1 - acc),
                     "%.4f %%" % (100 * (1 - acc))])
    wtab(rows, ["门限", "接受率", "拒绝率", "拒绝率"])

    print("""    ★★ 看清楚第一行：**门限设成 1σ，会拒掉 31.73%% 的好测量。**
      这就是为什么两边的地板都是 1.0 sigma：
      门限的功能是「挡住野值」，不是「筛掉噪声」。

    ★ 而 5σ 门限的拒绝率是 %.2e —— 换句话说，
      大约每 %.0f 万次测量才会误拒一次。
      这才是它敢叫「门限」的原因。
""" % (1 - chi2_cdf(5.0, 1), 1.0 / (1e4 * (1 - chi2_cdf(5.0, 1)))))

    # ---- ⑤ 多维：这才是重点 ----
    sub("⑤ ★★ 多维新息：为什么 3 维的拒绝率比 1 维高得多")

    print("""    数学原因（不跳步）：

      新息向量 d 的维数是 n，则统计量 T = d^T S^-1 d 服从自由度为 n 的
      卡方分布（因为 S^-1 把 d 白化了 —— 复习第 4 章 01 §4.1 的马氏距离）。

      门限检验是 T < gate^2。**同一个 gate，维数越高，T 越大。**
      直观：T 是 n 个独立的「平方」之和，n 越大，和的尾部越厚。
""")

    rows = []
    for g in gates:
        r1 = 1 - chi2_cdf(g, 1)
        r2 = 1 - chi2_cdf(g, 2)
        r3 = 1 - chi2_cdf(g, 3)
        rows.append(["%.0f sigma" % g,
                     "%.3e" % r1, "%.3e" % r2, "%.3e" % r3,
                     "%.1f x" % (r3 / r1)])
    wtab(rows, ["门限", "1 维（T>g^2）", "2 维（T>g^2）", "3 维（T>g^2）", "3维/1维"])

    print("""    ★★ 5σ 时 3 维的误拒率是 1 维的 %.1f 倍。
      这是个必须知道的工程事实：**如果你把一个 3 维新息当成 1 维来设门限，
      实际误拒率会高一个数量级以上。**

    ★ 所以两边都必须区分维数：
      PX4 有 sq(innovation) 和 sq(innovation[i]) 两个重载；
      AP 有 posTestRatio（2 维）、velTestRatio（2 或 3 维）、
      hgtTestRatio（1 维）三个不同的式子。

    ⚠️ 但上面这张表用的统计量是【马氏范数】 T = sum(w_i^2) > g^2，
       这是「把 n 轴合成一个标量」的检验。
       **PX4 实际做的不是这个** —— 见 ⑤c。
""" % ((1 - chi2_cdf(5.0, 3)) / (1 - chi2_cdf(5.0, 1))))

    # ---- ⑤b AP 的等效门限是 n*gate^2，不是 gate^2 ----
    sub("⑤b ★★ 但 AP 的等效卡方门限是 n x gate^2，不是 gate^2")

    print("""    PX4 单轴的形式（ekf.h:1188）确实是「T = d^2/S > gate^2 就拒」；
    AP 不是。**AP 的阈值不是 gate^2。** 以 velTestRatio 为例
    （PosVelFusion.cpp:844）：

        velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01*gate, 1.0)))

    写成数学式：

                    sum(nu_i^2)
        ratio  =  ------------------------      拒绝条件：ratio > 1
                  gate^2 * sum(sigma_i^2)

    两边同乘分母：

                    sum(nu_i^2)  >  gate^2 * sum(sigma_i^2)

    若各轴方差相同（sigma_i^2 = s^2，n 轴），则 sum(sigma_i^2) = n*s^2；
    而 sum(nu_i^2) ~ s^2 * chi2_n。代入：

        s^2 * chi2_n  >  gate^2 * n * s^2
        ⟺  chi2_n > n * gate^2

    ★★ 所以 **AP 的等效卡方门限是 n*gate^2**：
       n = 3、gate = 5 时是 **75**，而不是 25。

    ★ 物理含义：**AP 投的是「三轴的 RMS 新息超没超 gate」，
      PX4 投的是「任意一轴超没超 gate」。** 两条规则都对，
      但严格程度差很远 —— 见下表最后一列。
""")

    rows = []
    for g in gates:
        r_norm = 1 - chi2_cdf(g, 3)                 # sum(w^2) > g^2
        r_ap = 1 - chi2_cdf(math.sqrt(3.0) * g, 3)  # chi2_3 > 3*g^2     （AP 约定）
        rows.append(["%.0f sigma" % g,
                     "%.3e" % r_norm, "%.3e" % r_ap,
                     "%.4g x" % (r_norm / r_ap if r_ap > 0 else float("inf"))])
    wtab(rows, ["门限", "合成标量 chi2_3 > g^2", "AP chi2_3 > 3g^2", "两者之比"])

    # ---- ⑤c PX4 真正的规则：逐轴 OR ----
    sub("⑤c ★★ PX4 真正的规则不是马氏范数，而是「逐轴 OR」")

    print("""    回头看 ekf.h:1197-1215 那个循环（本实验 ① 里引过）：

        for (i = 0; i < 3; i++) {
            test_ratio[i] = sq(innov[i]) / (sq(gate) * innov_var[i]);
            if (test_ratio[i] > 1.f) { innovation_rejected = true; }
        }

    它【没有】把三轴加起来。它是：

        拒绝  <=>  存在某一轴 i，使得 |w_i| > gate

    设单轴超限概率 p = P(|w| > g) = erfc(g/sqrt(2))，三轴独立，则

        PX4 拒绝率 = P(至少一轴超限)
                   = 1 - P(三轴都不超限)
                   = 1 - (1 - p)^3
                   = 3p - 3p^2 + p^3

    而 AP 是「RMS 超限」，等价于 chi2_3 > 3*gate^2。
""")

    rows = []
    for g in gates:
        p = 1.0 - math.erf(g / math.sqrt(2.0))          # 单轴超限概率
        r_px4_real = 1.0 - (1.0 - p) ** 3               # 逐轴 OR，n=3
        r_px4_norm = 1 - chi2_cdf(g, 3)                 # sum(w^2) > g^2
        p_ap = 1 - chi2_cdf(math.sqrt(3.0) * g, 3)      # chi2_3 > 3 g^2
        rows.append(["%.0f sigma" % g,
                     "%.3e" % r_px4_real, "%.3e" % r_px4_norm, "%.3e" % p_ap,
                     "%.4g x" % (r_px4_real / p_ap if p_ap > 0 else float("inf"))])
    wtab(rows, ["门限", "PX4 真身:逐轴OR", "对照:合成标量", "AP:RMS", "PX4/AP"])

    p1 = 1.0 - math.erf(5.0 / math.sqrt(2.0))       # 单轴 5 sigma 超限概率
    r_px4_5 = 1.0 - (1.0 - p1) ** 3                 # PX4 逐轴 OR，3 轴
    r_norm_5 = 1 - chi2_cdf(5.0, 3)                 # 合成标量 sum(w^2) > 25
    r_ap_5 = 1 - chi2_cdf(math.sqrt(3.0) * 5.0, 3)  # AP RMS: chi2_3 > 75
    print("""    ★★★ 这是本实验最值得记住的一张表。三个不同的统计量，
      在 5 sigma / 3 轴下的误拒率分别是：

          PX4 逐轴 OR          %.3e      （= 1-(1-p)^3, p = %.3e）
          合成标量（马氏范数） %.3e      （比逐轴 OR 大 %.1f 倍）
          AP 的 RMS            %.3e      （= chi2_3 > 3*gate^2 = 75）

      · 逐轴 OR 比「合成标量」【小】 —— 因为合成标量是
        「三轴平方和超一个阈值」，它把「三轴同时各超一点点」
        这种被逐轴 OR 漏掉的情况也算成拒绝了。
        单轴 5 sigma 的事件太稀有，三轴同时发生几乎不可能，
        所以逐轴 OR 近似等于 3p（本例 %.3e ≈ 3 x %.3e）。

      ⇒ **P(PX4 拒绝) / P(AP 拒绝) ≈ %.2e**，
        即 PX4 的误拒率是 AP 的 **%.1e 倍**。
        换句话说：AP 的「500」在数值上确实等于 5 sigma，
        但它检验的是三轴 RMS，比 PX4 的「任一轴 5 sigma」严了
        **约 10 个数量级**。

      ⚠️⚠️ 但**这不代表 AP 更「松」或更「紧」是 bug**，因为：
        · 严格与否只在【同样的新息分布】下才有意义；
        · 两边对新息方差 sigma 的估计（R 矩阵、观测噪声参数）
          完全不同，5 sigma 在两边对应的物理偏差量级并不相同；
        · 门限的作用是「挡住野值」，不是「精确控制误拒率」。
          只要 1.7e-6 和 3.3e-16 都远小于真实野值出现率，两者都够用。

      ★ 真正的工程结论只有一句：
        **调参数时不要把 AP 的 500 和 PX4 的 5.0 当成同一个东西来抄。**
        写文档、做对比试验时必须注明「几轴、什么统计量」。
""" % (r_px4_5, p1, r_norm_5, (r_norm_5 / r_px4_5), r_ap_5,
       r_px4_5, p1,
       (r_px4_5 / r_ap_5), (r_px4_5 / r_ap_5)))

    # ---- ⑥ 蒙特卡洛 ----
    sub("⑥ 蒙特卡洛验证：脚本真的把拒绝率跑出来")

    rng = np.random.default_rng(20260916)
    n_mc = 1000000
    z = rng.standard_normal((n_mc, 3))

    print("    采样 %d 组 3 维标准正态，前 k 列平方和作为 k 维统计量。" % n_mc)
    print()

    rows = []
    for g in gates:
        g2 = g * g
        for k in (1, 2, 3):
            t = np.sum(z[:, :k] ** 2, axis=1)
            n_rej = int(np.count_nonzero(t >= g2))
            emp = n_rej / float(n_mc)
            thy = 1 - chi2_cdf(g, k)
            rows.append(["%.0f sigma" % g, k,
                         "%.3e" % thy, n_rej, "%.3e" % emp,
                         "%.3e" % (n_rej / n_mc - thy)])
    wtab(rows, ["门限", "维数", "理论拒绝率", "实测拒绝数(共1e6)",
               "实测拒绝率", "偏差"])

    print("""    ★ 1 维 5σ 那一行的「理论拒绝数」只有 %.2f 次 ——
      100 万次采样里期望只出现不到一次，所以实测是 0。
      这不是脚本错了，而是【这类事件靠蒙特卡洛测不出来】。

      ⇒ 工程含义：**5σ 门限的误拒率无法通过飞行数据统计估计出来，
        只能用卡方分布解析地算。** 这也是为什么两边的源码里
        都是解析式而不是查表。
""" % (1e6 * (1 - chi2_cdf(5.0, 1))))

    # ---- ⑦ 与源码对照 ----
    sub("⑦ 回到源码：AP 的 velTestRatio 是 2 维还是 3 维？")
    src_banner(["ap_vel_imax", "ap_vel_idx", "ap_vel_gate"], "答案在 imax 上")

    print("""    AP 的 velocity 测试里有一个 imax（PosVelFusion.cpp:826-844）：

        uint8_t imax = 2;
        // Don't fuse vertical velocity observations if disabled in sources or not available
        if ((!frontend->sources.haveVelZSource() || PV_AidingMode != AID_ABSOLUTE ||
             !gpsDataDelayed.have_vz) && !useExtNavVel) {
            imax = 1;                       // 没有垂速源时降成水平 2 维
        }

        ftype innovVelSumSq = 0;
        ftype varVelSum = 0;
        for (uint8_t i = 0; i<=imax; i++) {
            stateIndex   = i + 4;
            const float innovation = stateStruct.velocity[i] - velPosObs[i];
            innovVelSumSq += sq(innovation);
            varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
            varVelSum += varInnovVelPos[i];
        }
        velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01 * gate, 1.0)));

    ★ 注意 stateIndex = i + 4 这一行 ——
      i=0/1/2 对应速度的三个轴，状态索引正好是 4/5/6。
      **这就是实验 1 那张索引表在实际代码里的样子。**

    ★ 注意分母：gate^2 乘的是 **varVelSum（各轴方差之和）**，
      分子是 **各轴新息平方之和**。
      这正是卡方统计量 T = sum(d_i^2 / S_i) 在 S_i 相等时的形式。

    ★ 对照 PX4：PX4 的 velocity aid source 用 updateVelocityAidSrcStatus()
      算出 3 维的 test_ratio 数组，然后 setEstimatorAidStatusTestRatio()
      在【任意一轴】超门限时就把整个 aid source 标成 innovation_rejected。

      ⇒ 差别：AP 是「几轴合并成一个标量检验」，
        PX4 是「逐轴检验，任一轴失败则整体拒绝」。
        AP 的写法在某一轴噪声天生较大时更宽容；
        PX4 的写法更容易定位到具体是哪一轴出问题（有 test_ratio[3] 可看）。
""")

    rows = [
        ["AP", "各轴新息平方【求和】/ 各轴方差之和 x gate^2",
         "一个标量 pos/vel/hgtTestRatio", "宽容"],
        ["PX4", "逐轴 sq(innov) / (gate^2 x innov_var)",
         "test_ratio[] 数组，任一轴超限即拒绝", "敏感"],
    ]
    wtab(rows, ["", "统计量", "输出形式", "性格"])

    print("""    ★ 这两种写法没有绝对优劣，取舍点在于：

      · 观测误差【确实各轴独立同分布】时 -> PX4 的逐轴检验更有效
        （自由度少，检验势高）。
      · 观测误差【各轴量级差很多】时 -> AP 的合并检验更稳
        （不会因为某一轴天生方差大就反复被拒）。

      GPS 速度/位置的水平两轴量级接近，所以两边做法差别不大；
      而 AP 把 [N,E,D] 合并成一个标量，实际上是承认
      「GPS 的 D 轴和水平轴噪声不一样，不该用同一个门限逐个比」。
""")

    sub("⑧ 门限默认值总表（从源码里读出来的）")
    src_banner(["px4_gates_c", "px4_mag_gate", "px4_hdg_gate", "px4_beta_gate",
                "px4_baro_gate", "px4_gpsp_gate", "px4_gpsv_gate",
                "px4_of_gate", "px4_flow_gate_c", "ap_gate_defaults"],
               "出处清单（左 PX4 / 右 AP）")

    rows = [
        ["GPS 速度", "EKF2_GPS_V_GATE", "5.0", "EK3_VEL_I_GATE", "500", "5 sigma"],
        ["GPS 位置", "EKF2_GPS_P_GATE", "5.0", "EK3_POS_I_GATE", "500", "5 sigma"],
        ["气压/GPS 高度", "EKF2_BARO_GATE", "5.0", "EK3_HGT_I_GATE", "500", "5 sigma"],
        ["磁力计 XYZ", "EKF2_MAG_GATE", "3.0", "EK3_MAG_I_GATE", "300", "3 sigma"],
        ["航向", "EKF2_HDG_GATE", "2.6", "（无独立参数）", "—", "2.6 sigma"],
        ["侧滑", "EKF2_BETA_GATE", "5.0", "（无独立参数）", "—", "5 sigma"],
        ["光流", "EKF2_OF_GATE", "3.0", "EK3_FLOW_I_GATE", "300（plane 500）",
         "3 sigma"],
        ["GPS 跳变半径", "（无对应参数）", "—", "EK3_GLITCH_RAD", "25 m", "—"],
    ]
    wtab(rows, ["传感器", "PX4 参数", "默认", "AP 参数", "默认", "换算"])

    print("""    ★ 三个「看起来不一样」的地方，逐个解释：

      · 航向 2.6σ：航向融合的误差模型最不准（磁偏角、倾斜误差都算进去），
        门限必须收紧。PX4 单独给了 EKF2_HDG_GATE。
        AP 走的是另一条路 —— EK3_MAG_CAL 直接切换融合模式（见 15.4.3），
        不怎么靠调门限。

      · 光流：AP 的 plane 默认 500、copter 默认 300（源码里 Plane 段是
        FLOW_I_GATE_DEFAULT 500，Copter/Rover 段是 300）；
        PX4 的光流门限在参数表里叫 EKF2_OF_GATE（默认 3.0），
        内部字段名是 flow_innov_gate。

      · EK3_GLITCH_RAD 在 PX4 里【没有对应参数】。
        这是因为两边对「位置新息持续偏大」的处理哲学不同（见 15.5.1）。

    ★ 还有一个隐藏的对照：
      AP 的门限参数全部是 **uint16 百分数**（@Range: 100 1000，
      @Increment: 25），PX4 全部是 **float sigma**（min 1.0，unit SD）。
      这直接决定了两边地面站的滑块长得不一样：
      AP 是 25 一档的整数，PX4 是 0.1 一档的小数。
""")


# ---------------------------------------------------------------------------
# 实验 3：融合顺序并排对照
# ---------------------------------------------------------------------------

# PX4 Ekf::controlFusionModes 的真实顺序（control.cpp:46-162）
PX4_ORDER = [
    ("system_flag_buffer 出队", "at_rest / in_air / is_fixed_wing / gnd_effect", "体系状态", 100),
    ("tilt alignment 监视", "getTiltVariance() < sq(radians(3))", "对齐", 100),
    ("controlMagFusion()", "磁力计（含 3D 与航向两种模型）", "观测控制", 50),
    ("controlOpticalFlowFusion()", "光流", "观测控制", 50),
    ("controlGpsFusion()", "GPS 位置/速度/航向", "观测控制", 10),
    ("_aux_global_position.update()", "辅助全局位置", "观测控制", 10),
    ("controlAirDataFusion()", "空速", "观测控制", 50),
    ("controlBetaFusion()", "合成侧滑", "观测控制", 50),
    ("controlDragFusion()", "机体阻力", "观测控制", 50),
    ("controlHeightFusion()", "高度源仲裁 + 高度融合", "观测控制", 25),
    ("controlGravityFusion()", "重力矢量", "观测控制", 50),
    ("controlExternalVisionFusion()", "外部视觉", "观测控制", 30),
    ("controlAuxVelFusion()", "辅助速度", "观测控制", 30),
    ("controlZeroInnovationHeadingUpdate()", "零新息航向保持", "兜底", 100),
    ("_zero_velocity_update.update()", "零速度伪观测（地面）", "兜底", 100),
    ("_zero_gyro_update.update()", "零角速率伪观测（地面）", "兜底", 100),
    ("controlFakePosFusion()", "假位置（无辅助时约束漂移）", "兜底", 100),
    ("controlFakeHgtFusion()", "假高度", "兜底", 100),
    ("updateDeadReckoningStatus()", "更新推算/失效状态位", "状态汇总", 100),
]

# AP NavEKF3_core::UpdateFilter 的真实顺序（core.cpp:646-746）
AP_ORDER = [
    ("startPredictEnabled = predict", "前端给不给这一拍做预测", "调度", 100),
    ("statesInitialised 检查", "没初始化就直接 return", "调度", 100),
    ("fill_scratch_variables()", "预计算三角函数、DCM 等", "准备", 100),
    ("update_sensor_selection()", "传感器亲和性（affinity）选择", "准备", 100),
    ("controlFilterModes()", "所有模式与状态位的决策（含 setAidingMode）", "决策", 100),
    ("readIMUData()", "从 IMU 缓冲区取延迟时刻的数据", "准备", 100),
    ("  UpdateStrapdownEquationsNED()", "四元数/速度/位置 惯导递推（预测）", "预测", 100),
    ("  CovariancePrediction(nullptr)", "协方差预测（代码生成的大块）", "预测", 100),
    ("  runYawEstimatorPrediction()", "EKF-GSF 偏航估计器：预测", "GSF", 100),
    ("  SelectMagFusion()", "磁力计融合（3D 或航向）", "融合", 50),
    ("  SelectVelPosFusion()", "GPS/外视 速度+位置+高度融合", "融合", 10),
    ("  runYawEstimatorCorrection()", "EKF-GSF 偏航估计器：用 GPS 速度校正", "GSF", 10),
    ("  SelectRngBcnFusion()", "测距信标", "融合", 5),
    ("  SelectFlowFusion()", "光流", "融合", 50),
    ("  SelectBodyOdomFusion()", "机体里程计", "融合", 30),
    ("  SelectTasFusion()", "真空速", "融合", 50),
    ("  SelectBetaDragFusion()", "侧滑/阻力", "融合", 50),
    ("  updateFilterStatus()", "汇总 filterStatus 位", "状态汇总", 100),
    ("  1 Hz: moveEKFOrigin / checkUpdateEarthField", "每秒一次的低频任务", "低频", 1),
    ("calcOutputStates()", "把状态推到输出时间轴", "输出", 100),
    ("地面 5 秒不健康 -> 强制重置", "防「陀螺还在稳定」的假收敛", "保护", 1),
]


def exp3():
    head("实验 3  ★ 融合顺序并排对照：两种调度结构")

    print("""  同一个问题：一拍里该按什么顺序处理观测？

    两边给出的顺序【很像】，但**结构完全不同**：

      PX4：controlFusionModes() 只是【决策层】，它按固定顺序调用
           controlXxxFusion()，每个函数只决定「用不用这个观测」；
           真正的门限检验和融合在融合函数内部，
           结果写进 _aid_src_* 这些 uORB 消息结构体。
      AP ：UpdateFilter() 是【决策 + 融合】一条龙，
           Select*Fusion() 里既判也融。
""")

    src_banner(["px4_ctrl_fusion", "ap_update_filter"], "① 两个入口函数")

    sub("② 真实顺序并排（左 PX4 / 右 AP）")
    src_banner(["px4_ctrl_seq", "ap_update_seq"], "序号是按源码里的出现顺序抄的")

    n = max(len(PX4_ORDER), len(AP_ORDER))
    rows = []
    for i in range(n):
        p = PX4_ORDER[i] if i < len(PX4_ORDER) else ("", "", "", "")
        a = AP_ORDER[i] if i < len(AP_ORDER) else ("", "", "", "")
        rows.append([i + 1, p[0], a[0]])
    wtab(rows, ["#", "PX4 controlFusionModes", "AP UpdateFilter"])

    print("""    ★ 注意 AP 那一列前面有缩进的两段：它们**不在 if 之外**，
      而是在 if (runUpdates) { ... } 里面。也就是说：
      **预测 + 融合 + 状态汇总 是一个原子块，要么全做要么全不做。**

    ★★ 这个 if (runUpdates) 就是 AP 的「多 lane 调度」入口 ——
      见下面 ④。
""")

    sub("③ 结构差异：决策和融合是分开还是合并")

    rows = [
        ["结构", "决策与融合分层", "决策与融合一体"],
        ["决策层", "controlXxxFusion() 只改 _control_status.flags.*",
         "controlFilterModes() 改模式 + setAidingMode()"],
        ["融合层", "融合函数自己调 aid_src 更新 + fuse",
         "SelectXxxFusion() 内部既判也融"],
        ["观测状态载体", "estimator_aid_source{1,2,3}d_s（uORB 消息）",
         "成员变量 + faultStatus 位"],
        ["对外可见性", "直接 publish 成 uORB 话题", "写日志 / 写 MAVLink"],
        ["代码规模", "EKF/ 下 40 个 .cpp", "AP_NavEKF3/ 下 14 个文件"],
    ]
    wtab(rows, ["", "PX4 EKF2", "AP EKF3"])

    src_banner(["px4_aidsrc_msg", "px4_velaid", "px4_velaid_h",
                "px4_pub_aidsrc", "px4_pub_helper", "px4_pub_gnssvel"],
               "★★ PX4 最值得学的一个设计：观测状态就是 uORB 消息")

    print("""    ★★ 在【这个版本】的 PX4 里，aid source 不是一个 C++ 类，
      而是**uORB 消息结构体本身**：

        estimator_aid_source1d_s  _aid_src_airspeed{};   // ekf.h 里共 21 个
        estimator_aid_source2d_s  _aid_src_gnss_pos{};
        estimator_aid_source3d_s  _aid_src_gnss_vel{};
        ...

      每个都带 innovation / innovation_variance / test_ratio /
      fusion_status / time_last_fuse 等字段。填完之后：

        EKF2::PublishAidSourceStatus(timestamp)              // EKF2.cpp:864
          -> PublishAidSourceStatus(_ekf.aid_src_gnss_vel(),
                                    _status_gnss_vel_pub_last,
                                    _estimator_aid_src_gnss_vel_pub);  // :905
             -> pub.publish(status_out);                     // EKF2.hpp:252 模板

      ⇒ 于是 `estimator_aid_source3d_s` 这个 uORB 话题里，
        **创新、创新方差、检验比、有没有被融合**，一次性全在。

    ★★★ 这是本章最实用的一个工程差异：

      **调试「为什么 GPS 被拒了」时**
        PX4  ：listener estimator_aid_source3d_s
               -> 一次看到 innovation / test_ratio / innovation_rejected
        AP   ：看 EKF3 的日志（NKF1/NKF3/NKF4 记录）或者
               faultStatus 的 bad_nvel/bad_npos/... 位
               （PosVelFusion.cpp:1084-1111）

      两边都能定位到「哪一轴」，但 PX4 的粒度是**每一条 aid source 一个话题**，
      AP 的粒度是**一组布尔位**。
      PX4 的做法多花了不少 uORB 带宽和内存，换来的是**现场排障速度**。
      这是典型的 PX4 风格取舍：可观测性优先。
""")

    # ---- ④ 调度 ----
    sub("④ ★★ 调度差异：多实例怎么分 CPU")
    src_banner(["ap_frames_pred", "ap_cpu_budget", "ap_target_dt", "px4_predict_us"],
               "两边都有「别让 EKF 吃满 CPU」的机制")

    print("""    AP：

        _framesPerPrediction = uint8_t((EKF_TARGET_DT / (_frameTimeUsec*1e-6) + 0.5));

      其中 EKF_TARGET_DT = 0.012 s（约 83 Hz）。
      然后在 UpdateFilter 的调用点：

        for (uint8_t i=0; i<num_cores; i++) {
            bool allow_state_prediction = true;
            if (core[i].getFramesSincePredict() < (_framesPerPrediction+3) &&
                AP::dal().ekf_low_time_remaining(AP_DAL::EKFType::EKF3, i)) {
                allow_state_prediction = false;      // 这一拍只做融合，不做预测
            }
            core[i].UpdateFilter(allow_state_prediction);
        }

      ⇒ **AP 让多个 lane 共享同一个 CPU 预算，谁快超支谁就跳过预测。**
        这是「lane 可以开到 6 个」的前提。

    PX4：

        EKF2_PREDICT_US 默认 10000（100 Hz）—— 每个实例各自固定预测周期，
        实例之间不共享预算；多实例的 CPU 压力靠
        EKF2_MULTI_IMU / EKF2_MULTI_MAG 限制实例数来控制。

    ★★ 一句话：
      AP 是「软实时协商」（运行时让路），
      PX4 是「静态配置」（配置时定死）。
""")

    rows = []
    for hi in (100, 200, 400, 800):
        fpp = int(0.012 / (1.0 / hi) + 0.5)
        rows.append([hi, "%.5f" % (1.0 / hi), fpp, "%.4f" % (fpp / float(hi))])
    wtab(rows, ["IMU 环路频率 (Hz)", "帧间隔 (s)", "_framesPerPrediction",
               "实际预测周期 (s)"])
    print("""    ⇒ 400 Hz 的 IMU 上，_framesPerPrediction = 5：
      每 5 帧 IMU 做一次协方差预测，中间 4 帧只做状态递推。
      **这正是 AP 的「延迟时间轴」设计**（EKF 跑在过去 12 ms 处，
      而 12 ms 正好是 100 Hz 的周期）。

    ★ 顺带解释 AP 的 _framesPerPrediction + 3 里那个 3：
      允许偶尔连续 3 帧不预测（补上因为 GC 或日志写入抖动
      而错过的预测），超过了就说明 CPU 真的不够了。
""")

    # ---- ⑤ 简化调度仿真 ----
    sub("⑤ 简化仿真：1 秒钟里两个滤波器各做了什么")

    rates = {"IMU": 100, "mag": 50, "baro": 25, "GPS": 10}

    def simulate(n_lanes, imu_hz=100, target_dt=0.012, seconds=1.0, budget=0.70):
        """按 AP 的调度规则跑 seconds 秒，返回统计。

        budget 是「这一拍还剩多少 CPU 预算」的简化模型：
        每拍消耗与 lane 数成正比，超过 budget 就抑制预测。
        """
        n_ticks = int(imu_hz * seconds)
        frames_per_pred = max(1, int(target_dt * imu_hz + 0.5))
        stats = [dict(pred=0, fuse=0, suppressed=0) for _ in range(n_lanes)]
        since = [0] * n_lanes
        for _t in range(n_ticks):
            # 简化：lane 数越多，单拍的剩余预算越少
            low_time = (n_lanes * 0.18) > budget * 1.0
            for i in range(n_lanes):
                since[i] += 1
                allow = True
                if since[i] < (frames_per_pred + 3) and low_time:
                    allow = False
                stats[i]["fuse"] += 1
                if allow:
                    stats[i]["pred"] += 1
                    since[i] = 0
                else:
                    stats[i]["suppressed"] += 1
        return stats, frames_per_pred

    rows = []
    for n_lanes in (1, 2, 3, 4, 6):
        st, fpp = simulate(n_lanes)
        rows.append([n_lanes, fpp, st[0]["pred"], st[0]["fuse"],
                     st[0]["suppressed"],
                     "%d" % (st[0]["pred"] * n_lanes)])
    wtab(rows, ["lane 数", "_framesPerPrediction", "lane0 预测次数",
               "lane0 融合次数", "lane0 被抑制", "全部 lane 预测总数"])

    print("""    ★ 这张表就是 AP「软实时协商」的样子：
      lane 数 <= 3 时每拍都做预测（100 次/秒）；
      从 4 个 lane 起预算不够，预测被压到 25 次/秒 ——
      **但融合一次都没少做**（融合是 EKF 的「信息入口」，不能省）。

    ★ 为什么压到 25 次/秒就不再往下掉了？
      因为抑制的判据是

          core[i].getFramesSincePredict() < (_framesPerPrediction + 3)

      一旦距上次预测满 _framesPerPrediction + 3 帧，**不管多缺 CPU
      都必须预测一次**（否则协方差会无限膨胀）。
      100 Hz IMU 上 _framesPerPrediction = 1，所以最低预测周期是 4 帧
      = 25 次/秒。**这个下限是源码写死的。**

      ⇒ 所以 lane 数继续增加（4 -> 6）时，
        单个 lane 的预测率不变，但「全部 lane 预测总数」线性增长 ——
        这正是 AP 不敢默认多开 lane 的原因：
        EK3_IMU_MASK 默认 3（两个 lane），不是 6。

    ★★ 这是个很硬的工程判断：
      **预测（协方差 24x24 的乘法）可以降频，观测融合不能降频。**
      因为降预测的代价是协方差增长变慢一点，
      而降融合的代价是传感器数据被直接丢掉。
""")

    # ---- ⑥ 5 秒强制重置 ----
    sub("⑥ AP 独有的一个保护：地面 5 秒不健康就强制重置")
    src_banner(["ap_5s_reset"], "UpdateFilter 的最后一段")

    print("""        if (filterStatus.value != 0) last_filter_ok_ms = dal.millis();
        if (filterStatus.value == 0 && last_filter_ok_ms != 0 &&
            dal.millis() - last_filter_ok_ms > 5000 && !dal.get_armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "EKF3 IMU%u forced reset", imu_index);
            last_filter_ok_ms = 0;
            statesInitialised = false;
            InitialiseFilterBootstrap();
        }

    ★ 源码注释把症状写得很直白：
      「车辆在怠速时 EKF 对自己的状态过度自信，症状是用户尝试解锁时
        报 gyros still settling。在这种状态下 EKF 无法自己恢复，
        所以我们硬重置一次，让它重来。」

    ★★ 这是「协方差过度自信」的教科书级案例 ——
      复习第 4 章 01 §7.4：模型错了，P 还是会自信地缩小。
      这里 P 缩小到 filterStatus 报不健康，但滤波器不会自己变大回去，
      所以只能人为重置。

    ★ PX4 的对应做法是【对调】：它不重置，而是让
      EKF2Selector 把不健康的实例换掉（AP 也有 lane 切换，
      但这里多了一道"整机级"的兜底）。见实验 4。
""")


# ---------------------------------------------------------------------------
# 实验 4：lane 选择器仿真
# ---------------------------------------------------------------------------

CORE_ERR_LIM = 1.0     # AP_NavEKF3.h:522
BETTER_THRESH = 0.5    # AP_NavEKF3.h:523
ERR_THRESH = 0.2       # AP_NavEKF3.cpp:639 (EK3_ERR_THRESH 默认)
PX4_ERR_RED = 0.2      # params_selector.yaml (EKF2_SEL_ERR_RED 默认)
PX4_REL_LIM = 1.0      # EKF2Selector.hpp:146
PX4_REL_THRESH = 0.5   # EKF2Selector.hpp:147


def error_score(vel_tr, pos_tr, hgt_tr, mag_tr_sum=None, mag_affinity=False):
    """AP NavEKF3_core::errorScore() 的忠实实现（Outputs.cpp:37-61）。"""
    score = max(0.5 * (vel_tr + pos_tr), hgt_tr)
    if mag_affinity and mag_tr_sum is not None:
        score = max(score, 0.3 * mag_tr_sum)
    return score


def px4_combined(vel_tr, pos_tr, hgt_tr):
    """PX4 EKF2Selector::combined_test_ratio（EKF2Selector.cpp:294）。"""
    return max(0.5 * (vel_tr + pos_tr), hgt_tr)


def make_truth(n_lanes=4, n_steps=600):
    """造一份 lane 的 test ratio 时间序列（仿真数据，脚本自己生成）。

    场景：起飞后 3 秒，lane 0（默认 primary）的 GPS 位置开始跳变；
    同时 lane 2 的 IMU 振动变大（速度新息持续偏大）；
    lane 3 的磁力计被干扰。lane 1 全程健康。
    """
    t = np.arange(n_steps) * 0.05          # 20 Hz 更新，共 30 秒
    tr = np.zeros((n_steps, n_lanes, 3))   # [step, lane, (vel, pos, hgt)]

    # lane 0：3 秒后 GPS 位置跳变
    tr[:, 0, 0] = 0.25
    tr[:, 0, 1] = np.where(t < 3.0, 0.30, 9.0)      # posTestRatio 冲到 9
    tr[:, 0, 2] = 0.20

    # lane 1：全程健康
    tr[:, 1, 0] = 0.30
    tr[:, 1, 1] = 0.35
    tr[:, 1, 2] = 0.25

    # lane 2：8 秒后 IMU 振动 -> 速度/高度新息变大
    ramp = np.clip((t - 8.0) / 2.0, 0.0, 1.0)
    tr[:, 2, 0] = 0.30 + 1.10 * ramp
    tr[:, 2, 1] = 0.35 + 0.10 * ramp
    tr[:, 2, 2] = 0.25 + 2.20 * ramp

    # lane 3：全程磁干扰（只在开启 MAG affinity 时才计入分数）
    tr[:, 3, 0] = 0.40
    tr[:, 3, 1] = 0.45
    tr[:, 3, 2] = 0.30

    return t, tr


def run_ap_selector(scores, dt=0.05,
                    err_thresh=ERR_THRESH, better_thresh=BETTER_THRESH,
                    health=None, min_primary_s=10.0, armed=True, primary=0):
    """AP NavEKF3::updateFilter 里的 lane 选择逻辑（AP_NavEKF3.cpp:936-988）。

    忠实实现的要点：
      · updateCoreErrorScores()  —— 取所有 lane 的 errorScore()
      · updateCoreRelativeErrors()
            error = score[i] - score[primary]
            只在 (error > 0) 或 (error < -max(err_thresh, 0.05)) 时累加
            累加后 clamp 到 +-CORE_ERR_LIM
      · coreBetterScore(i, j) —— 先比健康，再比 tilt/yaw 对齐，最后比相对误差
      · altCoreAvailable 需要「不是 primary 至少 10 秒」
      · 切换条件是 primaryErrorScore > 1.0 或 !healthy 或 betterCore
      · 切换后 resetCoreErrors()
    """
    n_lanes = scores.shape[1]
    rel = np.zeros(n_lanes)
    last_primary_t = np.full(n_lanes, -1e9)
    t = 0.0
    log = []

    for k in range(scores.shape[0]):
        sc = scores[k]
        if health is None:
            h = np.ones(n_lanes, dtype=bool)
        else:
            h = health[k]

        primary_score = sc[primary]

        # --- updateCoreRelativeErrors ---
        for i in range(n_lanes):
            if i == primary:
                continue
            err = sc[i] - sc[primary]
            if err > 0 or err < -max(err_thresh, 0.05):
                rel[i] += err
                rel[i] = min(max(rel[i], -CORE_ERR_LIM), CORE_ERR_LIM)

        # --- 找替代 lane ---
        better_core = False
        alt_available = False
        new_primary = primary
        if armed:
            for i in range(n_lanes):
                if i == primary:
                    continue
                # coreBetterScore(i, new_primary)
                if not h[i]:
                    continue
                ok = rel[i] < rel[new_primary]
                if ok and (t - last_primary_t[i]) > min_primary_s:
                    alt_available = True
                    better_core = bool(rel[i] <= -better_thresh)
                    new_primary = i
            alt_available = (new_primary != primary)

            if alt_available and (primary_score > 1.0 or not h[primary] or better_core):
                log.append((k, t, primary, new_primary,
                            "score>1" if primary_score > 1.0 else
                            ("unhealthy" if not h[primary] else "better")))
                last_primary_t[primary] = t
                primary = new_primary
                rel[:] = 0.0
                rel = np.zeros(n_lanes)

        t += dt

    return log, primary


def run_px4_selector(scores, dt=0.05, err_red=PX4_ERR_RED,
                     rel_thresh=PX4_REL_THRESH,
                     health=None, armed=True, selected=0):
    """PX4 EKF2Selector 的选择逻辑（EKF2Selector.cpp:338-359, 709-784）。

    忠实实现的要点：
      · combined_test_ratio = max(0.5*(vel+pos), hgt)
      · healthy = (fault_flags == 0) && combined > 0
      · warning = combined >= 1
      · relative_test_ratio 的更新规则与 AP 一字不差（0.2 阈值、+-1 限幅）
      · 切换条件：
          不健康 -> 换到健康的、test_ratio 最小的（优先不同 IMU）
          健康但有 lower_error_available -> 换到 relative_test_ratio 最小的
        其中 lower_error_available 需要 relative <= -0.5 且
        该实例至少 10 秒没被选中
      · 另外还有「距上次切换 > 10 s」或「primary 处于 warning 且
        已持续 1 s 无 warning」的门槛
    """
    n = scores.shape[1]
    rel = np.zeros(n)
    last_selected_t = np.full(n, -1e9)
    last_change_t = -1e9
    time_last_no_warning = np.full(n, -1e9)
    t = 0.0
    log = []

    for k in range(scores.shape[0]):
        sc = scores[k]
        if health is None:
            h = np.ones(n, dtype=bool)
        else:
            h = health[k]
        combined = sc
        warning = combined >= 1.0
        if not warning[selected]:
            time_last_no_warning[selected] = t

        # --- 相对误差累加（与 AP 同样的规则）---
        for i in range(n):
            if i == selected:
                continue
            delta = combined[i] - combined[selected]
            thr = max(err_red, 0.05)
            if delta > 0 or delta < -thr:
                rel[i] += delta
                rel[i] = min(max(rel[i], -PX4_REL_LIM), PX4_REL_LIM)

        # --- 找替代 ---
        best_alt = None
        alt_err = 0.0
        best_ekf = selected
        best_tr = float("inf")
        lower_error_available = False
        for i in range(n):
            if i == selected or not h[i]:
                continue
            if rel[i] < alt_err:
                best_alt = i
                alt_err = rel[i]
                if rel[i] <= -rel_thresh and (t - last_selected_t[i]) > 10.0:
                    lower_error_available = True
            if 0.0 < combined[i] < best_tr:
                best_ekf = i
                best_tr = combined[i]

        if not h[selected]:
            # PX4 的优先级：先找「不同 IMU 的最佳健康实例」，
            # 简化模型里没有 IMU 编号，退化成 best_ekf
            new_sel = best_ekf
            if new_sel != selected:
                log.append((k, t, selected, new_sel, "unhealthy"))
                last_selected_t[selected] = t
                last_change_t = t
                selected = new_sel
                rel[:] = 0.0
        elif lower_error_available and best_alt is not None:
            if ((t - last_change_t) > 10.0
                    or (warning[selected]
                        and (t - time_last_no_warning[selected]) > 1.0)):
                log.append((k, t, selected, best_alt, "lower_rel_err"))
                last_selected_t[selected] = t
                last_change_t = t
                selected = best_alt
                rel[:] = 0.0
        t += dt

    return log, selected


def exp4():
    head("实验 4  ★★ lane 选择器仿真：AP 的 lane vs PX4 的 instance")

    print("""  「多个滤波器实例 -> 选一个 -> 输出」这件事，两边都做。
  本实验先摆源码结构，再用同一份故障场景把两套选择器都跑一遍。
""")

    src_banner(["ap_score", "px4_sel_comb"], "① 先看两边的打分公式")
    print("""        AP  NavEKF3_core::errorScore():
              score = MAX(score, 0.5f * (velTestRatio + posTestRatio));
              score = MAX(score, hgtTestRatio);

        PX4 EKF2Selector::Run():
              combined_test_ratio = fmaxf(0.5f * (vel_test_ratio + pos_test_ratio),
                                          hgt_test_ratio);

    ★★ 一模一样。连 0.5 的系数都一样。

    AP 还多两项（都是可选的）：
        if (assume_zero_sideslip() && 至少 2 个空速传感器 && affinity 开了 ARSP)
            score = MAX(score, 0.3f * tasTestRatio);
        if (frontend->_affinity & EKF_AFFINITY_MAG)
            score = MAX(score, 0.3f * (magTestRatio.x + magTestRatio.y + magTestRatio.z));

      ⇒ 那个 0.3 的系数是**故意压低灵敏度**：
        源码注释说「避免阵风这类事件把 error score 抬起来，
        从而误报高误差分数」。
""")

    src_banner(["ap_rel_err", "ap_rel_body", "ap_err_lim", "ap_err_thresh",
                "ap_better", "px4_sel_rel", "px4_sel_rel2", "px4_sel_lim",
                "px4_sel_red", "px4_sel_prim_upd", "px4_sel_prim_set"],
               "② 相对误差的累加规则")

    rows = [
        ["累加量", "error = score[i] - score[primary]",
         "error_delta = combined[i] - combined[selected]"],
        ["累加条件", "error > 0 或 error < -max(ERR_THRESH, 0.05)",
         "delta > 0 或 delta < -max(EKF2_SEL_ERR_RED, 0.05)"],
        ["阈值默认", "EK3_ERR_THRESH = 0.2", "EKF2_SEL_ERR_RED = 0.2"],
        ["限幅", "+-CORE_ERR_LIM = +-1", "+-_rel_err_score_lim = +-1"],
        ["切换阈值", "BETTER_THRESH = 0.5", "_rel_err_thresh = 0.5"],
        ["最短任期", "10 秒（coreLastTimePrimary_us）",
         "10 秒（time_last_selected）"],
        ["累加时机", "每拍（NavEKF3::UpdateFilter 里无条件调用）",
         "只在 primary 更新时（primary_updated）"],
        ["陀螺故障例外", "（没有）",
         "threshold 直接置 0，鼓励尽快切走"],
    ]
    wtab(rows, ["项目", "AP EKF3", "PX4 EKF2"])

    print("""    ★★★ 这张表是本章最值得记住的一张：

      **两个团队的 lane/instance 选择器是同一个算法。**
      阈值 0.2 / 限幅 1.0 / 切换 0.5 / 最短任期 10 秒 —— 全一样。

      差别只在【管道】：
        AP  把选择器**编在 EKF 前端**（NavEKF3 类自己持有 primary 索引），
            而且 lane 的选择和 EKF 的 update 在同一个函数里（AP_NavEKF3.cpp:936）。
        PX4 把选择器做成**独立模块** ekf2_selector（EKF2Selector.cpp），
            通过 uORB 订阅各实例的 estimator_status 再决定，
            EKF2 模块本身完全不知道有选择器存在。

      ⇒ 这就是「同一个算法，两种架构」。
        PX4 的做法让 ekf2 模块保持纯粹（只做估计），
        代价是多一层 uORB 往返和 ~几十毫秒的延迟。
""")

    # ---- ③ 健康与失能的定义 ----
    sub("③ 「健康」怎么定义")
    src_banner(["px4_sel_switch", "px4_sel_else", "px4_sel_req",
                "ap_switch", "ap_laneswitch"],
               "两边的切换触发条件")

    print("""    PX4 EKF2Selector::Run() 里那串 if / else if 的【优先级】：

        1) !selected.healthy          -> 优先换到「不同 IMU」的最佳健康实例，
                                         否则换到 test ratio 最小的健康实例
        2) lower_error_available      -> 换到相对误差最小的（best_ekf_alternate）
           且 (距上次切换 > 10 s 或 (selected 在 warning 且已 1 s 不 warning))
        3) _request_instance 有效      -> 用户手动指定（地面站/vehicle 命令）

    ★ AP 的对应物是 NavEKF3::updateFilter 里的那段（AP_NavEKF3.cpp:944-989）：

        for (coreIndex ...) {
            altCoreAvailable = coreBetterScore(coreIndex, newPrimaryIndex) &&
                               imuSampleTime_us - coreLastTimePrimary_us[coreIndex] > 1E7;
            if (altCoreAvailable) {
                betterCore = altCoreError <= -BETTER_THRESH;
                betterCore |= newCore.have_aligned_yaw() && !oldCore.have_aligned_yaw();
                newPrimaryIndex = coreIndex;
            }
        }
        altCoreAvailable = newPrimaryIndex != primary;
        if (altCoreAvailable && (primaryErrorScore > 1.0f || !core[primary].healthy() || betterCore)) {
            ... primary = newPrimaryIndex;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 lane switch %u", primary);
        }

    ★ 注意 AP 多出来的一个条件（PX4 没有）：

        betterCore |= newCore.have_aligned_yaw() && !oldCore.have_aligned_yaw();

      ⇒ **「偏航已经对齐的 lane」优先级高于「偏航还没对齐的」。**
        源码注释说明这是为了处理「无磁力计飞行时，副 lane 反而先完成
        偏航对齐」的情况。这是个很实际的细节：
        没有磁力计时两个 lane 都得靠 EKF-GSF 慢慢对偏航，
        谁先对上谁更该当主。

    ★ 用户在 AP 侧指定主 lane 走的是 EK3_PRIMARY，但它
      **只在未解锁时强制执行**（AP_NavEKF3.cpp:991）：

        const uint8_t user_primary = uint8_t(_primary_core) < num_cores? _primary_core : 0;
        if (primary != user_primary && core[user_primary].healthy() && !armed) {
            primary = user_primary;
        }

      源码注释解释了原因（很值得一读）：
      「在地面且未解锁时强制使用用户指定的主 lane。
        否则每次飞行用哪个 IMU 就成了抽奖 ——
        lane 选择和 GPS 更新时序的耦合会导致不同飞行用不同的 lane 当主，
        而不同 IMU 的噪声特性差别很大，这会造成性能不一致。」

      ⇒ 所以 AP 的策略是：**地面锁死（保证可复现），空中放开（保证鲁棒）。**
        PX4 的 _request_instance 则没有这个「仅地面生效」的限制。
""")

    rows = [
        ["AP", "primaryErrorScore > 1.0", "分数超 1 就换"],
        ["AP", "!core[primary].healthy()", "filterStatus 报不健康"],
        ["AP", "betterCore（相对误差 <= -0.5）", "健康但明显更差也让位"],
        ["AP", "checkLaneSwitch(): altErrorScore < lowest && < 0.9",
         "故障保护前的最后抢救（见下）"],
        ["PX4", "!healthy（filter_fault_flags != 0 或 combined <= 0）",
         "不健康就换，优先换到不同 IMU 的实例"],
        ["PX4", "lower_error_available（rel <= -0.5 且 10 s 内没被选过）",
         "健康但明显更差也让位"],
    ]
    wtab(rows, ["", "条件", "含义"])

    src_banner(["ap_laneswitch", "ap_ls_guard", "ap_ls_avail", "ap_ls_cond"],
               "★ AP 的 checkLaneSwitch()")

    print("""        /*
          check if switching lanes will reduce the normalised
          innovations. This is called when the vehicle code is about to
          trigger an EKF failsafe, and it would like to avoid that by
          using a different EKF lane
        */
        void NavEKF3::checkLaneSwitch(void)
        {
            uint32_t now = AP::dal().millis();
            if (lastLaneSwitch_ms != 0 && now - lastLaneSwitch_ms < 5000) {
                return;                       // 5 秒内不切第二次
            }
            float primaryErrorScore = core[primary].errorScore();
            float lowestErrorScore = primaryErrorScore;
            for (uint8_t coreIndex=0; coreIndex<num_cores; coreIndex++) {
                ...
                bool altCoreAvailable = newCore.healthy() && newCore.have_aligned_yaw()
                                        && newCore.have_aligned_tilt();
                float altErrorScore = newCore.errorScore();
                if (altCoreAvailable && altErrorScore < lowestErrorScore && altErrorScore < 0.9) { ... }
            }
            ...
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "EKF3 lane switch %u", primary);
        }

    ★★ 这是 PX4 没有的一个钩子。注意三点：

      · 它是**由 vehicle 代码在即将触发 EKF failsafe 时主动调用**的，
        不是 EKF 自己的常规流程；
      · 门限是**绝对分数 < 0.9**（比常规切换的 BETTER_THRESH=0.5
        相对阈值宽松得多）—— 都要 failsafe 了，能救就救；
      · 有 **5 秒硬间隔** + 必须 healthy + 必须 tilt/yaw 都已对齐。

      ⇒ 一句话：**AP 把「换 lane」当成避免 failsafe 的最后一张牌。**
        PX4 的 selector 里【没有】这个入口。它的三段决策
        （不健康换 / 相对误差低换 /_request_instance 手动指定）
        都是「按分数走」或「听命令」，没有任何一处是
        「vehicle 说快 failsafe 了，你赶紧找个更好的 lane」。

      ⇒ 这是本章少数几个**真正的方法论差异**，而不是实现差异：
        · AP 的思路：EKF 自己要多尽一份力去避免触发 failsafe；
        · PX4 的思路：EKF 只负责给出诚实的误差分数，
          failsafe 的决策完全交给 commander，
          切换实例也由 commander 下命令。
        两边都能飞，但**排障时你会看到完全不同的现象**。
""")

    # ---- ④ 用同一份场景跑两套选择器 ----
    sub("④ ★★ 用同一份故障场景，把两套选择器都跑一遍")

    t, tr = make_truth(n_lanes=4, n_steps=600)
    ap_scores = np.zeros((len(t), 4))
    px4_scores = np.zeros((len(t), 4))
    for i in range(4):
        ap_scores[:, i] = [error_score(tr[k, i, 0], tr[k, i, 1], tr[k, i, 2])
                           for k in range(len(t))]
        px4_scores[:, i] = [px4_combined(*tr[k, i]) for k in range(len(t))]

    print("""    场景 A（30 秒，20 Hz 更新，4 个 lane）：

        lane 0  3 秒起 GPS 位置跳变        -> posTestRatio 从 0.30 冲到 9.0
        lane 1  全程健康                   -> 三个 test ratio 都在 0.3 上下
        lane 2  8 秒起 IMU 振动            -> vel/hgt test ratio 缓慢爬升
        lane 3  全程磁干扰                 -> 三项 test ratio 中等偏高
""")
    rows = []
    for k, tag in [(0, "t = 0 s"), (100, "t = 5 s"), (200, "t = 10 s"),
                   (400, "t = 20 s"), (599, "t = 30 s")]:
        rows.append([tag] + ["%.2f" % ap_scores[k, i] for i in range(4)])
    wtab(rows, ["时刻", "lane0", "lane1", "lane2", "lane3"])
    print("    （表中数字 = errorScore() = max(0.5*(velTR+posTR), hgtTR)）")
    print()

    log_ap, final_ap = run_ap_selector(ap_scores, dt=0.05)
    log_px4, final_px4 = run_px4_selector(px4_scores, dt=0.05)

    sub("场景 A：AP 的 lane 切换过程")
    if log_ap:
        rows = [["%.2f" % tt, "%d" % a, "%d" % b, why]
                for _, tt, a, b, why in log_ap]
        wtab(rows, ["时刻 (s)", "从 lane", "到 lane", "触发原因"])
    else:
        print("    （没有发生切换）\n")
    print("    最终 primary = lane %d" % final_ap)
    print()

    sub("场景 A：PX4 的 instance 切换过程")
    if log_px4:
        rows = [["%.2f" % tt, "%d" % a, "%d" % b, why]
                for _, tt, a, b, why in log_px4]
        wtab(rows, ["时刻 (s)", "从 inst", "到 inst", "触发原因"])
    else:
        print("    （没有发生切换）\n")
    print("    最终 selected = instance %d" % final_px4)
    print()

    print("""    ★ 两边都在 %.2f 秒把 lane 0 / instance 0 换掉了，但**触发原因不同**：

      · AP 的触发是 primaryErrorScore > 1.0
        （0.5*(0.25+9.0) = %.2f，远超 1.0，立刻切）；
      · PX4 的触发是 lower_error_available
        （combined_test_ratio 冲到 %.2f >= 1.0 -> warning，
          相对误差累加到 -1.0 <= -_rel_err_thresh，且距上次切换很久）。

    ★ 两个实现都要求「替代实例至少 10 秒没被选过」，
      所以在 30 秒窗口里最多切一两次。
      **这是有意设计：lane 切换会让位置/航向状态跳变，不能太频繁。**

    ★ 注意 AP 的表里「触发原因」那一列写的是 score>1 而不是 unhealthy ——
      因为在这个场景里 lane 0 的 filterStatus 还是健康的
      （只有位置新息超标），只是**误差分数太高**。
      这正是 AP 比 PX4 多出来的一条切换依据：
      PX4 的 combined_test_ratio > 1 只会置 warning 位，
      真正的切换还是要等相对误差累加够。
""" % (log_ap[0][1], 0.5 * (tr[60, 0, 0] + tr[60, 0, 1]), px4_scores[60, 0]))

    # ---- 场景 B：只有磁干扰，暴露两边的真实分歧 ----
    sub("④b ★★★ 场景 B：只让磁力计出问题 —— 这里两边会分道扬镳")

    print("""    两边的打分公式在【只用 vel/pos/hgt】时一模一样。
    但 AP 的 errorScore() 还有一项是 PX4 完全没有的：

        if (frontend->_affinity & EKF_AFFINITY_MAG) {
            score = MAX(score, 0.3f * (magTestRatio.x + magTestRatio.y + magTestRatio.z));
        }

    ⇒ 于是：**磁力计严重受扰但 vel/pos/hgt 都正常时，
      AP 会把那个 lane 判为高误差，PX4 根本看不见。**
""")

    t2, tr2 = make_truth(n_lanes=4, n_steps=600)
    # 覆盖成：所有 lane 的 vel/pos/hgt 都一样，只有 lane 2 的磁被严重干扰
    tr2[:, :, 0] = 0.30
    tr2[:, :, 1] = 0.35
    tr2[:, :, 2] = 0.25

    # 磁 test ratio 之和：lane0/1/3 健康（约 1.0），lane2 从 10 秒起飙到 30
    mag_sum = np.zeros((len(t2), 4))
    mag_sum[:, 0] = 1.0
    mag_sum[:, 1] = 1.0
    mag_sum[:, 3] = 1.0
    mag_sum[:, 2] = np.where(t2 < 10.0, 1.0, 30.0)

    ap_scores2 = np.zeros((len(t2), 4))
    px4_scores2 = np.zeros((len(t2), 4))
    for i in range(4):
        for k in range(len(t2)):
            ap_scores2[k, i] = error_score(tr2[k, i, 0], tr2[k, i, 1],
                                           tr2[k, i, 2], mag_sum[k, i], True)
            px4_scores2[k, i] = px4_combined(*tr2[k, i])

    rows = []
    for k, tag in [(0, "t = 0 s"), (200, "t = 10 s"), (400, "t = 20 s")]:
        rows.append([tag] +
                    ["%.2f" % ap_scores2[k, i] for i in range(4)] +
                    ["%.2f" % px4_scores2[k, i] for i in range(4)])
    wtab(rows, ["时刻", "AP:l0", "AP:l1", "AP:l2", "AP:l3",
                "PX4:i0", "PX4:i1", "PX4:i2", "PX4:i3"])

    # 关键：让 lane 2 / instance 2 当上初始 primary，
    # 这样才能看出「谁能发现 lane 2 有问题」。
    log_ap2, final_ap2 = run_ap_selector(ap_scores2, dt=0.05, primary=2,
                                         min_primary_s=0.0)
    log_px4b, final_px4b = run_px4_selector(px4_scores2, dt=0.05, selected=2)

    print("    初始 primary / selected 都设成 lane 2（模拟 EK3_PRIMARY=2），")
    print("    看两边会不会把它换掉。")
    print()
    print("    AP  （开 MAG affinity）：%s，最终 primary = lane %d"
          % ("切换发生" if log_ap2 else "没有切换", final_ap2))
    for _, tt, a, b, why in log_ap2:
        print("        %.2f s  lane %d -> lane %d  （%s）" % (tt, a, b, why))
    print("    PX4                    ：%s，最终 selected = instance %d"
          % ("切换发生" if log_px4b else "没有切换", final_px4b))
    for _, tt, a, b, why in log_px4b:
        print("        %.2f s  inst %d -> inst %d  （%s）" % (tt, a, b, why))
    print()

    print("""    ★★★ 这就是本实验最有价值的一格：

      **同一份磁干扰，AP 会把 lane 2 换掉，PX4 一直留在 instance 2 上。**
      （PX4 表面上"没动"，但它的磁故障处理走的是另一条完全不同的路。）

    原因不在算法优劣，而在**两个选择器对「什么算故障」的定义不同**：

      · AP 的 errorScore() 把磁力计的 test ratio 也算进去
        （条件是 affinity 里开了 MAG），所以「磁受扰」= 「误差大」；
        分数 9.0 > 1.0，直接触发 primaryErrorScore > 1.0 的切换条件。
      · PX4 的 combined_test_ratio 只看 vel / pos / hgt 三项。
        磁的问题由**另一条路径**处理：
        mag_control.cpp 的 checkMagField() 直接把
        `_control_status.flags.mag_field_disturbed` 置位，
        然后 controlMagFusion() 决定停用磁融合、改切 GSF 偏航。
        **它根本不进 selector。**

      ⇒ 两边形成了两种完全不同的「磁故障处理哲学」：

        场景                AP                       PX4
        ----------------  -----------------------  ---------------------------
        磁受扰、其他正常    抬高 errorScore          置 mag_field_disturbed
                          -> 可能换 lane             -> 停磁融合、切 GSF 偏航
        整个 lane 被带偏    换 lane 是最直接的解法   靠 GSF 把偏航单独救回来

    ★ 公平地说，两种都能工作：
      AP 把「磁」纳入 lane 健康度，用**换 lane**解决；
      PX4 把「磁」隔离在 EKF 内部，用**换观测模型**解决。

      **但调参和排障时你看到的现象完全不同**：
        AP  ：看到 "EKF3 lane switch 1" 的 GCS 告警，
              并且 EK3_ERR_THRESH 会影响它多久切一次；
        PX4  ：看不到 lane/instance 变化，
              但 estimator_status 里 mag_test_ratio 会飙高，
              innovation 日志里能看到磁新息被拒。
      这是本章最值得记住的一条实操知识。
""")
    sub("⑤ lane / instance 上限与内存")
    src_banner(["ap_imumask", "ap_imumask_def", "ap_alloc", "px4_sel_max"],
               "两边能开几个实例，由什么决定")

    rows = [
        ["AP", "EK3_IMU_MASK（位掩码，最多 6 位）", "默认 3 -> 2 个 lane",
         "运行时检查 available_memory() >= sizeof(NavEKF3_core)*n + 4096"],
        ["PX4", "EKF2_MULTI_IMU / EKF2_MULTI_MAG", "默认 0 -> 单实例",
         "编译期 EKF2_MAX_INSTANCES = 2（受限内存）/ 9"],
    ]
    wtab(rows, ["", "参数", "默认", "上限怎么来的"])

    print("""    ★ 差别很大：

      · AP 的 lane 数是**运行时位掩码**决定的，
        而且会先量一下剩余内存，不够就拒绝启动（num_cores = 0）。
        ⇒ 同一份固件，在不同板子上能跑的 lane 数可能不同。

      · PX4 的实例数是**编译期常量**，用 CONSTRAINED_MEMORY 区分
        小内存板（2）和大板（9）。
        ⇒ 同一块板子上行为完全可预测。

    ★★ 一句话总结整个实验：
      **算法同源（同一个人写的），架构不同（前端内联 vs 独立模块），
        资源策略不同（运行时协商 vs 编译期定死）。**
      这三句话基本可以概括 AP EKF3 和 PX4 EKF2 的全部关系。
""")

    # -----------------------------------------------------------------------
    # ⑥ 三组「同一件事，两种做法」
    # -----------------------------------------------------------------------
    sub("⑥ ★★ 三组「同一件事，两种做法」（前四节的数值结论落地到具体故障）")
    print("""    实验 1~4 讲的是「状态怎么排、门限怎么算、顺序怎么定、lane 怎么选」。
    这一节把它们收拢成三个真实故障场景，每个场景看两边各自怎么做。
    所有行号都来自本机源码，不是凭印象写的。
""")

    # --- 第一组 ---
    sub("⑥-1 GPS 丢失之后：一个枚举 vs 一组布尔位")
    src_banner(["ap_aiding", "ap_aid_enum", "ap_aid_loss", "ap_aid_switch", "ap_retry"],
               "AP：setAidingMode() 是一个状态机")
    src_banner(["px4_deadreck", "px4_deadreck_h", "px4_noreck", "px4_deadreck_w", "px4_noaid_tout"],
               "PX4：没有枚举，只有一组 flags + 一个现算的布尔量")

    rows = [
        ["AP", "PV_AidingMode", "枚举三态：AID_ABSOLUTE=0 / AID_NONE=1 / AID_RELATIVE=2",
         "由 setAidingMode() 每步重算"],
        ["AP", "怎么进入 AID_NONE", "判断 attAidLossCritical（姿态参考也丢了）才降级",
         "降级时 GCS 打印 stopped aiding"],
        ["AP", "多久重试", "编译期常量：tiltDriftTimeMax=15s / posRetry 10s(有速度) 7s(无速度)",
         "改不了，除非改代码（或者重编译）"],
        ["PX4", "用什么表示", "一组 _control_status.flags 布尔位：gps / ev_pos / ev_vel / aux_gpos",
         "没有互斥的枚举，可以多个同时为真"],
        ["PX4", "怎么算失效", "现算布尔：inertial_dead_reckoning = 非(velPosAiding 或 optFlow 或 airData)",
         "每步都重新算，不做状态记忆"],
        ["PX4", "多久算超时", "运行期参数 EKF2_NOAID_TOUT，默认 5000000 us = 5 s",
         "改参数就行"],
    ]
    wtab(rows, ["", "维度", "怎么做", "关键差别"])

    print("""    ★ 这一组是本章第一个「同一件事两种做法」：

      · AP 把「有没有外部位置/速度参考」压缩成**一个三态枚举**
        PV_AidingMode。好处是逻辑互斥、不可能出现自相矛盾的状态；
        代价是重试节奏被写死成常量（15 s / 10 s / 7 s），
        想改必须改代码。

      · PX4 把同一件事拆成**一堆正交的布尔位**，
        再用一行表达式把「是否在纯惯导推算」现算出来。
        好处是每个源（GPS / 外部视觉位置 / 外部视觉速度 / 辅助全局位置）
        都能独立开关，超时门限也做成了参数 EKF2_NOAID_TOUT（默认 5 s）。
        代价是没有任何地方能一眼看出「现在到底处于哪一档」。

    ★★ 落到调试上：
      AP  看日志里的 PV_AidingMode 一列（0/1/2），一看就知道档次；
          再看到 stopped aiding 这条 GCS 文本，就知道掉了姿态参考。
      PX4 看 estimator_status 里的 flags 位，
          或者直接看 EKF2 有没有刷 dead reckon time exceeded。
      一个是「状态名」，一个是「超时事件」，这是两种完全不同的排障习惯。
""")

    # --- 第二组 ---
    sub("⑥-2 磁干扰：先判场强 vs 先算新息比值")
    src_banner(["px4_magfield", "px4_wmm", "px4_magcheck", "px4_mag_dist"],
               "PX4：checkMagField() 是一个独立算子")
    src_banner(["ap_mag_gate", "ap_mag_health", "ap_mag_bad", "ap_mag_cal", "ap_magcal_def",
                "ap_mag_ef_lim"],
               "AP：健康判断长在磁融合流程内部")

    rows = [
        ["PX4", "判据", "把测到的磁场强度 |B| 和「期望强度」比，超容差就置 mag_field_disturbed",
         "期望强度优先用 GPS 给的 _mag_strength_gps"],
        ["PX4", "没有 GPS 时", "退到常数：0.45 Gauss 中心 ± 0.40 Gauss 门",
         "或者按 mag_check 位强制要求 WMM 可用"],
        ["PX4", "结果语义", "mag_field_disturbed 是一个「环境被干扰」的标志",
         "后面的融合逻辑再去决定要不要继续用磁"],
        ["AP", "判据", "逐轴算 magTestRatio[i] = sq(innov) / (sq(MAX(0.01*gate,1)) * varInnov)",
         "注意这是本章实验 2 里那个百分数门限的同一套公式"],
        ["AP", "健康怎么定", "magHealth = 三轴 testRatio 全部 < 1.0",
         "不健康就直接 return，这一步不做磁融合"],
        ["AP", "长期一致性", "用 EK3_MAG_EF_LIM（默认 50 mGauss）限制「学到的地磁 - WMM 地磁」",
         "0 表示关掉 WMM 约束"],
        ["AP", "模式选择", "EK3_MAG_CAL（Copter 默认 3）+ EK3_MAG_MASK 可按 core 强制",
         "0/1/2/3/4/5/6 各有明确语义"],
    ]
    wtab(rows, ["", "维度", "怎么做", "备注"])

    print("""    ★ 这一组看着像同一件事，其实判据落在**不同的量**上：

      · PX4 判的是**测量本身**：|B| 和期望值差多少。
        它先回答「现在的磁场环境还可信吗」，
        把这个答案放在 mag_field_disturbed 里。
        注意 0.45 / 0.40 这两个常数是写死的（没有 GPS 可用时），
        也就是说无 GPS 时它只能判断「像不像地球磁场」，判断不了「偏了多少」。

      · AP 判的是**新息**：测量和滤波器预测差多少，再除以门限平方和方差。
        它先回答「这一步的磁测量该不该用」，
        把答案放在 magHealth / faultStatus.bad_* 里。
        长期漂移另外交给 EK3_MAG_EF_LIM 管。

    ★★ 落到实操上，这就是为什么两边「磁罗盘报错」的现象不一样：
      AP 更常见的是 EK3_MAG_CAL 相关的模式切换（比如地面用简单航向融合、
        起飞后再转 3 轴融合），以及 magTestRatio 冲高导致磁融合被跳过；
      PX4 更常见的是 mag_field_disturbed 直接置位，
        然后磁融合在所有实例上一起被削弱。
      一个是「逐步否决」，一个是「整块标记」。
""")

    # --- 第三组 ---
    sub("⑥-3 偏航获取：同一个 EKF-GSF，两种用法")
    src_banner(["ap_gsf_n", "ap_gsf_models", "ap_gsf_update", "ap_gsf_corr", "ap_gsf_fuse"],
               "AP：EKF-GSF 常驻在每个 lane 里")
    src_banner(["ap_gsf_cont", "ap_gsf_throt", "ap_gsf_ymeth", "ap_gsf_validh"],
               "AP 用法一：把 GSF 偏航当常规测量，连续融合")
    src_banner(["ap_gsf_rst", "ap_gsf_usemask", "ap_gsf_rstmax", "ap_gsf_txt"],
               "AP 用法二：限额的应急重置")
    src_banner(["px4_gsf_gyro", "px4_gsf_pred", "px4_gsf_avail", "px4_gsf_fail"],
               "PX4：EKF-GSF 同样常驻，但只有一种用法")
    src_banner(["px4_gsf_reset", "px4_gsf_call1", "px4_gsf_call2", "px4_gsf_magf", "px4_gsf_gate"],
               "PX4：一次性重置 + 重置后停用磁力计")
    src_banner(["ap_srcyaw", "ap_src1yaw"],
               "AP 里 GSF 是一个可配置的一等偏航源（靠 EK3_SRCn_YAW 选）")

    rows = [
        ["AP", "模型数 / 初值", "N_MODELS_EKFGSF = 5；yaw_increment = 2*pi/5 均匀铺开",
         "每个模型一个独立 EKF"],
        ["AP", "吃什么数据", "IMU（update）+ GPS 速度（fuseVelData），有则加空速",
         "完全不需要磁力计"],
        ["AP", "是不是常驻", "是：runYawEstimatorPrediction / Correction 每个 IMU 步都调",
         "跑不跑由 EKFGSF_run_filterbank 决定"],
        ["AP", "用法一", "连续融合：fuseEulerYaw(yawFusionMethod::GSF)，140 ms 节流",
         "前提是 yaw_source_last == SourceYaw::GSF"],
        ["AP", "用法二", "限额重置：EKFGSF_resetMainFilterYaw(emergency_reset)",
         "每 core 一位的 EK3_GSF_USE_MASK 管权限"],
        ["AP", "重置次数上限", "EK3_GSF_RST_MAX，默认 2",
         "超了就不再允许应急重置"],
        ["AP", "重置时的提示", "GSF 是配置源时 INFO「yaw aligned using GPS」",
         "否则 WARNING「emergency yaw reset」"],
        ["PX4", "是不是常驻", "是：setGyroBias + predict 每个 IMU 步都调",
         "用的是增量角/增量速度，不是原始角速率"],
        ["PX4", "可用性判据", "isYawEmergencyEstimateAvailable()：isActive() 且 yawVar < sq(EKFGSF_yaw_err_max)",
         "方差没收敛就不给用"],
        ["PX4", "偏航差多少算坏", "isYawFailure()：|yaw_error| > 25 度",
         "25 度这个数写死在源码里"],
        ["PX4", "怎么用", "两处调用 resetYawToEKFGSF()：未对齐时、以及紧急重置时",
         "只有「重置」这一种用法"],
        ["PX4", "重置后的动作", "把 mag_fault 置位，停用磁力计",
         "防止磁融合再把偏航拽回去"],
        ["PX4", "重置的限流", "没有次数参数，只拒绝「刚做过 yaw reset」的情况",
         "AP 是计数，PX4 是时间窗"],
    ]
    wtab(rows, ["", "维度", "怎么做", "备注"])

    print("""    ★ 这一组特别值得讲，因为**它推翻了「AP 常驻 / PX4 应急」这个想当然的说法**：

      两边的 EKF-GSF 都是**常驻**的 ——
        AP 在 runYawEstimatorPrediction() 里每步调 yawEstimator->update()，
        PX4 在 controlGpsFusion() 里每步调 _yawEstimator.predict()。
      两边的**消费方式**也都是「重置 + 一条日志」——
        AP  MagFusion.cpp:1573 的 WARNING「emergency yaw reset」
        PX4 gps_control.cpp:256 的 ECL_WARN「GPS emergency yaw reset」，
        而且 PX4 重置完还会把 mag_fault 置位，主动停用磁力计，
        这个动作 AP 在同一个函数里没有（AP 是另走 GSF 作为源那条路）。

    ★★ 真正的差别只有一条，但影响很大：

      **AP 多了一条「连续融合」的通路。**
        当 yaw_source_last == SourceYaw::GSF 时，
        AP 会把 GSF 偏航当成一路**普通测量**，
        用 fuseEulerYaw(yawFusionMethod::GSF) 以 140 ms 的节流周期
        持续融合进主滤波器（MagFusion.cpp:254/258）。
        注意 yawFusionMethod 这个枚举（core.h:675）里
        MAGNETOMETER / GPS / GSF / STATIC / PREDICTED / EXTNAV
        是**并列**的 —— 也就是说在 AP 眼里，
        「用 GSF 定偏航」和「用磁力计定偏航」是同一层级的两件事，
        靠 EK3_SRCn_YAW 参数选。

      **PX4 里不存在这条通路。**
        GSF 偏航永远不会被当成测量逐次融合，
        只能通过 resetYawToEKFGSF() 把状态「掰」一次。

    ★★ 落到现象上：
      · AP 无磁力计飞行时，偏航是**持续被 GSF 修正**的，
        所以偏航方差能一直收敛，日志里能看到 yaw 融合一直在跑；
        应急重置有次数上限 EK3_GSF_RST_MAX（默认 2），
        用完就只剩连续融合这条路。
      · PX4 平时不碰 GSF 偏航，只在 isYawFailure() 判到
        「和 GSF 差了 25 度以上」时才掰一次，
        而且掰完立刻把磁力计打成故障。
        所以 PX4 的偏航在磁干扰下更像「跳一下然后彻底不用磁」。

    ★★★ 这就是本章最想让你带走的一个方法论：
      同一个算法（EKF-GSF，连模型数 5 都是同一个数），
      在 AP 里是**一等公民的偏航源**，
      在 PX4 里是**应急备胎**。
      读源码时如果只读算法文件，这两个结论你一个都得不出来 ——
      必须去读**谁调用它、在哪个条件下调用**。

    ★★★ 整个本章的收尾：
      ArduPilot EKF3 和 PX4 EKF2 的数学内核几乎是同一套
      （同一个误差状态、同一套门限公式、同一个选择器评分），
      真正的差异全部在**工程外壳**上：
        状态向量怎么排（实验 1）、门限用什么单位（实验 2）、
        融合顺序和调度怎么安排（实验 3）、
        多实例怎么选（实验 4）、
        以及这三个故障场景里「状态放哪、判据算什么」。
      读源码时把注意力放在这一层，比背公式有用得多。
""")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

EXPERIMENTS = {
    "1": ("★★ 状态向量并排对照与协方差组织", exp1),
    "2": ("★★ 门限单位：sigma vs 百分数 + 拒绝率", exp2),
    "3": ("★ 融合顺序并排对照与调度差异", exp3),
    "4": ("★★ lane 选择器仿真（AP vs PX4 selector）", exp4),
}


def main():
    args = sys.argv[1:]
    if not args:
        for k in sorted(EXPERIMENTS):
            print("\n" + "#" * W)
            print("#  实验 %s：%s" % (k, EXPERIMENTS[k][0]))
            print("#" * W)
            EXPERIMENTS[k][1]()
        print()
        print("=" * W)
        print("  完成。")
        print("=" * W)
        return 0

    for a in args:
        if a not in EXPERIMENTS:
            print("  未知实验：%s（可选 %s）" % (a, "/".join(sorted(EXPERIMENTS))))
            return 1
        EXPERIMENTS[a][1]()

    print()
    print("=" * W)
    print("  完成。")
    print("=" * W)
    return 0


if __name__ == "__main__":
    sys.exit(main())
