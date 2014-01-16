/* Include files */

#include <stddef.h>
#include "blas.h"
#include "cesta_sfun.h"
#include "c1_cesta.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "cesta_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[26] = { "rows", "columns", "xFrom",
  "yFrom", "xx", "xTo", "yTo", "x", "y", "dx", "dy", "modif", "deg",
  "obstacleAngle", "visDiff", "obstDist", "nargin", "nargout", "map", "xPos",
  "yPos", "angle", "range", "leftDist", "centerDist", "rightDist" };

/* Function Declarations */
static void initialize_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void initialize_params_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void enable_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void disable_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_cesta(SFc1_cestaInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_cesta(SFc1_cestaInstanceStruct
  *chartInstance);
static void set_sim_state_c1_cesta(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_st);
static void finalize_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void sf_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void c1_chartstep_c1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void initSimStructsc1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void registerMessagesc1_cesta(SFc1_cestaInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[23]);
static real_T c1_mpower(SFc1_cestaInstanceStruct *chartInstance, real_T c1_a);
static void c1_eml_scalar_eg(SFc1_cestaInstanceStruct *chartInstance);
static void c1_eml_error(SFc1_cestaInstanceStruct *chartInstance);
static void c1_b_eml_error(SFc1_cestaInstanceStruct *chartInstance);
static real_T c1_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance, const
  mxArray *c1_rad2deg, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_c_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_d_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_cesta, const char_T *c1_identifier);
static uint8_T c1_e_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_cestaInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_cesta = 0U;
}

static void initialize_params_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
}

static void enable_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_cesta(SFc1_cestaInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c1_cesta(SFc1_cestaInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  uint8_T c1_d_hoistedGlobal;
  uint8_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T *c1_centerDist;
  real_T *c1_leftDist;
  real_T *c1_rightDist;
  c1_rightDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_centerDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_leftDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(4), FALSE);
  c1_hoistedGlobal = *c1_centerDist;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *c1_leftDist;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *c1_rightDist;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = chartInstance->c1_is_active_c1_cesta;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_cesta(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_centerDist;
  real_T *c1_leftDist;
  real_T *c1_rightDist;
  c1_rightDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_centerDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_leftDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  *c1_centerDist = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 0)), "centerDist");
  *c1_leftDist = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 1)), "leftDist");
  *c1_rightDist = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c1_u, 2)), "rightDist");
  chartInstance->c1_is_active_c1_cesta = c1_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 3)), "is_active_c1_cesta");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_cesta(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
}

static void sf_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
  int32_T c1_i0;
  real_T *c1_leftDist;
  real_T *c1_xPos;
  real_T *c1_yPos;
  real_T *c1_angle;
  real_T *c1_centerDist;
  real_T *c1_rightDist;
  real_T *c1_range;
  real_T (*c1_map)[10000];
  c1_range = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_rightDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_centerDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_angle = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_yPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_xPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_leftDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_map = (real_T (*)[10000])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i0 = 0; c1_i0 < 10000; c1_i0++) {
    _SFD_DATA_RANGE_CHECK((*c1_map)[c1_i0], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c1_leftDist, 1U);
  _SFD_DATA_RANGE_CHECK(*c1_xPos, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_yPos, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_angle, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_centerDist, 5U);
  _SFD_DATA_RANGE_CHECK(*c1_rightDist, 6U);
  _SFD_DATA_RANGE_CHECK(*c1_range, 7U);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_cesta(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_cestaMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c1_chartstep_c1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  int32_T c1_i1;
  real_T c1_map[10000];
  real_T c1_xPos;
  real_T c1_yPos;
  real_T c1_angle;
  real_T c1_range;
  uint32_T c1_debug_family_var_map[26];
  real_T c1_rows;
  real_T c1_columns;
  real_T c1_xFrom;
  real_T c1_yFrom;
  real_T c1_xx;
  real_T c1_xTo;
  real_T c1_yTo;
  real_T c1_x;
  real_T c1_y;
  real_T c1_dx;
  real_T c1_dy;
  real_T c1_modif;
  real_T c1_deg;
  real_T c1_obstacleAngle;
  real_T c1_visDiff;
  real_T c1_obstDist;
  real_T c1_nargin = 5.0;
  real_T c1_nargout = 3.0;
  real_T c1_leftDist;
  real_T c1_centerDist;
  real_T c1_rightDist;
  real_T c1_b;
  real_T c1_A;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_b_y;
  real_T c1_d_x;
  real_T c1_e_x;
  real_T c1_b_A;
  real_T c1_f_x;
  real_T c1_g_x;
  real_T c1_c_y;
  real_T c1_h_x;
  real_T c1_i_x;
  real_T c1_c_A;
  real_T c1_j_x;
  real_T c1_k_x;
  real_T c1_d_y;
  real_T c1_l_x;
  real_T c1_m_x;
  real_T c1_d_A;
  real_T c1_n_x;
  real_T c1_o_x;
  real_T c1_e_y;
  real_T c1_p_x;
  real_T c1_q_x;
  real_T c1_b_xFrom;
  real_T c1_b_xTo;
  int32_T c1_i2;
  int32_T c1_r_x;
  real_T c1_b_yFrom;
  real_T c1_b_yTo;
  int32_T c1_i3;
  int32_T c1_f_y;
  real_T c1_b_b;
  real_T c1_g_y;
  real_T c1_c_b;
  real_T c1_h_y;
  real_T c1_d_b;
  real_T c1_i_y;
  real_T c1_e_b;
  real_T c1_j_y;
  real_T c1_f_b;
  real_T c1_k_y;
  real_T c1_g_b;
  real_T c1_l_y;
  real_T c1_h_b;
  real_T c1_m_y;
  real_T c1_i_b;
  real_T c1_n_y;
  real_T c1_j_b;
  real_T c1_o_y;
  real_T c1_k_b;
  real_T c1_p_y;
  real_T c1_s_x;
  real_T c1_t_x;
  real_T c1_q_y;
  real_T c1_u_x;
  real_T c1_v_x;
  real_T c1_e_A;
  real_T c1_B;
  real_T c1_w_x;
  real_T c1_r_y;
  real_T c1_x_x;
  real_T c1_s_y;
  real_T c1_t_y;
  real_T c1_y_x;
  real_T c1_ab_x;
  real_T c1_u;
  const mxArray *c1_u_y = NULL;
  real_T c1_bb_x;
  real_T c1_cb_x;
  real_T c1_b_u[4];
  const mxArray *c1_v_y = NULL;
  real_T *c1_b_rightDist;
  real_T *c1_b_centerDist;
  real_T *c1_b_leftDist;
  real_T *c1_b_range;
  real_T *c1_b_angle;
  real_T *c1_b_yPos;
  real_T *c1_b_xPos;
  real_T (*c1_b_map)[10000];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T guard5 = FALSE;
  boolean_T guard6 = FALSE;
  boolean_T guard7 = FALSE;
  boolean_T guard8 = FALSE;
  boolean_T guard9 = FALSE;
  boolean_T guard10 = FALSE;
  boolean_T guard11 = FALSE;
  c1_b_range = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_rightDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c1_b_centerDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c1_b_angle = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_yPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_xPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_leftDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_map = (real_T (*)[10000])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_xPos;
  c1_b_hoistedGlobal = *c1_b_yPos;
  c1_c_hoistedGlobal = *c1_b_angle;
  c1_d_hoistedGlobal = *c1_b_range;
  for (c1_i1 = 0; c1_i1 < 10000; c1_i1++) {
    c1_map[c1_i1] = (*c1_b_map)[c1_i1];
  }

  c1_xPos = c1_hoistedGlobal;
  c1_yPos = c1_b_hoistedGlobal;
  c1_angle = c1_c_hoistedGlobal;
  c1_range = c1_d_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 26U, 26U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_rows, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_columns, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_xFrom, 2U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_yFrom, 3U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_xx, 4U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_xTo, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_yTo, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_x, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_y, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_dx, 9U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_dy, 10U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_modif, 11U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_deg, 12U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_obstacleAngle, 13U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_visDiff, 14U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_obstDist, 15U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 16U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 17U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_map, 18U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_xPos, 19U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_yPos, 20U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_angle, 21U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_range, 22U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_leftDist, 23U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_centerDist, 24U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_rightDist, 25U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
  c1_b = c1_angle;
  c1_angle = 57.295779513082323 * c1_b;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  c1_leftDist = -1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_centerDist = -1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
  c1_rightDist = -1.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 25);
  c1_rows = 100.0;
  c1_columns = 100.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
  c1_A = c1_xPos;
  c1_b_x = c1_A;
  c1_c_x = c1_b_x;
  c1_b_y = c1_c_x / 10.0;
  c1_d_x = c1_b_y;
  c1_e_x = c1_d_x;
  c1_e_x = muDoubleScalarRound(c1_e_x);
  c1_xFrom = c1_e_x - 6.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
  c1_b_A = c1_yPos;
  c1_f_x = c1_b_A;
  c1_g_x = c1_f_x;
  c1_c_y = c1_g_x / 10.0;
  c1_h_x = c1_c_y;
  c1_i_x = c1_h_x;
  c1_i_x = muDoubleScalarRound(c1_i_x);
  c1_yFrom = c1_i_x - 6.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
  c1_c_A = c1_xPos;
  c1_j_x = c1_c_A;
  c1_k_x = c1_j_x;
  c1_d_y = c1_k_x / 10.0;
  c1_l_x = c1_d_y;
  c1_xx = c1_l_x;
  c1_m_x = c1_xx;
  c1_xx = c1_m_x;
  c1_xx = muDoubleScalarRound(c1_xx);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
  c1_xTo = c1_xx + 6.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_d_A = c1_yPos;
  c1_n_x = c1_d_A;
  c1_o_x = c1_n_x;
  c1_e_y = c1_o_x / 10.0;
  c1_p_x = c1_e_y;
  c1_q_x = c1_p_x;
  c1_q_x = muDoubleScalarRound(c1_q_x);
  c1_yTo = c1_q_x + 6.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 34);
  if (CV_EML_IF(0, 1, 0, c1_xFrom < 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 35);
    c1_xFrom = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 38);
  if (CV_EML_IF(0, 1, 1, c1_yFrom < 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 39);
    c1_yFrom = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 42);
  if (CV_EML_IF(0, 1, 2, c1_xTo > c1_columns)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
    c1_xTo = c1_columns;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 46);
  if (CV_EML_IF(0, 1, 3, c1_yTo > c1_rows)) {
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 47);
    c1_yTo = c1_rows;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
  c1_b_xFrom = c1_xFrom;
  c1_b_xTo = c1_xTo;
  c1_i2 = (int32_T)(c1_b_xTo + (1.0 - c1_b_xFrom));
  _SFD_FOR_LOOP_VECTOR_CHECK(c1_b_xFrom, 1.0, c1_b_xTo, mxDOUBLE_CLASS, c1_i2);
  c1_x = c1_b_xFrom;
  c1_r_x = 0;
  while (c1_r_x <= c1_i2 - 1) {
    c1_x = c1_b_xFrom + (real_T)c1_r_x;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
    c1_b_yFrom = c1_yFrom;
    c1_b_yTo = c1_yTo;
    c1_i3 = (int32_T)(c1_b_yTo + (1.0 - c1_b_yFrom));
    _SFD_FOR_LOOP_VECTOR_CHECK(c1_b_yFrom, 1.0, c1_b_yTo, mxDOUBLE_CLASS, c1_i3);
    c1_y = c1_b_yFrom;
    c1_f_y = 0;
    while (c1_f_y <= c1_i3 - 1) {
      c1_y = c1_b_yFrom + (real_T)c1_f_y;
      CV_EML_FOR(0, 1, 1, 1);
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
      if (CV_EML_IF(0, 1, 4, c1_map[((int32_T)(real_T)
            _SFD_EML_ARRAY_BOUNDS_CHECK("map", (int32_T)_SFD_INTEGER_CHECK("x",
              c1_x), 1, 100, 1, 0) + 100 * ((int32_T)(real_T)
             _SFD_EML_ARRAY_BOUNDS_CHECK("map", (int32_T)_SFD_INTEGER_CHECK("y",
               c1_y), 1, 100, 2, 0) - 1)) - 1] == 1.0)) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 54);
        c1_b_b = c1_x;
        c1_g_y = 10.0 * c1_b_b;
        c1_dx = c1_xPos - c1_g_y;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
        c1_c_b = c1_y;
        c1_h_y = 10.0 * c1_c_b;
        c1_dy = c1_yPos - c1_h_y;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
        c1_modif = 0.0;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 57);
        c1_d_b = c1_x;
        c1_i_y = 10.0 * c1_d_b;
        guard11 = FALSE;
        if (CV_EML_COND(0, 1, 0, c1_i_y > c1_xPos)) {
          c1_e_b = c1_y;
          c1_j_y = 10.0 * c1_e_b;
          if (CV_EML_COND(0, 1, 1, c1_j_y > c1_yPos)) {
            CV_EML_MCDC(0, 1, 0, TRUE);
            CV_EML_IF(0, 1, 5, TRUE);
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 58);
            c1_modif = 0.0;
          } else {
            guard11 = TRUE;
          }
        } else {
          guard11 = TRUE;
        }

        if (guard11 == TRUE) {
          CV_EML_MCDC(0, 1, 0, FALSE);
          CV_EML_IF(0, 1, 5, FALSE);
        }

        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 60);
        c1_f_b = c1_x;
        c1_k_y = 10.0 * c1_f_b;
        guard10 = FALSE;
        if (CV_EML_COND(0, 1, 2, c1_k_y < c1_xPos)) {
          c1_g_b = c1_y;
          c1_l_y = 10.0 * c1_g_b;
          if (CV_EML_COND(0, 1, 3, c1_l_y > c1_yPos)) {
            CV_EML_MCDC(0, 1, 1, TRUE);
            CV_EML_IF(0, 1, 6, TRUE);
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 61);
            c1_modif = 90.0;
          } else {
            guard10 = TRUE;
          }
        } else {
          guard10 = TRUE;
        }

        if (guard10 == TRUE) {
          CV_EML_MCDC(0, 1, 1, FALSE);
          CV_EML_IF(0, 1, 6, FALSE);
        }

        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 63);
        c1_h_b = c1_x;
        c1_m_y = 10.0 * c1_h_b;
        guard9 = FALSE;
        if (CV_EML_COND(0, 1, 4, c1_m_y < c1_xPos)) {
          c1_i_b = c1_y;
          c1_n_y = 10.0 * c1_i_b;
          if (CV_EML_COND(0, 1, 5, c1_n_y < c1_yPos)) {
            CV_EML_MCDC(0, 1, 2, TRUE);
            CV_EML_IF(0, 1, 7, TRUE);
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 64);
            c1_modif = 180.0;
          } else {
            guard9 = TRUE;
          }
        } else {
          guard9 = TRUE;
        }

        if (guard9 == TRUE) {
          CV_EML_MCDC(0, 1, 2, FALSE);
          CV_EML_IF(0, 1, 7, FALSE);
        }

        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 66);
        c1_j_b = c1_x;
        c1_o_y = 10.0 * c1_j_b;
        guard8 = FALSE;
        if (CV_EML_COND(0, 1, 6, c1_o_y > c1_xPos)) {
          c1_k_b = c1_y;
          c1_p_y = 10.0 * c1_k_b;
          if (CV_EML_COND(0, 1, 7, c1_p_y < c1_yPos)) {
            CV_EML_MCDC(0, 1, 3, TRUE);
            CV_EML_IF(0, 1, 8, TRUE);
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 67);
            c1_modif = 270.0;
          } else {
            guard8 = TRUE;
          }
        } else {
          guard8 = TRUE;
        }

        if (guard8 == TRUE) {
          CV_EML_MCDC(0, 1, 3, FALSE);
          CV_EML_IF(0, 1, 8, FALSE);
        }

        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 71);
        c1_deg = 0.0;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 72);
        c1_s_x = c1_dy;
        c1_t_x = c1_s_x;
        c1_q_y = muDoubleScalarAbs(c1_t_x);
        c1_u_x = c1_mpower(chartInstance, c1_dx) + c1_mpower(chartInstance,
          c1_dy);
        c1_v_x = c1_u_x;
        if (c1_v_x < 0.0) {
          c1_eml_error(chartInstance);
        }

        c1_v_x = muDoubleScalarSqrt(c1_v_x);
        c1_e_A = c1_q_y;
        c1_B = c1_v_x;
        c1_w_x = c1_e_A;
        c1_r_y = c1_B;
        c1_x_x = c1_w_x;
        c1_s_y = c1_r_y;
        c1_t_y = c1_x_x / c1_s_y;
        c1_y_x = c1_t_y;
        c1_ab_x = c1_y_x;
        guard7 = FALSE;
        if (c1_ab_x < -1.0) {
          guard7 = TRUE;
        } else {
          if (1.0 < c1_ab_x) {
            guard7 = TRUE;
          }
        }

        if (guard7 == TRUE) {
          c1_b_eml_error(chartInstance);
        }

        c1_ab_x = muDoubleScalarAsin(c1_ab_x);
        c1_u = c1_ab_x;
        c1_u_y = NULL;
        sf_mex_assign(&c1_u_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0),
                      FALSE);
        c1_deg = c1_emlrt_marshallIn(chartInstance, sf_mex_call_debug("rad2deg",
          1U, 1U, 14, c1_u_y), "rad2deg");
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 73);
        c1_obstacleAngle = c1_deg + c1_modif;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 76);
        c1_visDiff = c1_angle - c1_obstacleAngle;
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 77);
        c1_bb_x = c1_mpower(chartInstance, c1_dx) + c1_mpower(chartInstance,
          c1_dy);
        c1_obstDist = c1_bb_x;
        if (c1_obstDist < 0.0) {
          c1_eml_error(chartInstance);
        }

        c1_cb_x = c1_obstDist;
        c1_obstDist = c1_cb_x;
        c1_obstDist = muDoubleScalarSqrt(c1_obstDist);
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 78);
        sf_mex_printf("%s =\\n", "ans");
        c1_b_u[0] = c1_angle;
        c1_b_u[1] = c1_obstacleAngle;
        c1_b_u[2] = c1_obstDist;
        c1_b_u[3] = c1_visDiff;
        c1_v_y = NULL;
        sf_mex_assign(&c1_v_y, sf_mex_create("y", c1_b_u, 0, 0U, 1U, 0U, 2, 1, 4),
                      FALSE);
        sf_mex_call_debug("disp", 0U, 1U, 14, c1_v_y);
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 79);
        if (CV_EML_IF(0, 1, 9, c1_obstDist < c1_range)) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 81);
          guard5 = FALSE;
          if (CV_EML_COND(0, 1, 8, 45.0 >= c1_visDiff)) {
            if (CV_EML_COND(0, 1, 9, c1_visDiff > 15.0)) {
              CV_EML_MCDC(0, 1, 4, TRUE);
              CV_EML_IF(0, 1, 10, TRUE);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 82);
              guard6 = FALSE;
              if (CV_EML_COND(0, 1, 10, c1_rightDist == -1.0)) {
                guard6 = TRUE;
              } else if (CV_EML_COND(0, 1, 11, c1_obstDist < c1_rightDist)) {
                guard6 = TRUE;
              } else {
                CV_EML_MCDC(0, 1, 5, FALSE);
                CV_EML_IF(0, 1, 11, FALSE);
              }

              if (guard6 == TRUE) {
                CV_EML_MCDC(0, 1, 5, TRUE);
                CV_EML_IF(0, 1, 11, TRUE);
                _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 83);
                c1_rightDist = c1_obstDist;
              }
            } else {
              guard5 = TRUE;
            }
          } else {
            guard5 = TRUE;
          }

          if (guard5 == TRUE) {
            CV_EML_MCDC(0, 1, 4, FALSE);
            CV_EML_IF(0, 1, 10, FALSE);
          }

          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 89);
          guard3 = FALSE;
          if (CV_EML_COND(0, 1, 12, 15.0 >= c1_visDiff)) {
            if (CV_EML_COND(0, 1, 13, c1_visDiff > -15.0)) {
              CV_EML_MCDC(0, 1, 6, TRUE);
              CV_EML_IF(0, 1, 12, TRUE);
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 90);
              guard4 = FALSE;
              if (CV_EML_COND(0, 1, 14, c1_centerDist == -1.0)) {
                guard4 = TRUE;
              } else if (CV_EML_COND(0, 1, 15, c1_obstDist < c1_centerDist)) {
                guard4 = TRUE;
              } else {
                CV_EML_MCDC(0, 1, 7, FALSE);
                CV_EML_IF(0, 1, 13, FALSE);
              }

              if (guard4 == TRUE) {
                CV_EML_MCDC(0, 1, 7, TRUE);
                CV_EML_IF(0, 1, 13, TRUE);
                _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 91);
                c1_centerDist = c1_obstDist;
              }
            } else {
              guard3 = TRUE;
            }
          } else {
            guard3 = TRUE;
          }

          if (guard3 == TRUE) {
            CV_EML_MCDC(0, 1, 6, FALSE);
            CV_EML_IF(0, 1, 12, FALSE);
          }

          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 96);
          guard1 = FALSE;
          guard2 = FALSE;
          if (CV_EML_COND(0, 1, 16, c1_leftDist == -1.0)) {
            guard2 = TRUE;
          } else if (CV_EML_COND(0, 1, 17, -15.0 >= c1_visDiff)) {
            if (CV_EML_COND(0, 1, 18, c1_visDiff >= -45.0)) {
              guard2 = TRUE;
            } else {
              guard1 = TRUE;
            }
          } else {
            guard1 = TRUE;
          }

          if (guard2 == TRUE) {
            CV_EML_MCDC(0, 1, 8, TRUE);
            CV_EML_IF(0, 1, 14, TRUE);
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 97);
            if (CV_EML_IF(0, 1, 15, c1_obstDist < c1_leftDist)) {
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 98);
              c1_leftDist = c1_obstDist;
            }
          }

          if (guard1 == TRUE) {
            CV_EML_MCDC(0, 1, 8, FALSE);
            CV_EML_IF(0, 1, 14, FALSE);
          }
        }
      }

      c1_f_y++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 1, 0);
    c1_r_x++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -98);
  _SFD_SYMBOL_SCOPE_POP();
  *c1_b_leftDist = c1_leftDist;
  *c1_b_centerDist = c1_centerDist;
  *c1_b_rightDist = c1_rightDist;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
}

static void registerMessagesc1_cesta(SFc1_cestaInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_rad2deg;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)chartInstanceVoid;
  c1_rad2deg = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_rad2deg), &c1_thisId);
  sf_mex_destroy(&c1_rad2deg);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i4;
  int32_T c1_i5;
  int32_T c1_i6;
  real_T c1_b_inData[10000];
  int32_T c1_i7;
  int32_T c1_i8;
  int32_T c1_i9;
  real_T c1_u[10000];
  const mxArray *c1_y = NULL;
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i4 = 0;
  for (c1_i5 = 0; c1_i5 < 100; c1_i5++) {
    for (c1_i6 = 0; c1_i6 < 100; c1_i6++) {
      c1_b_inData[c1_i6 + c1_i4] = (*(real_T (*)[10000])c1_inData)[c1_i6 + c1_i4];
    }

    c1_i4 += 100;
  }

  c1_i7 = 0;
  for (c1_i8 = 0; c1_i8 < 100; c1_i8++) {
    for (c1_i9 = 0; c1_i9 < 100; c1_i9++) {
      c1_u[c1_i9 + c1_i7] = c1_b_inData[c1_i9 + c1_i7];
    }

    c1_i7 += 100;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 100, 100),
                FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

const mxArray *sf_c1_cesta_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[23];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i10;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 23), FALSE);
  for (c1_i10 = 0; c1_i10 < 23; c1_i10++) {
    c1_r0 = &c1_info[c1_i10];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i10);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i10);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[23])
{
  c1_info[0].context = "";
  c1_info[0].name = "mrdivide";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[0].fileTimeLo = 1357951548U;
  c1_info[0].fileTimeHi = 0U;
  c1_info[0].mFileTimeLo = 1319729966U;
  c1_info[0].mFileTimeHi = 0U;
  c1_info[1].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c1_info[1].name = "rdivide";
  c1_info[1].dominantType = "double";
  c1_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[1].fileTimeLo = 1346510388U;
  c1_info[1].fileTimeHi = 0U;
  c1_info[1].mFileTimeLo = 0U;
  c1_info[1].mFileTimeHi = 0U;
  c1_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[2].name = "eml_scalexp_compatible";
  c1_info[2].dominantType = "double";
  c1_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c1_info[2].fileTimeLo = 1286818796U;
  c1_info[2].fileTimeHi = 0U;
  c1_info[2].mFileTimeLo = 0U;
  c1_info[2].mFileTimeHi = 0U;
  c1_info[3].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[3].name = "eml_div";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[3].fileTimeLo = 1313347810U;
  c1_info[3].fileTimeHi = 0U;
  c1_info[3].mFileTimeLo = 0U;
  c1_info[3].mFileTimeHi = 0U;
  c1_info[4].context = "";
  c1_info[4].name = "mtimes";
  c1_info[4].dominantType = "double";
  c1_info[4].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[4].fileTimeLo = 1289519692U;
  c1_info[4].fileTimeHi = 0U;
  c1_info[4].mFileTimeLo = 0U;
  c1_info[4].mFileTimeHi = 0U;
  c1_info[5].context = "";
  c1_info[5].name = "round";
  c1_info[5].dominantType = "double";
  c1_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c1_info[5].fileTimeLo = 1343830384U;
  c1_info[5].fileTimeHi = 0U;
  c1_info[5].mFileTimeLo = 0U;
  c1_info[5].mFileTimeHi = 0U;
  c1_info[6].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/round.m";
  c1_info[6].name = "eml_scalar_round";
  c1_info[6].dominantType = "double";
  c1_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c1_info[6].fileTimeLo = 1307651238U;
  c1_info[6].fileTimeHi = 0U;
  c1_info[6].mFileTimeLo = 0U;
  c1_info[6].mFileTimeHi = 0U;
  c1_info[7].context = "";
  c1_info[7].name = "abs";
  c1_info[7].dominantType = "double";
  c1_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[7].fileTimeLo = 1343830366U;
  c1_info[7].fileTimeHi = 0U;
  c1_info[7].mFileTimeLo = 0U;
  c1_info[7].mFileTimeHi = 0U;
  c1_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c1_info[8].name = "eml_scalar_abs";
  c1_info[8].dominantType = "double";
  c1_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[8].fileTimeLo = 1286818712U;
  c1_info[8].fileTimeHi = 0U;
  c1_info[8].mFileTimeLo = 0U;
  c1_info[8].mFileTimeHi = 0U;
  c1_info[9].context = "";
  c1_info[9].name = "mpower";
  c1_info[9].dominantType = "double";
  c1_info[9].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[9].fileTimeLo = 1286818842U;
  c1_info[9].fileTimeHi = 0U;
  c1_info[9].mFileTimeLo = 0U;
  c1_info[9].mFileTimeHi = 0U;
  c1_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c1_info[10].name = "power";
  c1_info[10].dominantType = "double";
  c1_info[10].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c1_info[10].fileTimeLo = 1348191930U;
  c1_info[10].fileTimeHi = 0U;
  c1_info[10].mFileTimeLo = 0U;
  c1_info[10].mFileTimeHi = 0U;
  c1_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[11].name = "eml_scalar_eg";
  c1_info[11].dominantType = "double";
  c1_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[11].fileTimeLo = 1286818796U;
  c1_info[11].fileTimeHi = 0U;
  c1_info[11].mFileTimeLo = 0U;
  c1_info[11].mFileTimeHi = 0U;
  c1_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[12].name = "eml_scalexp_alloc";
  c1_info[12].dominantType = "double";
  c1_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[12].fileTimeLo = 1352424860U;
  c1_info[12].fileTimeHi = 0U;
  c1_info[12].mFileTimeLo = 0U;
  c1_info[12].mFileTimeHi = 0U;
  c1_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c1_info[13].name = "floor";
  c1_info[13].dominantType = "double";
  c1_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[13].fileTimeLo = 1343830380U;
  c1_info[13].fileTimeHi = 0U;
  c1_info[13].mFileTimeLo = 0U;
  c1_info[13].mFileTimeHi = 0U;
  c1_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c1_info[14].name = "eml_scalar_floor";
  c1_info[14].dominantType = "double";
  c1_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[14].fileTimeLo = 1286818726U;
  c1_info[14].fileTimeHi = 0U;
  c1_info[14].mFileTimeLo = 0U;
  c1_info[14].mFileTimeHi = 0U;
  c1_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c1_info[15].name = "eml_scalar_eg";
  c1_info[15].dominantType = "double";
  c1_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[15].fileTimeLo = 1286818796U;
  c1_info[15].fileTimeHi = 0U;
  c1_info[15].mFileTimeLo = 0U;
  c1_info[15].mFileTimeHi = 0U;
  c1_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c1_info[16].name = "mtimes";
  c1_info[16].dominantType = "double";
  c1_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[16].fileTimeLo = 1289519692U;
  c1_info[16].fileTimeHi = 0U;
  c1_info[16].mFileTimeLo = 0U;
  c1_info[16].mFileTimeHi = 0U;
  c1_info[17].context = "";
  c1_info[17].name = "sqrt";
  c1_info[17].dominantType = "double";
  c1_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[17].fileTimeLo = 1343830386U;
  c1_info[17].fileTimeHi = 0U;
  c1_info[17].mFileTimeLo = 0U;
  c1_info[17].mFileTimeHi = 0U;
  c1_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[18].name = "eml_error";
  c1_info[18].dominantType = "char";
  c1_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c1_info[18].fileTimeLo = 1343830358U;
  c1_info[18].fileTimeHi = 0U;
  c1_info[18].mFileTimeLo = 0U;
  c1_info[18].mFileTimeHi = 0U;
  c1_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c1_info[19].name = "eml_scalar_sqrt";
  c1_info[19].dominantType = "double";
  c1_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c1_info[19].fileTimeLo = 1286818738U;
  c1_info[19].fileTimeHi = 0U;
  c1_info[19].mFileTimeLo = 0U;
  c1_info[19].mFileTimeHi = 0U;
  c1_info[20].context = "";
  c1_info[20].name = "asin";
  c1_info[20].dominantType = "double";
  c1_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c1_info[20].fileTimeLo = 1343830370U;
  c1_info[20].fileTimeHi = 0U;
  c1_info[20].mFileTimeLo = 0U;
  c1_info[20].mFileTimeHi = 0U;
  c1_info[21].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c1_info[21].name = "eml_error";
  c1_info[21].dominantType = "char";
  c1_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c1_info[21].fileTimeLo = 1343830358U;
  c1_info[21].fileTimeHi = 0U;
  c1_info[21].mFileTimeLo = 0U;
  c1_info[21].mFileTimeHi = 0U;
  c1_info[22].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/asin.m";
  c1_info[22].name = "eml_scalar_asin";
  c1_info[22].dominantType = "double";
  c1_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_asin.m";
  c1_info[22].fileTimeLo = 1343830376U;
  c1_info[22].fileTimeHi = 0U;
  c1_info[22].mFileTimeLo = 0U;
  c1_info[22].mFileTimeHi = 0U;
}

static real_T c1_mpower(SFc1_cestaInstanceStruct *chartInstance, real_T c1_a)
{
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_ak;
  real_T c1_d_a;
  real_T c1_e_a;
  real_T c1_b;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_eml_scalar_eg(chartInstance);
  c1_ak = c1_c_a;
  c1_d_a = c1_ak;
  c1_eml_scalar_eg(chartInstance);
  c1_e_a = c1_d_a;
  c1_b = c1_d_a;
  return c1_e_a * c1_b;
}

static void c1_eml_scalar_eg(SFc1_cestaInstanceStruct *chartInstance)
{
}

static void c1_eml_error(SFc1_cestaInstanceStruct *chartInstance)
{
  int32_T c1_i11;
  static char_T c1_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  int32_T c1_i12;
  static char_T c1_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c1_b_u[4];
  const mxArray *c1_b_y = NULL;
  for (c1_i11 = 0; c1_i11 < 30; c1_i11++) {
    c1_u[c1_i11] = c1_cv0[c1_i11];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c1_i12 = 0; c1_i12 < 4; c1_i12++) {
    c1_b_u[c1_i12] = c1_cv1[c1_i12];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c1_y, 14, c1_b_y));
}

static void c1_b_eml_error(SFc1_cestaInstanceStruct *chartInstance)
{
  int32_T c1_i13;
  static char_T c1_cv2[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c1_u[30];
  const mxArray *c1_y = NULL;
  int32_T c1_i14;
  static char_T c1_cv3[4] = { 'a', 's', 'i', 'n' };

  char_T c1_b_u[4];
  const mxArray *c1_b_y = NULL;
  for (c1_i13 = 0; c1_i13 < 30; c1_i13++) {
    c1_u[c1_i13] = c1_cv2[c1_i13];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c1_i14 = 0; c1_i14 < 4; c1_i14++) {
    c1_b_u[c1_i14] = c1_cv3[c1_i14];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c1_y, 14, c1_b_y));
}

static real_T c1_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance, const
  mxArray *c1_rad2deg, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_rad2deg), &c1_thisId);
  sf_mex_destroy(&c1_rad2deg);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_c_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i15;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i15, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i15;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_d_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_cesta, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_is_active_c1_cesta),
    &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_cesta);
  return c1_y;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_cestaInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_cestaInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_cesta_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1276706124U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2482210843U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2190770489U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1863759241U);
}

mxArray *sf_c1_cesta_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("sUTOmbrwRsErDCIcT581nC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(100);
      pr[1] = (double)(100);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_cesta_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c1_cesta(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[9],T\"centerDist\",},{M[1],M[5],T\"leftDist\",},{M[1],M[10],T\"rightDist\",},{M[8],M[0],T\"is_active_c1_cesta\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_cesta_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_cestaInstanceStruct *chartInstance;
    chartInstance = (SFc1_cestaInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _cestaMachineNumber_,
           1,
           1,
           1,
           8,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_cestaMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_cestaMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _cestaMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"map");
          _SFD_SET_DATA_PROPS(1,2,0,1,"leftDist");
          _SFD_SET_DATA_PROPS(2,1,1,0,"xPos");
          _SFD_SET_DATA_PROPS(3,1,1,0,"yPos");
          _SFD_SET_DATA_PROPS(4,1,1,0,"angle");
          _SFD_SET_DATA_PROPS(5,2,0,1,"centerDist");
          _SFD_SET_DATA_PROPS(6,2,0,1,"rightDist");
          _SFD_SET_DATA_PROPS(7,1,1,0,"range");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,16,0,0,0,2,0,19,9);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",591,-1,3184);
        _SFD_CV_INIT_EML_IF(0,1,0,966,979,-1,1005);
        _SFD_CV_INIT_EML_IF(0,1,1,1012,1025,-1,1051);
        _SFD_CV_INIT_EML_IF(0,1,2,1058,1073,-1,1105);
        _SFD_CV_INIT_EML_IF(0,1,3,1112,1127,-1,1155);
        _SFD_CV_INIT_EML_IF(0,1,4,1268,1286,-1,3157);
        _SFD_CV_INIT_EML_IF(0,1,5,1393,1420,-1,1491);
        _SFD_CV_INIT_EML_IF(0,1,6,1509,1538,-1,1615);
        _SFD_CV_INIT_EML_IF(0,1,7,1633,1662,-1,1733);
        _SFD_CV_INIT_EML_IF(0,1,8,1751,1780,-1,1852);
        _SFD_CV_INIT_EML_IF(0,1,9,2297,2318,-1,3140);
        _SFD_CV_INIT_EML_IF(0,1,10,2366,2399,-1,2598);
        _SFD_CV_INIT_EML_IF(0,1,11,2424,2464,-1,2573);
        _SFD_CV_INIT_EML_IF(0,1,12,2651,2686,-1,2860);
        _SFD_CV_INIT_EML_IF(0,1,13,2711,2754,-1,2835);
        _SFD_CV_INIT_EML_IF(0,1,14,2909,2963,-1,3114);
        _SFD_CV_INIT_EML_IF(0,1,15,2988,3010,-1,3089);
        _SFD_CV_INIT_EML_FOR(0,1,0,1208,1228,3179);
        _SFD_CV_INIT_EML_FOR(0,1,1,1236,1256,3170);

        {
          static int condStart[] = { 1396, 1411 };

          static int condEnd[] = { 1407, 1420 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,1396,1420,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1512, 1527 };

          static int condEnd[] = { 1523, 1538 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,1512,1538,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1636, 1651 };

          static int condEnd[] = { 1647, 1662 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,1636,1662,2,4,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 1754, 1769 };

          static int condEnd[] = { 1765, 1780 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,3,1754,1780,2,6,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2370, 2387 };

          static int condEnd[] = { 2383, 2398 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,4,2370,2398,2,8,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2428, 2446 };

          static int condEnd[] = { 2442, 2464 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,5,2428,2464,2,10,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2655, 2672 };

          static int condEnd[] = { 2668, 2685 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,6,2655,2685,2,12,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2715, 2734 };

          static int condEnd[] = { 2730, 2754 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,7,2715,2754,2,14,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 2913, 2930, 2948 };

          static int condEnd[] = { 2926, 2944, 2962 };

          static int pfixExpr[] = { 0, 1, 2, -3, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,8,2913,2962,3,16,&(condStart[0]),&(condEnd[0]),
                                5,&(pfixExpr[0]));
        }

        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 100;
          dimVector[1]= 100;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c1_leftDist;
          real_T *c1_xPos;
          real_T *c1_yPos;
          real_T *c1_angle;
          real_T *c1_centerDist;
          real_T *c1_rightDist;
          real_T *c1_range;
          real_T (*c1_map)[10000];
          c1_range = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_rightDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c1_centerDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c1_angle = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_yPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_xPos = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_leftDist = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_map = (real_T (*)[10000])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_map);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_leftDist);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_xPos);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_yPos);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_angle);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_centerDist);
          _SFD_SET_DATA_VALUE_PTR(6U, c1_rightDist);
          _SFD_SET_DATA_VALUE_PTR(7U, c1_range);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _cestaMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "qXPaZER1ZBPFBnUBdbbFQG";
}

static void sf_opaque_initialize_c1_cesta(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_cestaInstanceStruct*) chartInstanceVar)->S,0);
  initialize_params_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
  initialize_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_cesta(void *chartInstanceVar)
{
  enable_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_cesta(void *chartInstanceVar)
{
  disable_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_cesta(void *chartInstanceVar)
{
  sf_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_cesta(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_cesta((SFc1_cestaInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_cesta();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_cesta(SimStruct* S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_cesta();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_cesta((SFc1_cestaInstanceStruct*)chartInfo->chartInstance,
    mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_cesta(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_cesta(S);
}

static void sf_opaque_set_sim_state_c1_cesta(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_cesta(S, st);
}

static void sf_opaque_terminate_c1_cesta(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_cestaInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_cesta_optimization_info();
    }

    finalize_c1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_cesta((SFc1_cestaInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_cesta(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_cesta((SFc1_cestaInstanceStruct*)(((ChartInfoStruct *)
      ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_cesta(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_cesta_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1827160553U));
  ssSetChecksum1(S,(3892624786U));
  ssSetChecksum2(S,(3544890243U));
  ssSetChecksum3(S,(2140783098U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_cesta(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_cesta(SimStruct *S)
{
  SFc1_cestaInstanceStruct *chartInstance;
  chartInstance = (SFc1_cestaInstanceStruct *)utMalloc(sizeof
    (SFc1_cestaInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_cestaInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_cesta;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_cesta;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_cesta;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_cesta;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_cesta;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_cesta;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_cesta;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_cesta;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_cesta;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_cesta;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_cesta;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_cesta_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_cesta(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_cesta(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_cesta(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_cesta_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
