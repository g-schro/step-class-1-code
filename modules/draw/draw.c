/*
 * @brief Implementation of draw module.
 *
 * This module drives a drawing machine based on a 2-joint planer robot. It
 * makes use of the step module for driving the stepper motors.
 *
 * The following console commands are provided:
 * > draw status
 * > draw test
 * See code for details.
 *
 * MIT License
 * 
 * Copyright (c) 2021 Eugene R Schroeder
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cmd.h"
#include "console.h"
#include "log.h"
#include "module.h"

#include "draw.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#define PI_F ((float)M_PI)
#define NUM_MOTORS 2
#define NUM_CART_DIM 2
#define DFLT_MS_PER_STEP 20
#define DFLT_MM_PER_CART_SEG 2.f

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum move_state {
    MOVE_STATE_NOT_CALIB,
    MOVE_STATE_IDLE,
    MOVE_STATE_ACTIVE,
};

struct state
{
    struct draw_cfg cfg;
    uint32_t ms_per_step;
    float mm_per_cart_seg;

    // Following are used for all move types.
    enum move_state move_state;
    enum draw_move_type move_type;
    int32_t move_final_steps[NUM_MOTORS];
    float move_final_mm[NUM_CART_DIM];

    // Following are used for Cartesian moves only.
    float move_distance_mm[NUM_CART_DIM];
    int32_t last_via_point_steps[NUM_MOTORS];
    float seg_distance_mm[NUM_CART_DIM];
    uint32_t num_segments;
    uint32_t crnt_seg_num;

    // This is the tool point position. It is updated when a move completes
    // from the viewpoint of this module; the actual physical move might not be
    // complete.
    int32_t initial_steps[NUM_MOTORS];
    float initial_mm[NUM_CART_DIM];

    float steps_per_radian[NUM_MOTORS];
};
    
////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t traj_plan_start(void);
static int32_t traj_plan_continue(void);
static int32_t solve_fk(float theta_1_rad, float theta_2_rad,
                        float* x_mm, float* y_mm);
static int32_t solve_ik(float x_mm, float y_mm,
                        float* theta_1_rad, float* theta_2_rad);

static int32_t cmd_draw_status(int32_t argc, const char** argv);
static int32_t cmd_draw_test(int32_t argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct state state;

static int32_t log_level = LOG_DEFAULT;

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_draw_status,
        .help = "Get module status, usage: draw status",
    },
    {
        .name = "test",
        .func = cmd_draw_test,
        .help = "Run test, usage: draw test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "draw",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
};

static const float rad_to_deg_factor = 180.0F / PI_F;
static const float deg_to_rad_factor = PI_F / 180.0F;

#define rad_to_deg(r) ((r) * rad_to_deg_factor)
#define deg_to_rad(d) ((d) * deg_to_rad_factor)

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default draw configuration.
 *
 * @param[out] cfg The draw configuration with defaults filled in.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_get_def_cfg(struct draw_cfg* cfg)
{
    cfg->step_motor_instance[0] = CONFIG_DRAW_DFLT_STEP_INSTANCE_1;
    cfg->step_motor_instance[1] = CONFIG_DRAW_DFLT_STEP_INSTANCE_2;
    cfg->link_len_mm[0] = CONFIG_DRAW_DFLT_LINK_1_LEN_MM;
    cfg->link_len_mm[1] = CONFIG_DRAW_DFLT_LINK_2_LEN_MM;

    return 0;
}

/*
 * @brief Initialize draw instance.
 *
 * @param[in] cfg The draw configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes the draw singleton module. Generally, it should
 * not access other modules as they might not have been initialized yet. An
 * exception is the log module.
 */
int32_t draw_init(struct draw_cfg* cfg)
{  
    memset(&state, 0, sizeof(state));

    state.cfg = *cfg;
    state.move_state = MOVE_STATE_NOT_CALIB;
    state.ms_per_step = DFLT_MS_PER_STEP;
    state.mm_per_cart_seg = DFLT_MM_PER_CART_SEG;
    return 0;
}

/*
 * @brief Start draw instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts the draw singleton module, to enter normal operation.
 */
int32_t draw_start(void)
{
    int32_t rc;
    uint32_t idx;
    struct step_motor_info smi;

    rc = cmd_register(&cmd_info);
    if (rc < 0) {
        log_error("draw_start: cmd error %d\n", rc);
        return MOD_ERR_RESOURCE;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++)
    {
        rc = step_get_info(state.cfg.step_motor_instance[idx], &smi);
        if (rc != 0)
            return rc;
        state.steps_per_radian[idx] = (float)smi.steps_per_rev/(PI_F * 2.0f);
    }
    return 0;
}

/*
 * @brief Run draw instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs the draw singleton module, during normal operation.
 */
int32_t draw_run(void)
{
    if (state.move_state != MOVE_STATE_ACTIVE)
        return 0;

    // If we don't have free command slots for both motors, nothing we can do.
    if (step_get_free_queue_slots(state.cfg.step_motor_instance[0]) <= 0 ||
        step_get_free_queue_slots(state.cfg.step_motor_instance[1]) <= 0)
    {
        return 0;
    }

    return traj_plan_continue();
}

/*
 * @brief Move to a position.
 *
 * @param[in] x_mm Location on x axis in mm.
 * @param[in] y_mm Location on y axis in mm.
 * @param[in] move_type Move type (joint or Cartesian).
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_move_to(float x_mm, float y_mm, enum draw_move_type move_type)
{
    float move_final_radians[NUM_MOTORS];
    int32_t rc;
    uint32_t idx;

    if (state.move_state != MOVE_STATE_IDLE)
        return MOD_ERR_STATE;

    // If we don't have free command slots for both motors, nothing we can do.
    if (step_get_free_queue_slots(state.cfg.step_motor_instance[0]) <= 0 ||
        step_get_free_queue_slots(state.cfg.step_motor_instance[1]) <= 0)
    {
        return MOD_ERR_STATE;
    }

    rc = solve_ik(x_mm, y_mm, &move_final_radians[0], &move_final_radians[1]);
    if (rc != 0) {
        return rc;
    }

    state.move_final_mm[0] = x_mm;
    state.move_final_mm[1] = y_mm;

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        state.move_final_steps[idx] = roundf(move_final_radians[idx] *
                                             state.steps_per_radian[idx]);
    }
    state.move_type = move_type;
    return traj_plan_start();
}

/*
 * @brief Jog joint.
 *
 * @param[in] joint_idx Joint to jog (0-based).
 * @param[in] jog_degrees Number of degrees to jog.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_jog_joint(uint32_t joint_idx, float jog_degrees)
{
    int32_t jog_steps = roundf(state.steps_per_radian[joint_idx] *
                               deg_to_rad(jog_degrees));
    int32_t rc;

    if (joint_idx >= NUM_MOTORS)
        return MOD_ERR_ARG;

    if (step_get_free_queue_slots(
            state.cfg.step_motor_instance[joint_idx]) <= 0) {
        return MOD_ERR_BUSY;
    }

    if (state.move_state != MOVE_STATE_IDLE &&
        state.move_state != MOVE_STATE_NOT_CALIB) {
        return MOD_ERR_BUSY;
    }

    state.initial_steps[joint_idx] += jog_steps;

    rc = step_queue_cmd(state.cfg.step_motor_instance[joint_idx],
                        STEP_CMD_TO_P_IN_M,
                        state.initial_steps[joint_idx],
                        (jog_steps > 0 ? jog_steps : -jog_steps) *
                        state.ms_per_step);
    if (rc < 0) {
        log_error("draw_jog_joint: step_queue_cmd fails rc=%ld\n", rc);
        state.move_state = MOVE_STATE_NOT_CALIB;
    } else {
        rc = solve_fk(state.initial_steps[0]/state.steps_per_radian[0],
                      state.initial_steps[1]/state.steps_per_radian[1],
                      &state.initial_mm[0], &state.initial_mm[1]);
        if (rc < 0) {
            log_error("draw_jog_jount: solve_fk fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
        }
    }
    return rc;
}

/*
 * @brief Joint calibration.
 *
 * @param[in] theta_1_deg  Joint angle 1 in degrees.
 * @param[in] theta_2_deg  Joint angle 2 in degrees.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t draw_joint_calib(float theta_1_deg, float theta_2_deg)
{
    int rc = 0;
    float pos_mm[NUM_CART_DIM];
    float theta_rad[NUM_MOTORS] = {
        deg_to_rad(theta_1_deg),
        deg_to_rad(theta_2_deg)};
    int32_t pos_steps[NUM_MOTORS] = {
        theta_rad[0] * state.steps_per_radian[0],
        theta_rad[1] * state.steps_per_radian[1]
    };
    uint32_t idx;

    if (state.move_state != MOVE_STATE_IDLE &&
        state.move_state != MOVE_STATE_NOT_CALIB) {
        return MOD_ERR_STATE;
    }

    rc = solve_fk(theta_rad[0], theta_rad[1], &pos_mm[0], &pos_mm[1]);
    if (rc != 0) {
        log_error("draw_joint_calib: solve_fk fails rc=%d\n", rc);
        return rc;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        rc = step_set_position(state.cfg.step_motor_instance[idx],
                               pos_steps[idx]);
        if (rc != 0) {
            log_error("draw_joint_calib: step_set_position 1 fails rc=%d\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        state.initial_steps[idx] = pos_steps[idx];
    }
    for (idx = 0; idx < NUM_CART_DIM; idx++) {
        state.initial_mm[idx] = pos_mm[idx];
    }
    state.move_state = MOVE_STATE_IDLE;
    return rc;
}    

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Start the trajectory planning for a move that has been queued.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t traj_plan_start(void)
{
    int32_t rc;
    uint32_t max_steps = 0;
    uint32_t idx;
    CRIT_STATE_VAR;
    float move_abs_distance_mm;

    if (state.move_type == DRAW_MOVE_TYPE_JOINT) {
        int32_t abs_steps;
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            abs_steps = (state.move_final_steps[idx] -
                         state.initial_steps[idx]);
            if (abs_steps < 0)
                abs_steps = -abs_steps;
            if (abs_steps > max_steps)
                max_steps = abs_steps;
        }

        CRIT_BEGIN_NEST();
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            rc = step_queue_cmd(state.cfg.step_motor_instance[idx],
                                STEP_CMD_TO_P_IN_M,
                                state.move_final_steps[idx],
                                max_steps * state.ms_per_step);
            if (rc < 0)
                break;
        }
        CRIT_END_NEST();

        if (rc < 0) {
            log_error("traj_plan_start: step_queue_cmd fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }
        
        // Move is done from this module's viewpoint.
        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.initial_steps[idx] = state.move_final_steps[idx];
        for (idx = 0; idx < NUM_CART_DIM; idx++)
            state.initial_mm[idx] = state.move_final_mm[idx];
        state.move_state = MOVE_STATE_IDLE;

    } else if (state.move_type == DRAW_MOVE_TYPE_CART) {
        state.crnt_seg_num = 0;
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            state.move_distance_mm[idx] =
                (state.move_final_mm[idx] - state.initial_mm[idx]);
            state.last_via_point_steps[idx] = state.initial_steps[idx];
        }
        move_abs_distance_mm = sqrtf((state.move_distance_mm[0] *
                                      state.move_distance_mm[0]) +
                                     (state.move_distance_mm[1] *
                                      state.move_distance_mm[1]));

        state.num_segments = ceilf(move_abs_distance_mm /
                                   state.mm_per_cart_seg);
        log_debug("traj_plan_start num_segments=%lu\n", state.num_segments);
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            state.seg_distance_mm[idx] =
                state.move_distance_mm[idx] / (float)state.num_segments;
        }
        state.move_state = MOVE_STATE_ACTIVE;
    } else {
        rc = MOD_ERR_INTERNAL;
        state.move_state = MOVE_STATE_NOT_CALIB;
    }
    return rc;
}

/*
 * @brief  Continue trajector planning.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This is only used for Cartesian moves. It computes the next via point and
 * queues the move to that point to the motors.
 */
static int32_t traj_plan_continue(void)
{
    int32_t rc;
    uint32_t idx;
    float via_point_mm[NUM_CART_DIM];
    float via_point_radians[NUM_MOTORS];
    int32_t via_point_steps[NUM_MOTORS];
    int32_t num_steps;
    uint32_t max_steps = 0;
    CRIT_STATE_VAR;

    log_debug("traj_plan_continue crnt_seg_num=%lu\n", state.crnt_seg_num);

    if (state.move_type != DRAW_MOVE_TYPE_CART ||
        state.crnt_seg_num >= state.num_segments)
        return MOD_ERR_INTERNAL;

    state.crnt_seg_num++;

    if (state.crnt_seg_num >= state.num_segments) {

        // For the last segment, use the move final destination rather than
        // calculate a via point.

        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            via_point_mm[idx] = state.move_final_mm[idx];
        }
    } else {
        // Calculate the next via point.
        for (idx = 0; idx < NUM_CART_DIM; idx++) {
            via_point_mm[idx] = state.initial_mm[idx] +
                state.seg_distance_mm[idx] * (float)state.crnt_seg_num;
        }
    }

    rc = solve_ik(via_point_mm[0], via_point_mm[1],
                  &via_point_radians[0], &via_point_radians[1]);

    if (rc != 0) {
        // Presumably we encountered a point outside the workspace.
        log_error("traj_plan_continue: solve_ik fails rc=%ld\n", rc);
        state.move_state = MOVE_STATE_NOT_CALIB;
        return rc;
    }

    for (idx = 0; idx < NUM_MOTORS; idx++) {
        via_point_steps[idx] = (int32_t)(via_point_radians[idx] *
                                         state.steps_per_radian[idx]);
        num_steps = via_point_steps[idx] - state.last_via_point_steps[idx];
        if (num_steps < 0)
            num_steps = -num_steps;
        if  (num_steps > max_steps)
            max_steps = num_steps;
    }

    if (max_steps > 0) {
        CRIT_BEGIN_NEST();
        for (idx = 0; idx < NUM_MOTORS; idx++) {
            rc = step_queue_cmd(state.cfg.step_motor_instance[idx],
                                STEP_CMD_TO_P_IN_M,
                                via_point_steps[idx],
                                max_steps * state.ms_per_step);
            if (rc < 0)
                break;
        }
        CRIT_END_NEST();

        if (rc < 0) {
            log_error("traj_plan_continue: step_queue_cmd fails rc=%ld\n", rc);
            state.move_state = MOVE_STATE_NOT_CALIB;
            return rc;
        }

        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.last_via_point_steps[idx] = via_point_steps[idx];
    }

    // If this was the last segment we are done, so the final position will be
    // the initial position for the next move.

    if (state.crnt_seg_num >= state.num_segments) {
        for (idx = 0; idx < NUM_MOTORS; idx++)
            state.initial_steps[idx] = state.move_final_steps[idx];
        for (idx = 0; idx < NUM_CART_DIM; idx++)
            state.initial_mm[idx] = state.move_final_mm[idx];
        state.move_state = MOVE_STATE_IDLE;
    }
    return rc;
}

/*
 * @brief  Perform forward kinematics.
 *
 * @param [in] theta_1_rad Joint 1 angle in radians.
 * @param [in] theta_2_rad Joint 2 angle in radians.
 * @param [out] x_mm Tool point x coordinate in mm.
 * @param [out] y_mm Tool point y coordinate in mm.

 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t solve_fk(float theta_1_rad, float theta_2_rad,
                        float* x_mm, float* y_mm)
{
    float theta_sum_rad = theta_1_rad + theta_2_rad;

    *x_mm = (state.cfg.link_len_mm[0] * cosf(theta_1_rad) +
          state.cfg.link_len_mm[1] * cosf(theta_sum_rad));
    *y_mm = (state.cfg.link_len_mm[0] * sinf(theta_1_rad) +
             state.cfg.link_len_mm[1] * sinf(theta_sum_rad));

    return 0;
}

/*
 * @brief  Perform inverse kinematics.
 *
 * @param [in] x_mm Tool point x coordinate in mm.
 * @param [in] y_mm Tool point y coordinate in mm.
 * @param [out] theta_1_rad Joint 1 angle in radians.
 * @param [out] theta_2_rad Joint 2 angle in radians.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t solve_ik(float x_mm, float y_mm,
                        float* theta_1_rad, float* theta_2_rad)
{
    const float len_1_mm = state.cfg.link_len_mm[0];
    const float len_2_mm = state.cfg.link_len_mm[1];
    static int init = 0;
    static float two_len_1;
    static float two_len_1_len_2;
    static float len_1_sqr_p_len_2_sqr;
    static float len_1_sqr_m_len_2_sqr;

    float x_sqr = x_mm * x_mm;
    float y_sqr = y_mm * y_mm;
    float x_sqr_p_y_sqr = x_sqr + y_sqr;

    float beta;
    float alpha;
    float gamma;

    if (!init) {
        init = 1;
        two_len_1 = 2.0F * len_1_mm;
        two_len_1_len_2 = 2.0F * len_1_mm * len_2_mm;
        len_1_sqr_p_len_2_sqr = len_1_mm * len_1_mm + len_2_mm * len_2_mm;
        len_1_sqr_m_len_2_sqr = len_1_mm * len_1_mm - len_2_mm * len_2_mm;
    }

    alpha = acosf((x_sqr_p_y_sqr + len_1_sqr_m_len_2_sqr)/
                  (two_len_1 * sqrtf(x_sqr_p_y_sqr)));
    if (!finitef(alpha))
        return MOD_ERR_INFEASIBLE;
    beta = acosf((len_1_sqr_p_len_2_sqr - x_sqr_p_y_sqr)/
                 two_len_1_len_2);
    if (!finitef(beta))
        return MOD_ERR_INFEASIBLE;
    
    gamma = atan2f(y_mm, x_mm);
    *theta_1_rad = gamma - alpha;
    *theta_2_rad = PI_F - beta;
    return 0;
}

/*
 * @brief Console command function for "draw status".
 *
 * @param[in] argc Number of arguments, including "draw"
 * @param[in] argv Argument values, including "draw"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: draw status
 */
static int32_t cmd_draw_status(int32_t argc, const char** argv)
{
    printc("Move: state=%d type=%d move_step_steps=[%ld, %ld]\n",
           state.move_state, state.move_type, state.move_final_steps[0],
           state.move_final_steps[1]);
    printc("      last_via_point_steps=[%ld, %ld] initial_steps=[%ld, %ld]\n",
           state.last_via_point_steps[0], state.last_via_point_steps[1], 
           state.initial_steps[0], state.initial_steps[1]);
    printc_float("      initial_mm=[", state.initial_mm[0], 3, ", ");
    printc_float(NULL, state.initial_mm[1], 3, "]\n");

    printc("      num_segments=%lu crnt_seg_num=%lu\n", state.num_segments,
           state.crnt_seg_num);
    printc("ms_per_step=%lu mm_per_cart_seg=", state.ms_per_step);
    printc_float(NULL, state.mm_per_cart_seg, 1, "\n");
    return 0;
}

/*
 * @brief Console command function for "draw test".
 *
 * @param[in] argc Number of arguments, including "draw"
 * @param[in] argv Argument values, including "draw"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: draw test [<op> [<arg>]]
 */
static int32_t cmd_draw_test(int32_t argc, const char** argv)
{
    int32_t rc = 0;
    int32_t num_args;
    struct cmd_arg_val arg_vals[3];
    float theta_1;
    float theta_2;
    float x;
    float y;

    // Handle help case.
    if (argc == 2) {
        printc("Test operations and param(s) are as follows:\n"
               "  Compute IK, usage: draw test ik <x> <y>\n"
               "  Compute FK, usage: draw test fk <t1> <t2>\n");
        printc("  Calibrate for zero joint angles, usage: draw test calib\n"
               "  Jog joint, usage: draw test jog {0|1} <deg>\n"
               "  Move, usage: draw test move x y {j|c}\n");
        printc("  Set speed, usage: draw test speed <ms-per-step>\n"
               "  Set mm per seg, usage: draw test mm <mm-per-seg>\n");

        return 0;
    }

    if (strcasecmp(argv[2], "ik") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "ff", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        rc = solve_ik(arg_vals[0].val.f,
                      arg_vals[1].val.f,
                      &theta_1,
                      &theta_2);
        printc_float("theta_1=", rad_to_deg(theta_1), 3, NULL);
        printc_float(" theat2=", rad_to_deg(theta_2), 3, "\n");
    } else if (strcasecmp(argv[2], "fk") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "ff", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        rc = solve_fk(deg_to_rad(arg_vals[0].val.f),
                      deg_to_rad(arg_vals[1].val.f),
                      &x,
                      &y);
        printc_float("x=", x, 4, NULL);
        printc_float(" y=", y, 4, "\n");
    } else if (strcasecmp(argv[2], "calib") == 0) {
        rc = draw_joint_calib(0.0f, 0.0f);

    } else if (strcasecmp(argv[2], "jog") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "uf", arg_vals);
        if (num_args != 2) {
            return MOD_ERR_BAD_CMD;
        }
        if (arg_vals[0].val.u >= NUM_MOTORS) {
            printc("Invalid joint\n");
            return MOD_ERR_ARG;
        }
        rc = draw_jog_joint(arg_vals[0].val.u, arg_vals[1].val.f);
    } else if (strcasecmp(argv[2], "move") == 0) {
        enum draw_move_type move_type = DRAW_MOVE_TYPE_JOINT;
        num_args = cmd_parse_args(argc-3, argv+3, "ffs", arg_vals);
        if (num_args != 3)
            return MOD_ERR_BAD_CMD;
        if (strcasecmp(arg_vals[2].val.s, "j") == 0) {
            move_type = DRAW_MOVE_TYPE_JOINT;
        } else if (strcasecmp(arg_vals[2].val.s, "c") == 0) {
            move_type = DRAW_MOVE_TYPE_CART;
        } else {
            printc("Invalid move type '%s'\n", arg_vals[2].val.s);
            return MOD_ERR_BAD_CMD;
        }
        rc = draw_move_to(arg_vals[0].val.f, arg_vals[1].val.f, move_type);
    } else if (strcasecmp(argv[2], "speed") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "u", arg_vals);
        if (num_args != 1)
            return MOD_ERR_BAD_CMD;
        state.ms_per_step = arg_vals[0].val.u;
    } else if (strcasecmp(argv[2], "mm") == 0) {
        num_args = cmd_parse_args(argc-3, argv+3, "f", arg_vals);
        if (num_args != 1)
            return MOD_ERR_BAD_CMD;
        state.mm_per_cart_seg = arg_vals[0].val.f;
    } else {
        printc("Invalid test '%s'\n", argv[2]);
        return MOD_ERR_BAD_CMD;
    }
    printc("Return code %ld\n", rc);
    return 0;
}
