#include "AK10_9_MIT_CAN_TX.h"
#include <math.h>
#include <string.h>

static int feedback_is_fresh(const AK10_9_TX_Inputs *in)
{
    return in && in->feedback_frames >= AK10_9_TX_MIN_FEEDBACK_FRAMES &&
           (uint32_t)(in->now_ms - in->feedback_last_ms) <= AK10_9_TX_FEEDBACK_TIMEOUT_MS;
}

static int is_safe_input(const AK10_9_TX_Inputs *in)
{
    return feedback_is_fresh(in) && in->drive_fault == 0u &&
           in->physical_enable_ok != 0u && in->emergency_stop == 0u &&
           in->can_error == 0u;
}

static int within(float value, float abs_max)
{
    return isfinite(value) && value >= -abs_max && value <= abs_max;
}

static int allowed_command(const AK10_9_MIT_Command *cmd)
{
    AK10_9_MIT_Frame frame;
    if (!cmd || AK10_9_MIT_Encode(AK10_9_MIT_DEFAULT_ID, cmd, &frame) != AK10_9_MIT_OK)
        return 0;
    return within(cmd->position_rad, AK10_9_TX_MAX_ABS_POSITION_RAD) &&
           within(cmd->velocity_rad_s, AK10_9_TX_MAX_ABS_SPEED_RAD_S) &&
           isfinite(cmd->kp) && cmd->kp >= 0.0f && cmd->kp <= AK10_9_TX_MAX_KP &&
           isfinite(cmd->kd) && cmd->kd >= 0.0f && cmd->kd <= AK10_9_TX_MAX_KD &&
           within(cmd->torque_nm, AK10_9_TX_MAX_ABS_TORQUE_NM);
}

static void request_inhibit(AK10_9_MIT_CAN_TX *ctx,
                            AK10_9_TX_InhibitFn inhibit, void *user)
{
    if (!ctx->inhibit_requested) {
        ctx->inhibit_requested = 1u;
        if (inhibit) inhibit(user);
    }
}

static void enter_stop(AK10_9_MIT_CAN_TX *ctx, uint32_t now_ms, AK10_9_TX_Fault reason)
{
    if (ctx->state != AK10_TX_ARMED) return;
    ctx->demand_valid = 0u;
    ctx->state = AK10_TX_STOPPING;
    ctx->stop_reason = reason;
    ctx->stop_start_ms = now_ms;
    ctx->stop_last_try_ms = now_ms;
    ctx->stop_attempts = 0u;
    ctx->stop_queued = 0u;
}

void AK10_9_TX_Init(AK10_9_MIT_CAN_TX *ctx)
{
    if (ctx) {
        memset(ctx, 0, sizeof(*ctx));
        ctx->state = AK10_TX_DISARMED;
    }
}

int AK10_9_TX_Arm(AK10_9_MIT_CAN_TX *ctx, const AK10_9_TX_Inputs *in,
                  uint8_t explicit_operator_confirmation)
{
    if (!ctx || !in || explicit_operator_confirmation != 1u ||
        ctx->state != AK10_TX_DISARMED || !is_safe_input(in)) return 0;
    ctx->state = AK10_TX_ARMED;
    ctx->fault = AK10_TX_FAULT_NONE;
    ctx->stop_reason = AK10_TX_FAULT_NONE;
    ctx->demand_valid = 0u; /* Arm never sends by itself. */
    ctx->has_sent = 0u;
    ctx->inhibit_requested = 0u;
    return 1;
}

int AK10_9_TX_SetCommand(AK10_9_MIT_CAN_TX *ctx, uint32_t now_ms,
                         const AK10_9_MIT_Command *command)
{
    if (!ctx || ctx->state != AK10_TX_ARMED) return 0;
    if (!allowed_command(command)) {
        enter_stop(ctx, now_ms, AK10_TX_FAULT_BAD_COMMAND);
        return 0;
    }
    ctx->demand = *command;
    ctx->demand_ms = now_ms;
    ctx->demand_valid = 1u;
    return 1;
}

void AK10_9_TX_RequestStop(AK10_9_MIT_CAN_TX *ctx, uint32_t now_ms)
{
    if (ctx) enter_stop(ctx, now_ms, AK10_TX_FAULT_NONE);
}

void AK10_9_TX_OnDisableAck(AK10_9_MIT_CAN_TX *ctx)
{
#if AK10_9_TX_ENABLE && AK10_9_DISABLE_CMD_CONFIRMED
    if (ctx && ctx->state == AK10_TX_STOPPING && ctx->stop_queued) {
        if (ctx->stop_reason == AK10_TX_FAULT_NONE) {
            ctx->state = AK10_TX_DISARMED;
            ctx->fault = AK10_TX_FAULT_NONE;
        } else {
            ctx->state = AK10_TX_FAULT_LATCHED;
            ctx->fault = ctx->stop_reason;
        }
    }
#else
    (void)ctx;
#endif
}

void AK10_9_TX_Step(AK10_9_MIT_CAN_TX *ctx, const AK10_9_TX_Inputs *in,
                     AK10_9_TX_SendFn send,
                     AK10_9_TX_InhibitFn inhibit, void *user)
{
    AK10_9_MIT_Frame frame;
    if (!ctx || !in) return;

    if (ctx->state == AK10_TX_ARMED) {
        if (in->emergency_stop)        enter_stop(ctx, in->now_ms, AK10_TX_FAULT_ESTOP);
        else if (!in->physical_enable_ok) enter_stop(ctx, in->now_ms, AK10_TX_FAULT_INTERLOCK);
        else if (in->can_error)        enter_stop(ctx, in->now_ms, AK10_TX_FAULT_CAN_BUS);
        else if (!feedback_is_fresh(in)) enter_stop(ctx, in->now_ms, AK10_TX_FAULT_FEEDBACK);
        else if (in->drive_fault != 0u) enter_stop(ctx, in->now_ms, AK10_TX_FAULT_DRIVE);
        else if (!ctx->demand_valid ||
                 (uint32_t)(in->now_ms - ctx->demand_ms) > AK10_9_TX_COMMAND_TIMEOUT_MS)
                                     enter_stop(ctx, in->now_ms, AK10_TX_FAULT_STALE_COMMAND);
        if (ctx->state == AK10_TX_ARMED &&
            (!ctx->has_sent ||
             (uint32_t)(in->now_ms - ctx->last_send_ms) >= AK10_9_TX_PERIOD_MS)) {
            if (AK10_9_MIT_Encode(AK10_9_MIT_DEFAULT_ID, &ctx->demand, &frame) != AK10_9_MIT_OK) {
                enter_stop(ctx, in->now_ms, AK10_TX_FAULT_BAD_COMMAND);
            } else {
#if AK10_9_TX_ENABLE
                if (!send || !send(&frame, user)) {
                    ctx->rejected_tx_count++;
                    enter_stop(ctx, in->now_ms, AK10_TX_FAULT_TX_REJECTED);
                } else {
                    ctx->accepted_tx_count++;
                    ctx->last_send_ms = in->now_ms;
                    ctx->has_sent = 1u;
                }
#else
                /* Preview-only. NO transport callback and NO physical CAN frame. */
                (void)send;
                ctx->preview_count++;
                ctx->last_send_ms = in->now_ms;
                ctx->has_sent = 1u;
#endif
            }
        }
    }

    if (ctx->state != AK10_TX_STOPPING) return;
    /* An independent hardware inhibit output must be supplied by the system.
     * Software's request alone does not guarantee the motor has stopped. */
    request_inhibit(ctx, inhibit, user);

#if AK10_9_TX_ENABLE && AK10_9_DISABLE_CMD_CONFIRMED
    if ((uint32_t)(in->now_ms - ctx->stop_start_ms) >= AK10_9_TX_STOP_ACK_TIMEOUT_MS) {
        ctx->state = AK10_TX_FAULT_LATCHED;
        ctx->fault = AK10_TX_FAULT_STOP_UNCONFIRMED;
        return;
    }
    if (in->can_error) {
        ctx->state = AK10_TX_FAULT_LATCHED;
        ctx->fault = AK10_TX_FAULT_STOP_UNCONFIRMED;
        return;
    }
    if (ctx->stop_attempts < AK10_9_TX_STOP_MAX_ATTEMPTS &&
        (ctx->stop_attempts == 0u ||
         (uint32_t)(in->now_ms - ctx->stop_last_try_ms) >= AK10_9_TX_STOP_RETRY_MS)) {
        memset(&frame, 0, sizeof(frame));
        frame.extended_id = (0x0Fu << 8) | (uint32_t)AK10_9_MIT_DEFAULT_ID;
        frame.dlc = 0u; /* V3.2.0 manual: motor disable uses DLC=0 */
        ctx->stop_attempts++;
        ctx->stop_last_try_ms = in->now_ms;
        if (send && send(&frame, user)) ctx->stop_queued = 1u;
        else ctx->rejected_tx_count++;
    }
#else
    /* Firmware-specific disable is unverified or real TX is disabled:
     * never claim a completed stop; enter a locked, externally-stop-required state. */
    (void)frame;
    (void)send;
    ctx->state = AK10_TX_FAULT_LATCHED;
    ctx->fault = AK10_TX_FAULT_STOP_UNCONFIRMED;
#endif
}

int AK10_9_TX_ManualReset(AK10_9_MIT_CAN_TX *ctx,
                          const AK10_9_TX_Inputs *in,
                          uint8_t operator_ack, uint8_t power_off_verified)
{
    if (!ctx || !in || ctx->state != AK10_TX_FAULT_LATCHED ||
        operator_ack != 1u || power_off_verified != 1u ||
        in->physical_enable_ok != 0u || in->can_error != 0u) return 0;
    /* DO NOT re-arm automatically on reset or power cycling. */
    AK10_9_TX_Init(ctx);
    return 1;
}

const char *AK10_9_TX_StateName(AK10_9_TX_State state)
{
    switch (state) {
        case AK10_TX_DISARMED: return "DISARMED";
        case AK10_TX_ARMED: return "ARMED";
        case AK10_TX_STOPPING: return "STOPPING";
        case AK10_TX_FAULT_LATCHED: return "FAULT_LATCHED";
        default: return "UNKNOWN";
    }
}

const char *AK10_9_TX_FaultName(AK10_9_TX_Fault fault)
{
    switch (fault) {
        case AK10_TX_FAULT_NONE: return "NONE";
        case AK10_TX_FAULT_FEEDBACK: return "FEEDBACK_STALE";
        case AK10_TX_FAULT_DRIVE: return "DRIVE_FAULT";
        case AK10_TX_FAULT_INTERLOCK: return "INTERLOCK_OPEN";
        case AK10_TX_FAULT_ESTOP: return "EMERGENCY_STOP";
        case AK10_TX_FAULT_CAN_BUS: return "CAN_BUS_ERROR";
        case AK10_TX_FAULT_STALE_COMMAND: return "COMMAND_STALE";
        case AK10_TX_FAULT_BAD_COMMAND: return "COMMAND_REJECTED";
        case AK10_TX_FAULT_TX_REJECTED: return "CAN_TX_REJECTED";
        case AK10_TX_FAULT_STOP_UNCONFIRMED: return "STOP_UNCONFIRMED";
        default: return "UNKNOWN";
    }
}
