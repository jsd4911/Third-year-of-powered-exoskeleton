#include "AK10_9_MIT.h"
#include <math.h>
#include <string.h>

/* Use full-scale 2^bits-1 quantization: e.g. zero is 0x7FFF/0x7FF.
 * The official manual's sample packer contains typographical inconsistencies
 * in its example constants; bit layout and published ranges are followed.
 */
static uint16_t quantize(float value, float low, float high, unsigned bits)
{
    const uint32_t maximum = (1u << bits) - 1u;
    const float scaled = ((value - low) / (high - low)) * (float)maximum;
    uint32_t quantized = (uint32_t)scaled; /* truncated, like manual float_to_uint */
    if (quantized > maximum) {
        quantized = maximum; /* last-bit floating rounding guard only */
    }
    return (uint16_t)quantized;
}

AK10_9_MIT_Result AK10_9_MIT_Encode(uint8_t motor_id,
                                     const AK10_9_MIT_Command *command,
                                     AK10_9_MIT_Frame *frame)
{
    uint16_t p, v, kp, kd, t;

    if (command == NULL || frame == NULL) {
        return AK10_9_MIT_ERR_NULL;
    }
    /* Only the numbered 1..255 drive IDs are accepted for this API. */
    if (motor_id == 0u) {
        return AK10_9_MIT_ERR_MOTOR_ID;
    }
    if (!isfinite(command->position_rad) ||
        !isfinite(command->velocity_rad_s) ||
        !isfinite(command->kp) ||
        !isfinite(command->kd) ||
        !isfinite(command->torque_nm)) {
        return AK10_9_MIT_ERR_NONFINITE;
    }
    if (command->position_rad < AK10_9_MIT_P_MIN ||
        command->position_rad > AK10_9_MIT_P_MAX ||
        command->velocity_rad_s < AK10_9_MIT_V_MIN ||
        command->velocity_rad_s > AK10_9_MIT_V_MAX ||
        command->kp < AK10_9_MIT_KP_MIN ||
        command->kp > AK10_9_MIT_KP_MAX ||
        command->kd < AK10_9_MIT_KD_MIN ||
        command->kd > AK10_9_MIT_KD_MAX ||
        command->torque_nm < AK10_9_MIT_T_MIN ||
        command->torque_nm > AK10_9_MIT_T_MAX) {
        return AK10_9_MIT_ERR_RANGE;
    }

    /* Order in the V3.0 MIT CAN wire format is KP, KD, P, V, T. */
    p  = quantize(command->position_rad, AK10_9_MIT_P_MIN, AK10_9_MIT_P_MAX, 16u);
    v  = quantize(command->velocity_rad_s, AK10_9_MIT_V_MIN, AK10_9_MIT_V_MAX, 12u);
    kp = quantize(command->kp, AK10_9_MIT_KP_MIN, AK10_9_MIT_KP_MAX, 12u);
    kd = quantize(command->kd, AK10_9_MIT_KD_MIN, AK10_9_MIT_KD_MAX, 12u);
    t  = quantize(command->torque_nm, AK10_9_MIT_T_MIN, AK10_9_MIT_T_MAX, 12u);

    /* Set the result only after every validation check succeeds. */
    memset(frame, 0, sizeof(*frame));
    frame->extended_id = ((uint32_t)AK10_9_MIT_FUNCTION_ID << 8u) | motor_id;
    frame->dlc = AK10_9_MIT_FRAME_DLC;
    frame->data[0] = (uint8_t)(kp >> 4u);
    frame->data[1] = (uint8_t)(((kp & 0x000Fu) << 4u) | (kd >> 8u));
    frame->data[2] = (uint8_t)(kd & 0x00FFu);
    frame->data[3] = (uint8_t)(p >> 8u);
    frame->data[4] = (uint8_t)(p & 0x00FFu);
    frame->data[5] = (uint8_t)(v >> 4u);
    frame->data[6] = (uint8_t)(((v & 0x000Fu) << 4u) | (t >> 8u));
    frame->data[7] = (uint8_t)(t & 0x00FFu);
    return AK10_9_MIT_OK;
}
