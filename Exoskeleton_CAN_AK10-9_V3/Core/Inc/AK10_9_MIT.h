#ifndef AK10_9_MIT_H
#define AK10_9_MIT_H

/* CubeMars AK10-9 V3.0 KV60, extended-CAN MIT (force control).
 * Protocol reference: AK Series Module Product Manual V3.0.1, section 4.2,
 * pages 38-41. This module ONLY ENCODEs packets: no CAN transmit API.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AK10_9_MIT_FUNCTION_ID  (0x08u)
#define AK10_9_MIT_DEFAULT_ID  (104u)
#define AK10_9_MIT_FRAME_DLC   (8u)

#define AK10_9_MIT_P_MIN      (-12.56f)
#define AK10_9_MIT_P_MAX      ( 12.56f)
#define AK10_9_MIT_V_MIN      (-28.0f)
#define AK10_9_MIT_V_MAX      ( 28.0f)
#define AK10_9_MIT_KP_MIN     (  0.0f)
#define AK10_9_MIT_KP_MAX     (500.0f)
#define AK10_9_MIT_KD_MIN     (  0.0f)
#define AK10_9_MIT_KD_MAX     (  5.0f)
#define AK10_9_MIT_T_MIN      (-54.0f)
#define AK10_9_MIT_T_MAX      ( 54.0f)

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float kp;
    float kd;
    float torque_nm;
} AK10_9_MIT_Command;

typedef struct
{
    uint32_t extended_id;     /* 0x0868 when motor_id=104 */
    uint8_t dlc;              /* 8 */
    uint8_t data[8];
} AK10_9_MIT_Frame;

typedef enum
{
    AK10_9_MIT_OK = 0,
    AK10_9_MIT_ERR_NULL,
    AK10_9_MIT_ERR_MOTOR_ID,
    AK10_9_MIT_ERR_NONFINITE,
    AK10_9_MIT_ERR_RANGE
} AK10_9_MIT_Result;

/* Returns an 8-byte, KP-first, 29-bit extended-ID MIT DATA frame.
 * Strict validation: rejects NaN/Inf and out-of-range inputs; never clips.
 * Does not enable or command a motor: the caller must implement any TX.
 */
AK10_9_MIT_Result AK10_9_MIT_Encode(uint8_t motor_id,
                                     const AK10_9_MIT_Command *command,
                                     AK10_9_MIT_Frame *frame);

#ifdef __cplusplus
}
#endif
#endif /* AK10_9_MIT_H */
