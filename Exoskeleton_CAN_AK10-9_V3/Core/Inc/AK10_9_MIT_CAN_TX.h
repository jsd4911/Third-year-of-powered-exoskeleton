#ifndef AK10_9_MIT_CAN_TX_H
#define AK10_9_MIT_CAN_TX_H

/* AK10-9 V3.0 KV60 experimental, fail-closed MIT TX state machine.
 * Independent of STM32 HAL: call only from main loop, NOT from RX ISR.
 * NOT a certified functional-safety device; physical E-stop is mandatory.
 */
#include <stdint.h>
#include "AK10_9_MIT.h"

/* PREVIEW IS THE DEFAULT. Two deliberate source edits are required to enable
 * physical transmission. Never enable for an attached exoskeleton or person.
 * Keep this definition in ONE header; do not override from a global compiler flag.
 */
#define AK10_9_TX_ENABLE                  (0)
#define AK10_9_TX_ENABLE_MAGIC            (0x00000000UL)
#define AK10_9_TX_ENABLE_EXPECTED_MAGIC   (0xA10C0DE1UL)

/* Firmware AK10_9_SW_V3.1: mode 0x0F disable is NOT yet verified in your unit.
 * Newer CubeMars V3.2.0 manual documents EID (0x0F << 8)|motor_id, DLC=0,
 * confirmation via feedback 0x29xx byte7=0x77. Keep 0 until BENCH-VERIFIED.
 */
#define AK10_9_DISABLE_CMD_CONFIRMED      (0)

#if AK10_9_TX_ENABLE && (AK10_9_TX_ENABLE_MAGIC != AK10_9_TX_ENABLE_EXPECTED_MAGIC)
#error "Physical CAN TX requires explicit lab safety enable review"
#endif
#if AK10_9_DISABLE_CMD_CONFIRMED && !AK10_9_TX_ENABLE
#error "Disable-command confirmation requires explicit physical TX build"
#endif

/* These are conservative *initial bench profile* maxima, not motor ratings.
 * Zero-only command profile until separately reviewed/changed in the lab.
 */
#define AK10_9_TX_MAX_ABS_POSITION_RAD  (0.0f)
#define AK10_9_TX_MAX_ABS_SPEED_RAD_S   (0.0f)
#define AK10_9_TX_MAX_KP                (0.0f)
#define AK10_9_TX_MAX_KD                (0.0f)
#define AK10_9_TX_MAX_ABS_TORQUE_NM     (0.0f)

#define AK10_9_TX_PERIOD_MS             (20u)  /* <=50 Hz */
#define AK10_9_TX_FEEDBACK_TIMEOUT_MS   (200u)
#define AK10_9_TX_COMMAND_TIMEOUT_MS    (100u)
#define AK10_9_TX_STOP_ACK_TIMEOUT_MS   (150u)
#define AK10_9_TX_STOP_RETRY_MS         (30u)
#define AK10_9_TX_STOP_MAX_ATTEMPTS     (3u)
#define AK10_9_TX_MIN_FEEDBACK_FRAMES   (3u)

typedef enum {
    AK10_TX_DISARMED = 0,
    AK10_TX_ARMED,
    AK10_TX_STOPPING,
    AK10_TX_FAULT_LATCHED
} AK10_9_TX_State;

typedef enum {
    AK10_TX_FAULT_NONE = 0,
    AK10_TX_FAULT_FEEDBACK,
    AK10_TX_FAULT_DRIVE,
    AK10_TX_FAULT_INTERLOCK,
    AK10_TX_FAULT_ESTOP,
    AK10_TX_FAULT_CAN_BUS,
    AK10_TX_FAULT_STALE_COMMAND,
    AK10_TX_FAULT_BAD_COMMAND,
    AK10_TX_FAULT_TX_REJECTED,
    AK10_TX_FAULT_STOP_UNCONFIRMED
} AK10_9_TX_Fault;

typedef struct {
    uint32_t now_ms;
    uint32_t feedback_last_ms;  /* main-loop snapshot of verified 0x29xx RX */
    uint32_t feedback_frames;   /* cumulative valid frames; >= 3 to arm */
    uint8_t drive_fault;        /* 0 = no fault; 0x77 disable ack separately */
    uint8_t physical_enable_ok; /* INDEPENDENT hardwired bench interlock, not a UI variable */
    uint8_t emergency_stop;     /* 1 if any emergency stop request */
    uint8_t can_error;          /* 1 for bus-off or CAN controller error */
} AK10_9_TX_Inputs;

typedef struct {
    AK10_9_TX_State state;
    AK10_9_TX_Fault fault;
    AK10_9_TX_Fault stop_reason;
    AK10_9_MIT_Command demand;
    uint32_t demand_ms;
    uint32_t last_send_ms;
    uint32_t stop_start_ms;
    uint32_t stop_last_try_ms;
    uint32_t accepted_tx_count;  /* HAL queue accepted, NOT motor confirmation */
    uint32_t preview_count;      /* computed only; no CAN transmission */
    uint32_t rejected_tx_count;
    uint8_t demand_valid;
    uint8_t has_sent;
    uint8_t stop_attempts;
    uint8_t stop_queued;
    uint8_t inhibit_requested;
} AK10_9_MIT_CAN_TX;

/* Transport callback must return 1 only if CAN mail-box accepted frame. */
typedef int (*AK10_9_TX_SendFn)(const AK10_9_MIT_Frame *frame, void *user);
/* Hardware safe-disable request (independent circuit supplied by integrator).
 * Callback invocation is NOT proof the actuator is electrically de-energized. */
typedef void (*AK10_9_TX_InhibitFn)(void *user);

void AK10_9_TX_Init(AK10_9_MIT_CAN_TX *ctx);
/* Explicit operator action. Merely calling Step() or receiving feedback NEVER arms. */
int AK10_9_TX_Arm(AK10_9_MIT_CAN_TX *ctx, const AK10_9_TX_Inputs *in,
                  uint8_t explicit_operator_confirmation);
/* Command must be renewed while armed; rejects non-finite, out-of-range,
 * and values outside the currently configured laboratory envelope. */
int AK10_9_TX_SetCommand(AK10_9_MIT_CAN_TX *ctx, uint32_t now_ms,
                         const AK10_9_MIT_Command *command);
void AK10_9_TX_RequestStop(AK10_9_MIT_CAN_TX *ctx, uint32_t now_ms);
/* Call from main loop ONLY after verifying ExtID=0x2968, DLC=8, data[7]=0x77.
 * Applicable only when verified disable command support is enabled. */
void AK10_9_TX_OnDisableAck(AK10_9_MIT_CAN_TX *ctx);
/* Tick from main loop (~5 ms); no blocking, no UART, no auto-rearm. */
void AK10_9_TX_Step(AK10_9_MIT_CAN_TX *ctx,
                     const AK10_9_TX_Inputs *in,
                     AK10_9_TX_SendFn send,
                     AK10_9_TX_InhibitFn inhibit,
                     void *user);
/* Manual reset requires external independent confirmation of power/safe state.
 * Fault reset returns to DISARMED only, it does not enable the actuator. */
int AK10_9_TX_ManualReset(AK10_9_MIT_CAN_TX *ctx,
                          const AK10_9_TX_Inputs *in,
                          uint8_t operator_ack, uint8_t power_off_verified);
const char *AK10_9_TX_StateName(AK10_9_TX_State state);
const char *AK10_9_TX_FaultName(AK10_9_TX_Fault fault);

#endif /* AK10_9_MIT_CAN_TX_H */
