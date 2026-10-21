/* Compile the production implementation against host kernel/bus substitutes. */
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

#include "../motion.c"

enum { TEST_I2C_MUTEX = 1, TEST_API_MUTEX = 2, TEST_WORKER = 10 };

static pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local ID caller_id = 20;
static _Thread_local unsigned int caller_state;
static _Thread_local INT client_lock_depth;
static BOOL bus_locked;
static MotionReport pending_report;
static BOOL report_pending;
static INT sends;
static INT fail_after = -1;
static BOOL short_write;
static BOOL wrong_sequence;
static BOOL short_report;
static BOOL send_failure;
static BOOL receive_failure;
static UW delay_ms;
static INT stage_pauses;
static INT update_delays;

typedef struct {
    INT channel;
    INT count;
} ServoWrite;

static ServoWrite writes[8192];
static size_t write_count;

ID tk_get_tid(void) { return caller_id; }
ER tk_ref_sys(T_RSYS *state)
{
    state->sysstat = caller_state;
    state->runtskid = caller_id;
    state->schedtskid = caller_id;
    return E_OK;
}

ER tk_loc_mtx(ID mutex, INT timeout)
{
    assert(timeout == TMO_FEVR);
    if (mutex == TEST_API_MUTEX) {
        assert(client_lock_depth == 0);
        assert(pthread_mutex_lock(&client_mutex) == 0);
        client_lock_depth = 1;
    } else {
        assert(mutex == TEST_I2C_MUTEX);
        assert(caller_id == TEST_WORKER);
        assert(!bus_locked);
        bus_locked = TRUE;
    }
    return E_OK;
}

ER tk_unl_mtx(ID mutex)
{
    if (mutex == TEST_API_MUTEX) {
        assert(client_lock_depth == 1);
        client_lock_depth = 0;
        assert(pthread_mutex_unlock(&client_mutex) == 0);
    } else {
        assert(mutex == TEST_I2C_MUTEX);
        assert(bus_locked);
        bus_locked = FALSE;
    }
    return E_OK;
}

ER tk_swri_dev(ID device, INT address, const void *data, SZ size, SZ *actual)
{
    const UB *bytes = data;
    assert(device == 3 && address == 0x40);
    assert(caller_id == TEST_WORKER && bus_locked);
    if (fail_after == 0) {
        fail_after = -1;
        return E_IO;
    }
    if (fail_after > 0) --fail_after;
    if (short_write) {
        short_write = FALSE;
        *actual = 0;
        return E_OK;
    }

    if (size == 5) {
        INT channel = (bytes[0] - PCA9685_LED0_ON_L) / 4;
        INT count = bytes[3] | ((bytes[4] & 15) << 8);
        assert(bytes[1] == 0 && bytes[2] == 0);
        assert(channel == 0 || channel == 1 || channel == 4
            || channel == 11 || channel == 13 || channel == 15);
        assert(write_count < sizeof(writes) / sizeof(writes[0]));
        writes[write_count++] = (ServoWrite){channel, count};
    } else {
        assert(size == 2);
    }
    *actual = size;
    return E_OK;
}

ER tk_dly_tsk(INT delay)
{
    assert(caller_id == TEST_WORKER && !bus_locked);
    assert(delay == 15 || delay == 300 || delay == 10);
    delay_ms += (UW)delay;
    if (delay == 300) ++stage_pauses;
    if (delay == 15) ++update_delays;
    sched_yield();
    return E_OK;
}

/*
 * Simulate copied command/reply delivery. The real execute_command runs in
 * worker context; multiple host callers contend on the real client mutex.
 * No host timing or task scheduling is claimed to model the target kernel.
 */
ER tk_snd_mbf(ID buffer, const void *data, INT size, INT timeout)
{
    MotionCommand command;
    ID previous_caller = caller_id;
    assert(buffer == 4 && timeout == TMO_FEVR);
    assert(client_lock_depth == 1 && !report_pending);
    assert(size == (INT)sizeof(command));
    ++sends;
    if (send_failure) return E_IO;
    memcpy(&command, data, sizeof(command));
    caller_id = TEST_WORKER;
    pending_report.sequence = command.sequence;
    pending_report.result = execute_command(&command);
    caller_id = previous_caller;
    if (wrong_sequence) ++pending_report.sequence;
    report_pending = TRUE;
    return E_OK;
}

INT tk_rcv_mbf(ID buffer, void *data, INT timeout)
{
    assert(buffer == 5 && timeout == TMO_FEVR);
    assert(client_lock_depth == 1 && report_pending);
    if (receive_failure) return E_IO;
    memcpy(data, &pending_report, sizeof(pending_report));
    report_pending = FALSE;
    return (INT)sizeof(pending_report) - (short_report ? 1 : 0);
}

INT tm_printf(const UB *format, ...)
{
    (void)format;
    return 0;
}

static void clear_trace(void)
{
    write_count = 0;
    stage_pauses = 0;
    update_delays = 0;
    delay_ms = 0;
}

static BOOL same_pose(const ArmPose *a, const ArmPose *b)
{
    return same_arm_joints(a, b) && a->gripper == b->gripper;
}

static void check_write(size_t index, INT channel, INT pulse)
{
    assert(index < write_count);
    assert(writes[index].channel == channel);
    assert(writes[index].count == (pulse * 4096) / 20000);
}

static void initialize(void)
{
    assert(motion_initialize_home() == MOTION_OK);
    assert(same_pose(&commanded_pose, &HOME));
    assert(!bus_locked && !motion_fault);
    clear_trace();
}

static void test_validation(void)
{
    static const size_t offsets[] = {
        offsetof(ArmPose, base), offsetof(ArmPose, shoulder),
        offsetof(ArmPose, elbow), offsetof(ArmPose, wrist_pitch),
        offsetof(ArmPose, wrist_rotate), offsetof(ArmPose, gripper),
    };
    static const uint16_t low[] = {400, 1300, 1100, 500, 400, 1450};
    static const uint16_t high[] = {2900, 2450, 2500, 1850, 400, 2700};
    ArmPose before = commanded_pose;
    INT previous_sends = sends;
    assert(MovJ(NULL) == MOTION_INVALID_POSE);

    for (size_t i = 0; i < 6; ++i) {
        uint16_t invalid[] = {(uint16_t)(low[i] - 1), (uint16_t)(high[i] + 1), 0, UINT16_MAX};
        for (size_t j = 0; j < 4; ++j) {
            ArmPose target = HOME;
            *(uint16_t *)((unsigned char *)&target + offsets[i]) = invalid[j];
            assert(MovJ(&target) == MOTION_INVALID_POSE);
            assert(execute_movj(&target) == MOTION_INVALID_POSE);
            assert(write_count == 0 && sends == previous_sends);
            assert(same_pose(&before, &commanded_pose));
        }
    }
}

static void test_staged_trajectory(void)
{
    /* Golden channel order/endpoints of the pre-refactor HOME -> shelf move. */
    assert(MovJ(&SHELF_SCAN) == MOTION_OK);
    assert(same_pose(&commanded_pose, &SHELF_SCAN));
    assert(write_count == 211 && update_delays == 150);
    assert(stage_pauses == 5 && delay_ms == 3750);
    for (size_t i = 0; i < 30; ++i) assert(writes[i].channel == 15);
    for (size_t i = 30; i < 90; ++i) assert(writes[i].channel == ((i % 2) ? 4 : 1));
    for (size_t i = 90; i < 120; ++i) assert(writes[i].channel == 0);
    for (size_t i = 120; i < 180; ++i) assert(writes[i].channel == ((i % 2) ? 4 : 1));
    for (size_t i = 180; i < 210; ++i) assert(writes[i].channel == 15);
    check_write(29, 15, 700);
    check_write(88, 1, 2000);
    check_write(89, 4, 1800);
    check_write(90, 0, 1560);
    check_write(119, 0, 400);
    check_write(178, 1, 1850);
    check_write(179, 4, 1900);
    check_write(209, 15, 1600);
    check_write(210, 13, 400);
    clear_trace();
}

static void test_pre_refactor_pwm_trace(void)
{
    /* Recorded from the saved pre-refactor implementation, not this API. */
    uint64_t hash = UINT64_C(14695981039346656037);
    initialize();
    assert(MoveShelfScan() == MOTION_OK);
    assert(MoveItemsScan() == MOTION_OK);
    assert(MoveShelfScan() == MOTION_OK);
    assert(MoveHome() == MOTION_OK);
    assert(write_count == 844 && update_delays == 600);
    for (size_t i = 0; i < write_count; ++i) {
        hash = (hash ^ (unsigned)writes[i].channel) * UINT64_C(1099511628211);
        hash = (hash ^ (unsigned)writes[i].count) * UINT64_C(1099511628211);
    }
    assert(hash == UINT64_C(0xaf5ae588315923c4));
    clear_trace();
}

static void test_gripper_and_home(void)
{
    assert(GripperClose() == MOTION_OK);
    assert(commanded_pose.gripper == 2700 && write_count == 30);
    for (size_t i = 0; i < write_count; ++i) assert(writes[i].channel == 11);
    clear_trace();
    assert(GripperClose() == MOTION_OK && write_count == 0);

    assert(MoveItemsScan() == MOTION_OK);
    assert(commanded_pose.base == 2900 && commanded_pose.gripper == 2700);
    for (size_t i = 0; i < write_count; ++i) assert(writes[i].channel != 11);
    assert(MoveHome() == MOTION_OK && commanded_pose.base == 1550);
    assert(MoveHome() == MOTION_OK && commanded_pose.base == 1550);
    assert(commanded_pose.gripper == 2700);

    clear_trace();
    assert(MovJ(&HOME) == MOTION_OK);
    assert(same_pose(&commanded_pose, &HOME));
    assert(write_count == 241 && update_delays == 180 && stage_pauses == 6);
    for (size_t i = 0; i < 211; ++i) assert(writes[i].channel != 11);
    for (size_t i = 211; i < 241; ++i) assert(writes[i].channel == 11);

    assert(MoveShelfScan() == MOTION_OK && commanded_pose.base == 400);
    assert(MoveItemsScan() == MOTION_OK);
    assert(MoveShelfScan() == MOTION_OK);
    assert(MoveHome() == MOTION_OK && commanded_pose.base == 1600);

    assert(GripperClose() == MOTION_OK);
    clear_trace();
    assert(GripperOpen() == MOTION_OK);
    assert(commanded_pose.gripper == 1450 && write_count == 30);
    for (size_t i = 0; i < write_count; ++i) assert(writes[i].channel == 11);
    clear_trace();
}

static void test_boundaries(void)
{
    ArmPose low = {400, 1300, 1100, 500, 400, 1450};
    ArmPose high = {2900, 2450, 2500, 1850, 400, 2700};
    assert(MovJ(&low) == MOTION_OK && same_pose(&commanded_pose, &low));
    assert(MovJ(&high) == MOTION_OK && same_pose(&commanded_pose, &high));
    initialize();
}

static void test_failures(void)
{
    assert(MoveShelfScan() == MOTION_OK);
    clear_trace();
    fail_after = 5;
    assert(MoveItemsScan() == MOTION_IO_ERROR);
    assert(commanded_pose.wrist_pitch == 1450); /* five successful 30 us steps */
    assert(commanded_pose.base == 400 && write_count == 5 && !bus_locked);
    assert(MoveHome() == MOTION_IO_ERROR && write_count == 5);
    initialize();
    short_write = TRUE;
    assert(GripperClose() == MOTION_IO_ERROR);
    assert(commanded_pose.gripper == 1450 && write_count == 0);
    initialize();
}

static void *concurrent_client(void *arg)
{
    caller_id = (ID)(uintptr_t)arg;
    for (INT i = 0; i < 4; ++i) {
        assert(MoveItemsScan() == MOTION_OK);
        assert(MoveHome() == MOTION_OK);
    }
    return NULL;
}

static void test_concurrent_callers(void)
{
    pthread_t a, b;
    INT previous_sends = sends;
    assert(pthread_create(&a, NULL, concurrent_client, (void *)(uintptr_t)21) == 0);
    assert(pthread_create(&b, NULL, concurrent_client, (void *)(uintptr_t)22) == 0);
    assert(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
    assert(sends == previous_sends + 16);
    assert(!report_pending && !bus_locked && !ipc_fault);
    clear_trace();
}

static void test_ipc_and_context_errors(void)
{
    INT previous_sends = sends;
    caller_id = 0;
    assert(MovJ(&HOME) == MOTION_WRONG_CONTEXT);
    caller_id = TEST_WORKER;
    assert(MovJ(&HOME) == MOTION_WRONG_CONTEXT);
    caller_id = 20;
    assert(sends == previous_sends);
    caller_state = TSS_INDP; /* ISR can still have an interrupted task ID. */
    assert(MoveHome() == MOTION_WRONG_CONTEXT);
    caller_state = TSS_DDSP;
    assert(MoveHome() == MOTION_WRONG_CONTEXT);
    caller_state = TSS_DINT;
    assert(MoveHome() == MOTION_WRONG_CONTEXT);
    caller_state = 0;
    assert(sends == previous_sends);

    wrong_sequence = TRUE;
    assert(MoveHome() == MOTION_IPC_ERROR);
    wrong_sequence = FALSE;
    previous_sends = sends;
    assert(MoveItemsScan() == MOTION_IPC_ERROR && sends == previous_sends);
    assert(client_lock_depth == 0);

    /* Explicit test resets below simulate fresh runs after latched IPC errors. */
    ipc_fault = FALSE;
    short_report = TRUE;
    assert(MoveHome() == MOTION_IPC_ERROR);
    short_report = FALSE;
    ipc_fault = FALSE;
    send_failure = TRUE;
    clear_trace();
    assert(MoveHome() == MOTION_IPC_ERROR && write_count == 0);
    send_failure = FALSE;
    ipc_fault = FALSE;
    receive_failure = TRUE;
    assert(MoveHome() == MOTION_IPC_ERROR);
    receive_failure = FALSE;
    report_pending = FALSE;
    assert(client_lock_depth == 0 && !bus_locked);
}

int main(void)
{
    _Static_assert(sizeof(ArmPose) == 12, "six uint16_t pulse widths");
    assert(MovJ(&HOME) == MOTION_NOT_READY);
    motion_configure(3, TEST_I2C_MUTEX, 4, 5, TEST_API_MUTEX, TEST_WORKER);
    assert(MovJ(&HOME) == MOTION_NOT_READY && write_count == 0);
    assert(MoveHome() == MOTION_NOT_READY && write_count == 0);
    assert(GripperOpen() == MOTION_NOT_READY && write_count == 0);

    caller_id = TEST_WORKER;
    assert(pca_init() == E_OK);
    caller_id = 20;
    initialize();
    test_validation();
    test_staged_trajectory();
    test_pre_refactor_pwm_trace();
    test_gripper_and_home();
    test_boundaries();
    test_failures();
    test_concurrent_callers();
    test_ipc_and_context_errors();
    puts("PASS: validation, staged PWM order/timing, poses/HOME correction, gripper isolation, faults, IPC, concurrent API callers");
    return 0;
}
