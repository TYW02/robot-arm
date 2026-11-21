/*
 * cli_bridge.c - connects the operator CLI (components/cli) to the mission
 * Owner: Reiner
 *
 * Included by integration/main.c just before the mission task, so it can use
 * main.c's mission helpers (latch, drive queue, direction readings) without
 * making them global. Not compiled on its own.
 *
 *   start   : sets START_FLG; the mission task waits on it (tk_wai_flg)
 *   stop    : mission_latch_stop("operator stop"); the drive task brakes on
 *             its next check (<= 10 ms) and the mission task parks
 *   reset   : once the mission task is parked, terminate it, clear the
 *             latch and restart it, so it waits for 'start' again
 *   status  : state (mapping walls, searching, gripping, delivering, stopped),
 *             held object, objects delivered, gyro health
 *   debug   : manual drive goes through the same drive queue and IMU
 *             interlock as the mission; accepted only in IDLE
 *
 * The UI task runs on core 0 (with USB); the mission, drive and IMU monitor
 * tasks run on core 1, so every operator command crosses cores.
 */

#include "../components/cli/cli_utkernel.h"

/* 1 = the mission waits for 'start'; 0 = old behaviour (starts by itself). */
#ifndef MISSION_WAIT_FOR_START
#define MISSION_WAIT_FOR_START   1
#endif

#define CLI_TASK_PRIORITY        9      /* below camera RX/TX on core 0 */
#define START_FLG_BIT            1U

typedef enum {
    PHASE_BOOT,      /* IMU start-up checks, mission task not running yet */
    PHASE_IDLE,      /* waiting for 'start'                               */
    PHASE_MAP,       /* 360-degree sweep recording the wall markers       */
    PHASE_SEARCH,    /* looking for / approaching an object               */
    PHASE_GRIP,      /* closing the claw on an object                     */
    PHASE_DELIVER,   /* finding / driving to the destination              */
} mission_phase_t;

static volatile mission_phase_t bridge_phase = PHASE_BOOT;
static volatile INT bridge_holding;          /* object id in the claw, 0 = none */
static volatile INT bridge_delivered;        /* objects delivered since boot    */
static ID           bridge_start_flg;


/* ---------- Hooks called from the mission task ---------- */

static void cli_bridge_set_phase(mission_phase_t phase, INT object_id)
{
    bridge_phase = phase;
    bridge_holding = object_id;
}

/* Top of the mission task: wait for 'start' while still honouring STOP. */
static void cli_bridge_wait_for_start(void)
{
#if MISSION_WAIT_FOR_START
    UINT pattern;

    cli_bridge_set_phase(PHASE_IDLE, 0);
    robot_logf((UB *)"[mission] READY: type 'start'\n");

    if (bridge_start_flg <= E_OK) {           /* no flag: start by itself */
        cli_bridge_set_phase(PHASE_MAP, 0);
        return;
    }

    for (;;) {
        mission_checkpoint();                 /* parks here if stopped */

        if (tk_wai_flg(bridge_start_flg, START_FLG_BIT,
                       TWF_ORW | TWF_BITCLR, &pattern, 200) == E_OK)
            break;
    }
    robot_logf((UB *)"[mission] START by operator\n");
#endif
    cli_bridge_set_phase(PHASE_MAP, 0);     /* the wall-mapping sweep runs next */
}

static void cli_bridge_delivered(INT object_id)
{
    (void)object_id;
    bridge_delivered++;
    cli_bridge_set_phase(PHASE_SEARCH, 0);
}

/* ---------- Command handlers (UI task, core 0) ---------- */

static const char *phase_name(void)
{
    if (mission_is_stopped()) return "STOPPED";
    switch (bridge_phase) {
    case PHASE_BOOT:    return "STARTING (IMU checks)";
    case PHASE_IDLE:    return "IDLE";
    case PHASE_MAP:     return "MAPPING walls (360 deg sweep)";
    case PHASE_SEARCH:  return "SEARCHING for object";
    case PHASE_GRIP:    return "GRIPPING";
    case PHASE_DELIVER: return "DELIVERING";
    }
    return "?";
}

static const char *stop_reason(void)
{
    const char *r = mission_stop_reason;
    return r ? r : "none";
}

static BOOL is_idle(void)
{
    return bridge_phase == PHASE_IDLE && !mission_is_stopped();
}

static void reject_unless_idle(char *reply)
{
    if (mission_is_stopped())
        tm_sprintf((UB *)reply, (UB *)"rejected: stopped (%s); type reset", stop_reason());
    else if (bridge_phase == PHASE_BOOT)
        tm_sprintf((UB *)reply, (UB *)"rejected: not ready yet (IMU start-up)");
    else
        tm_sprintf((UB *)reply, (UB *)"rejected: mission running; stop first");
}

/* The status text, in one place: the USB console prints it, and the Wi-Fi
   command port sends it back. out needs >= 256 bytes. */
static void format_status(char *out, size_t len)
{
    imu_heading_reading_t s = {0};
    char held[16];

    (void)len;
    imu_heading_read(&s);

    if (bridge_holding > 0) tm_sprintf((UB *)held, (UB *)"object %d", (INT)bridge_holding);
    else                    tm_sprintf((UB *)held, (UB *)"-");

    tm_sprintf((UB *)out,
               (UB *)"state: %s   holding: %s   delivered: %d\n"
                     "stop: %s\n"
                     "imu: %s   turned %d deg since start-up",
               phase_name(), held, (INT)bridge_delivered,
               stop_reason(),
               mission_imu_healthy(&s) ? "OK" : "NOT READY/FAULT",
               (INT)(s.relative_deg + (s.relative_deg < 0 ? -0.5f : 0.5f)));
}

static void print_status(void)
{
    char text[256];

    format_status(text, sizeof text);
    tm_printf((UB *)"%s\n", text);
}

static void do_start(char *reply)
{
#if MISSION_WAIT_FOR_START
    if (!is_idle()) { reject_unless_idle(reply); return; }
    if (tk_set_flg(bridge_start_flg, START_FLG_BIT) != E_OK)
        tm_sprintf((UB *)reply, (UB *)"error: could not signal the mission");
#else
    tm_sprintf((UB *)reply, (UB *)"rejected: this build starts by itself");
#endif
}

static void do_reset(char *reply)
{
    T_RTSK r;

    if (!mission_is_stopped()) {
        tm_sprintf((UB *)reply, (UB *)"rejected: not stopped");
        return;
    }
    if (task_state_id <= 0) {
        /* Stopped before the mission task existed: usermain has parked. */
        tm_sprintf((UB *)reply, (UB *)"rejected: stopped during start-up; power-cycle the car");
        return;
    }
    if (tk_ref_tsk(task_state_id, &r) != E_OK) {
        tm_sprintf((UB *)reply, (UB *)"error: cannot read mission task state");
        return;
    }
    BOOL parked  = (r.tskstat & TTS_WAI) && r.tskwait == TTW_SLP;
    BOOL ended   = (r.tskstat & TTS_DMT) != 0;   /* returned, e.g. sweep failed */
    if (!parked && !ended) {
        tm_sprintf((UB *)reply, (UB *)"busy: mission still stopping, try again");
        return;
    }

    /* Parked in mission_park() (holds no mutex and no motor command) or
       already ended: either way it can be restarted from the top. */
    if (parked)
        (void)tk_ter_tsk(task_state_id);
    claw_release();
    (void)tk_clr_flg(bridge_start_flg, ~START_FLG_BIT);

    UINT intsts;
    (void)ISpinLock(&mission_guard_lock, &intsts);
    mission_stop_latched = FALSE;
    mission_stop_reason = NULL;
    (void)ISpinUnlock(&mission_guard_lock, intsts);

    cli_bridge_set_phase(PHASE_IDLE, 0);

    if (tk_sta_tsk(task_state_id, 0) != E_OK) {
        mission_latch_stop("mission task restart failed");
        tm_sprintf((UB *)reply, (UB *)"error: mission restart failed");
        return;
    }
    robot_logf((UB *)"[mission] RESET by operator\n");
    tm_sprintf((UB *)reply, (UB *)"ok: reset, IDLE (type start)");
}

#if CLI_ENABLE_MANUAL
static void do_drive(const cmd_msg_t *m, char *reply)
{
    if (!is_idle()) { reject_unless_idle(reply); return; }
    if (task_drive_id <= 0) { tm_sprintf((UB *)reply, (UB *)"rejected: drive task not running"); return; }

    DriveCommand command = {0};
    command.left  = m->a;
    command.right = m->b;
    command.ms    = m->c;
    if (tk_snd_mbf(drive_mbf_id, &command, sizeof command, TMO_POL) != E_OK)
        tm_sprintf((UB *)reply, (UB *)"busy: drive queue full, try again");
}

static void do_claw(const cmd_msg_t *m, char *reply)
{
    if (mission_is_stopped() || !(bridge_phase == PHASE_IDLE || bridge_phase == PHASE_BOOT)) {
        reject_unless_idle(reply);
        return;
    }
    switch (m->a) {
    case CLI_CLAW_OFF:   claw_release(); break;
    case CLI_CLAW_OPEN:  claw_open();    break;
    case CLI_CLAW_CLOSE: claw_close();   break;
    default:             claw_angle(m->a); break;
    }
}
#endif

/* config.execute: runs in the UI task for every valid command. */
static void cli_bridge_execute(const cmd_msg_t *m, char *reply, size_t len)
{
    (void)len;   /* every reply below is shorter than CLI_REPLY_MAX */

    switch (m->type) {
    case CMD_STOP:
        mission_latch_stop("operator stop");     /* first: fastest path */
        break;
    case CMD_START:
        do_start(reply);
        break;
    case CMD_RESET:
        do_reset(reply);
        break;
    case CMD_STATUS:
        print_status();
        reply[0] = '\0';
        break;
#if CLI_ENABLE_MANUAL
    case CMD_DRIVE:
        do_drive(m, reply);
        break;
    case CMD_CLAW:
        do_claw(m, reply);
        break;
#endif
    default:
        tm_sprintf((UB *)reply, (UB *)"error: command not supported in this build");
        break;
    }
}

/* ---------- Start-up (usermain) ---------- */

static void cli_bridge_start(void)
{
    T_CFLG cf = { .flgatr = TA_TFIFO | TA_WSGL, .iflgptn = 0 };

    bridge_start_flg = tk_cre_flg(&cf);
    if (bridge_start_flg <= E_OK) {
        robot_logf((UB *)"[cli] start flag creation failed: %d\n", bridge_start_flg);
        return;
    }

    cli_task_config_t cfg = {
        .execute     = cli_bridge_execute,
        .priority    = CLI_TASK_PRIORITY,
        .stack_bytes = CLI_STACK_MIN,
        .status_text = format_status,
    };
    ID id = cli_utkernel_start(&cfg);
    if (id <= E_OK)
        robot_logf((UB *)"[cli] UI task start failed: %d\n", id);
}
