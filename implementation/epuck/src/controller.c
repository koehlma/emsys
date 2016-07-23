#include "controller.h"
#include "hal.h"
#include "proximity-filter.h"

void controller_reset(Controller* c, Sensors* sens) {
    approx_reset(&c->approx, sens);
    c->origin.x = sens->current.x;
    c->origin.y = sens->current.y;
    assert(!map_invalid_pos(map_discretize(c->origin)));
    blind_reset(&c->blind);
    mod_reset(&c->moderator);
    pa_reset(&c->pickup_artist);
    pe_reset(&c->path_exec);
    pf_reset(&c->path_finder);
    proximity_reset(&c->prox_map, sens);
    rhr_reset(&c->rhr);
    irs_reset(&c->ir_stab);
    tce_reset(&c->cop_eyes);
    vd_reset(&c->vic_dir);
    vf_reset(&c->vic_finder);
}

static unsigned int inquire_moderator_permission(Controller* c, Sensors* sens);
static void inquire_new_vicdir_data(VFState* vf, Sensors* sens);
static void inquire_blind_decision(Controller* c, Sensors* sens);
static void inquire_eyes_decision(Controller* c, Sensors* sens);
static void reset_appropriately(enum BlindRunChoice old_choice, Controller* c);
static void run_victim_finder(Controller* c, Sensors* sens);
static void run_path_finder_executer(Controller* c, Sensors* sens);

void controller_step(Controller* c, Sensors* sens) {
    enum BlindRunChoice old_choice;

    /* Zeroth, check whether we *want* to execute at all. */
    if (!inquire_moderator_permission(c, sens)) {
        /* Play dead. */
        return;
    }

    if (c->moderator.found_victim_xy) {
        /* If the moderator didn't set 'found_victim_xy', but
         * allows us to continue, then victim_xy isn't known to anyone. */
        if (sens->victim_attached) {
            filter_prox_attached(sens);
        } else {
            filter_prox_detached(c->moderator.victim, sens);
        }
    }
    approx_step(&c->approx, sens);
    inquire_new_vicdir_data(&c->vic_finder, sens);

    /* First, the eyes decide whether we need an interruption
     * (in order to execute Victim Direction). */
    inquire_eyes_decision(c, sens);

    /* Now the traffic cop decides "who is allowed to drive". */
    old_choice = c->blind.run_choice;
    inquire_blind_decision(c, sens);

    /* Do we need to reset any of the state machines? */
    if (old_choice != c->blind.run_choice) {
        reset_appropriately(old_choice, c);
    }

    /* Now we know what to do. */
    switch (c->blind.run_choice) {
    case BLIND_RUN_CHOICE_none:
        /* Nothing to do here. */
        break;
    case BLIND_RUN_CHOICE_rhr:
        rhr_step(&c->rhr, sens);
        break;
    case BLIND_RUN_CHOICE_victim_finder:
        run_victim_finder(c, sens);
        break;
    case BLIND_RUN_CHOICE_pickup_artist:
        pa_step(&c->pickup_artist);
        break;
    case BLIND_RUN_CHOICE_path_finder:
        assert(!c->path_finder.no_path && !c->path_finder.path_completed);
        run_path_finder_executer(c, sens);
        break;
    default:
        /* Uhh */
        hal_print("Controller illegal state?!  Resetting ...");
        assert(0);
        controller_reset(c, sens);
        hal_set_speed(0, 0);
        break;
    }

    /* Internal map is updated "automatically" by the "echoed" UpdateMap
     * packets.  So we only send the proxmap here. */
    proximity_step(&c->prox_map, sens);
}

static unsigned int inquire_moderator_permission(Controller* c, Sensors* sens) {
    ModInputs inputs;
    inputs.t2t_data = &sens->t2t.moderate;
    inputs.own_victim = c->vic_finder.victim;
    inputs.found_victim_xy = c->vic_finder.found_victim_xy;
    inputs.give_up = c->pickup_artist.is_dead;

    mod_step(&inputs, &c->moderator);
    return c->moderator.may_run_p;
}

void inquire_new_vicdir_data(VFState* vf, Sensors* sens) {
    VFInputs inputs;
    T2TData_VicDirSingle* vd_buf = &sens->t2t.vicdir_buf1;

    if (vd_buf->new_p) {
        inputs.x = vd_buf->x;
        inputs.y = vd_buf->y;
        inputs.phi = vd_buf->phi;
        vf_apply(&inputs, vf);

        vd_buf = &sens->t2t.vicdir_buf2;
        if (vd_buf->new_p) {
            inputs.x = vd_buf->x;
            inputs.y = vd_buf->y;
            inputs.phi = vd_buf->phi;
            vf_apply(&inputs, vf);
        }
    }
}

static void inquire_blind_decision(Controller* c, Sensors* sens) {
    BlindInputs inputs;
    inputs.found_victim_xy = c->moderator.found_victim_xy;
    inputs.need_angle = c->cop_eyes.need_angle;
    inputs.no_path = c->path_finder.no_path;
    inputs.path_completed = c->path_finder.path_completed;
    inputs.victim_attached = sens->victim_attached;
    inputs.origin = c->origin;
    inputs.victim = c->moderator.victim;
    blind_step(&inputs, &c->blind);
}

static void inquire_eyes_decision(Controller* c, Sensors* sens) {
    TCEInputs inputs;

    irs_step(&c->ir_stab, sens);

    inputs.found_victim_phi = c->vic_dir.victim_found;
    inputs.found_victim_xy = c->moderator.found_victim_xy;
    inputs.ray_phi = c->vic_dir.victim_phi;
    inputs.ir_stable = c->ir_stab.ir_stable;
    inputs.phi_give_up = c->vic_dir.give_up;
    tce_step(&inputs, &c->cop_eyes, sens);
}

static void reset_appropriately(enum BlindRunChoice old_choice, Controller* c) {
    switch (old_choice) {
        case BLIND_RUN_CHOICE_none:
            /* Nothing to do here. */
            break;
        case BLIND_RUN_CHOICE_rhr:
            rhr_reset(&c->rhr);
            break;
        case BLIND_RUN_CHOICE_victim_finder:
            vd_reset(&c->vic_dir);
            break;
        case BLIND_RUN_CHOICE_pickup_artist:
            /* It will/should never run twice, but reset it anyways. */
            pa_reset(&c->pickup_artist);
            break;
        case BLIND_RUN_CHOICE_path_finder:
            pf_reset(&c->path_finder);
            pe_reset(&c->path_exec);
            break;
        default:
            /* Uhh, ignore that. */
            hal_print("Invalid previous state in Controller: Unknown blind cop choice.");
            break;
    }
}

static void run_path_finder_executer(Controller* c, Sensors* sens) {
    /* Q: Why don't we base the pf_input on pf's state?
     * A: Because that's unnecessary coupling.  Only read fields declared
     *    explicitly as output!  'state' is not an output. */
    PathFinderInputs pf_inputs;
    PathExecInputs pe_inputs;

    pf_inputs.compute = 1;
    pf_inputs.is_victim = c->blind.is_victim;
    pf_inputs.dest = c->blind.dst;
    pf_inputs.step_complete = c->path_exec.done;
    pf_inputs.step_see_obstacle = c->path_exec.see_obstacle;
    pf_step(&pf_inputs, &c->path_finder, sens);

    pe_inputs.drive = c->path_finder.drive;
    pe_inputs.backwards = c->path_finder.backwards;
    pe_inputs.next = c->path_finder.next;
    pe_step(&pe_inputs, &c->path_exec, sens);
}

static void run_victim_finder(Controller* c, Sensors* sens) {
    vd_step(&sens->t2t.fixdir, &c->vic_dir, sens);
    if (sens->t2t.fixdir.have_incoming_fix && c->vic_dir.victim_found) {
        VFInputs inputs;
        inputs.x = sens->current.x;
        inputs.y = sens->current.y;
        inputs.phi = c->vic_dir.victim_phi;
        vf_apply(&inputs, &c->vic_finder);
    }
}
