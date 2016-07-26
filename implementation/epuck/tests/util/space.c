#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Need correct measurements from map_static and BellmanFord. */
#undef MAP_MAX_WIDTH
#define MAP_MAX_WIDTH 100
#undef MAP_MAX_HEIGHT
#define MAP_MAX_HEIGHT 100

#include "../../../../libraries/tinpuck/include/tinpuck/com.h"
#include "../../../../libraries/tinpuck/include/tinpuck/scheduler.h"
#include "tinbot.h"

typedef struct space_entry {
    size_t size;
    const char* pos;
    const char* decl;
} space_entry;

static space_entry space_entries[1000];
static size_t num_entries = 0;

static int compar(const void* p1, const void* p2) {
    const space_entry* se1 = (const space_entry*)p1;
    const space_entry* se2 = (const space_entry*)p2;
    if (se1->size != se2->size) {
        return (se1->size < se2->size) ? 1 : -1;
    }
    return strcmp(se1->pos, se2->pos);
}

#define USE_SPACE(POS,TYPE,NAME,ARRISH,DISPARRISH) do { \
        space_entries[num_entries].size = sizeof(TYPE ARRISH); \
        space_entries[num_entries].pos = #POS; \
        space_entries[num_entries].decl = #TYPE " " #NAME DISPARRISH; \
        ++num_entries; \
    } while (0)


static void collect_all() {

/*

The following is a *SINGLE* shell command poduces the following C code:

find \( -name build -o -name tests -o -name map-optimize -o -name map_heap.c -o -name hal_matlab.c \) -prune \
  -o -name '*.c' -print0 | \
sort -z | \
xargs -r0 grep -Prn 'static[^[(]*(\[.*)?$' | \
perl -pnl -e 's/^([^:]*:\d+):\W*static\W+(?:struct|volatile|const|unsigned|\W+)*(\w*)\W+([^\[; ]*)((?:\[[^\]]+\])*)([^\n]*)$/    USE_SPACE(\1,\2,\3~\4~\5/g' | \
perl -pnl -e 's%~(\[.*\])~(.*)$%,\1,"\1\2");%g' | \
perl -pnl -e 's%~~(.*)$%,[1],"\1");%g'

*/

    /* ------8<------8<------8<------8<------ */
    USE_SPACE(./hal/hal_epuck.c:19,hal_epuck_motor_wrapper,motor_wrapper,[1],";");
    USE_SPACE(./hal/hal_epuck.c:60,TinPackage,send_buf,[1]," = {0, 0, 0, 0, NULL, NULL, 0, NULL};");
    USE_SPACE(./hal/hal_epuck.c:61,int,send_buf_sending,[1]," = 0;");
    USE_SPACE(./hal/hal_epuck.c:82,char,send_buf_buf,[TIN_PACKAGE_MAX_LENGTH],"[TIN_PACKAGE_MAX_LENGTH];");
    USE_SPACE(./hal/hal_epuck.c:108,TinPackage,package,[1]," = {0, 0, CMD_PRINT, 0, NULL, NULL, 0, NULL};");
    USE_SPACE(./hal/map_static.c:10,char,map_accu,[MAP_INTERNAL_DATA_SIZE(MAP_MAX_WIDTH,MAP_MAX_HEIGHT)],"[MAP_INTERNAL_DATA_SIZE(MAP_MAX_WIDTH,MAP_MAX_HEIGHT)]");
    USE_SPACE(./hal/map_static.c:12,char,map_prox,[MAP_INTERNAL_DATA_SIZE(MAP_PROXIMITY_SIZE,MAP_PROXIMITY_SIZE)],"[MAP_INTERNAL_DATA_SIZE(MAP_PROXIMITY_SIZE,MAP_PROXIMITY_SIZE)]");
    USE_SPACE(./hal/map_static.c:49,char,printbuf,[TIN_PACKAGE_MAX_LENGTH],"[TIN_PACKAGE_MAX_LENGTH];");
    USE_SPACE(./main.c:15,int,ir_data,[8],"[8] = {0};");
    USE_SPACE(./main.c:17,double,lps_data,[3],"[3] = {0};");
    USE_SPACE(./main.c:18,int,lps_updated,[1]," = 0;");
    USE_SPACE(./main.c:20,int,pickup_data,[1],";");
    USE_SPACE(./main.c:22,int,state,[1]," = STATE_STARTUP;");
    USE_SPACE(./main.c:24,int,loop_counter,[1]," = 0;");
    USE_SPACE(./main.c:25,int,loop_freq,[1]," = 0;");
    USE_SPACE(./main.c:27,int,proximity_raw,[11][8],"[11][8] = {0};");
    USE_SPACE(./main.c:28,int,proximity_position,[1]," = 0;");
    USE_SPACE(./main.c:30,double,proximity,[8],"[8] = {0};");
    USE_SPACE(./main.c:32,int,do_reset,[1]," = 0;");
    USE_SPACE(./main.c:33,int,do_calibration,[1]," = 0;");
    USE_SPACE(./main.c:35,TinBot,bot,[1],";");
    USE_SPACE(./main.c:37,char,my_com_addr,[1],";");
    USE_SPACE(./main.c:40,char,mode,[1]," = 0;");
    USE_SPACE(./main.c:44,TinTask,update_ext_data_task,[1],";");
    USE_SPACE(./main.c:52,TinTask,proximity_filter_task,[1],";");
    USE_SPACE(./main.c:63,TinTask,reset_loop_counter_task,[1],";");
    USE_SPACE(./main.c:72,TinTask,send_heartbeat_task,[1],";");
    USE_SPACE(./main.c:90,TinPackage,response,[1]," = {NULL, NULL, CMD_HELLO, 0, NULL, NULL};");
    USE_SPACE(./main.c:91,char,data,[4 + 2 + 2],"[4 + 2 + 2] __attribute__ ((aligned (4)));");
    USE_SPACE(./main.c:128,TinPackage,response,[1]," = {NULL, NULL, CMD_DEBUG_INFO, 0, NULL, NULL};");
    USE_SPACE(./main.c:129,char,data,[4 * 11 + 2 + 6 + 1],"[4 * 11 + 2 + 6 + 1] __attribute__ ((aligned (4)));");
    USE_SPACE(./main.c:163,TinPackage,response,[1]," = {NULL, NULL, CMD_T2T_UPDATE_MAP, 0, NULL, NULL};");
    USE_SPACE(./main.c:164,char,data,[68],"[68] __attribute__ ((aligned (4)));");
    USE_SPACE(./main.c:345,int,tmp_pickup_data,[1]," = 0;");
    USE_SPACE(./main.c:346,int,state,[1]," = 0;");
    USE_SPACE(./src/approximator.c:14,double,tinbot_diameter,[1]," = 5.3;");
    USE_SPACE(./src/approximator.c:19,int,status,[1]," = 0;");
    USE_SPACE(./src/blind-cop.c:10,double,NO_PATH_TIMEOUT_SECS,[1]," = 40;");
    USE_SPACE(./src/controller.c:34,int,first_iter,[1]," = 1;");
    USE_SPACE(./src/path-exec.c:36,double,PE_MAX_STRAY,[1]," = 10;");
    USE_SPACE(./src/path-finder.c:148,int,APPROX_RADIUS,[1]," = 2;");
    USE_SPACE(./src/rhr.c:27,double,RHR_CONF_CORNER_D,[1]," = 9.5;");
    USE_SPACE(./src/rhr.c:28,double,RHR_CONF_CORNER_X,[1]," = 11;");
    USE_SPACE(./src/rhr.c:29,double,RHR_CONF_WALL_THRESH,[1]," = 2;");
    USE_SPACE(./src/rhr.c:30,double,RHR_CONF_WALL_D,[1]," = 1;");
    USE_SPACE(./src/rhr.c:31,double,RHR_CONF_STROKE_THRESH,[1]," = 1.8;");
    USE_SPACE(./src/rhr.c:41,int,order,[NUM_PROXIMITY],"[NUM_PROXIMITY] =");
    USE_SPACE(./src/t2t.c:38,char,mybuf,[100],"[100];");
    USE_SPACE(./src/tinbot.c:113,long,iterations,[1]," = 10000;");
    USE_SPACE(./src/tinbot.c:129,int,pathfin_origin_resetted,[1]," = 0;");
    USE_SPACE(./src/tinbot.c:176,TinMode,modes,[7],"[7] = {");
    USE_SPACE(./src/traffic-cop-eyes.c:14,double,MIN_DIST,[1]," = 12;");
    USE_SPACE(./src/victim-direction.c:26,double,VD_MIN_ON,[1]," = 9.0 / 360.0;");
    /* ------>8------>8------>8------>8------ */
}

static void collect_tinbot() {
/*

The following is a *SINGLE* shell command poduces the following C code:

echo '<SECTION>' | \
perl -pnl -e 's/^\W*(?:static|struct|volatile|const|unsigned|\W+)*(\w*)\W+([^\[; ]*)((?:\[[^\]]+\])*)([^\n]*)$/    USE_SPACE(tinbot.h:9-12,\1,\2~\3~\4/g' | \
perl -pnl -e 's%~(\[.*\])~(.*)$%,\1,"\1\2");%g' | \
perl -pnl -e 's%~~(.*)$%,[1],"\1");%g'

*/

    /* ------8<------8<------8<------8<------ */
    USE_SPACE(tinbot.h:9-12,Sensors,sens,[1],";");
    USE_SPACE(tinbot.h:9-12,Controller,controller,[1],";");
    USE_SPACE(tinbot.h:9-12,T2TData,rx_buffer,[1],";");
    USE_SPACE(tinbot.h:9-12,int,mode,[1],";");
    /* ------>8------>8------>8------>8------ */
}

static void collect_controller() {
/*

The following is a *SINGLE* shell command poduces the following C code:

echo '<SECTION>' | \
perl -pnl -e 's/^\W*(?:static|struct|volatile|const|unsigned|\W+)*(\w*)\W+([^\[; ]*)((?:\[[^\]]+\])*)([^\n]*)$/    USE_SPACE(controller.h:19-30,\1,\2~\3~\4/g' | \
perl -pnl -e 's%~(\[.*\])~(.*)$%,\1,"\1\2");%g' | \
perl -pnl -e 's%~~(.*)$%,[1],"\1");%g'

*/

    /* ------8<------8<------8<------8<------ */
    USE_SPACE(controller.h:19-30,ApproxState,approx,[1],";");
    USE_SPACE(controller.h:19-30,BlindState,blind,[1],";");
    USE_SPACE(controller.h:19-30,ModState,moderator,[1],";");
    USE_SPACE(controller.h:19-30,PathExecState,path_exec,[1],";");
    USE_SPACE(controller.h:19-30,PathFinderState,path_finder,[1],";");
    USE_SPACE(controller.h:19-30,PickupState,pickup_artist,[1],";");
    USE_SPACE(controller.h:19-30,ProxMapState,prox_map,[1],";");
    USE_SPACE(controller.h:19-30,RhrState,rhr,[1],";");
    USE_SPACE(controller.h:19-30,IRSState,ir_stab,[1],";");
    USE_SPACE(controller.h:19-30,TCEState,cop_eyes,[1],";");
    USE_SPACE(controller.h:19-30,VDState,vic_dir,[1],";");
    USE_SPACE(controller.h:19-30,VFState,vic_finder,[1],";");
    USE_SPACE(controller.h:19-30,ExactPosition,origin,[1],";");
    /* ------>8------>8------>8------>8------ */
}

static void collect_bellman_ford() {
/*

The following is a *SINGLE* shell command poduces the following C code:

echo '<SECTION>' | \
perl -pnl -e 's/^\W*(?:static|struct|volatile|const|unsigned|\W+)*(\w*)\W+([^\[; ]*)((?:\[[^\]]+\])*)([^\n]*)$/    USE_SPACE(bellman-ford.h:14-27,\1,\2~\3~\4/g' | \
perl -pnl -e 's%~(\[.*\])~(.*)$%,\1,"\1\2");%g' | \
perl -pnl -e 's%~~(.*)$%,[1],"\1");%g'

(Needs some manual editing due to comments.)

*/

    /* ------8<------8<------8<------8<------ */
    USE_SPACE(bellman-ford.h:14-27,ExactPosition,init,[1],";");
    USE_SPACE(bellman-ford.h:14-27,ExactPosition,goal,[1],";");
    USE_SPACE(bellman-ford.h:14-27,int16_t,distances,[NUM_VERTICES],"[NUM_VERTICES];");
    USE_SPACE(bellman-ford.h:14-27,int16_t,init_v,[1],";");
    USE_SPACE(bellman-ford.h:14-27,int16_t,goal_v,[1],";");
    USE_SPACE(bellman-ford.h:14-27,int16_t,succ,[NUM_VERTICES],"[NUM_VERTICES];");
    /* ------>8------>8------>8------>8------ */
}

static void collect_tinpuck() {

/*

The following is a *SINGLE* shell command poduces the following C code:

find -name build -prune -o -name '*.c' -print0 | \
sort -z | \
xargs -r0 grep -Prn 'static[^[(]*(\[.*)?$' | \
perl -pnl -e 's%^\.%tinpuck%g' | \
perl -pnl -e 's/^([^:]*:\d+):\W*static\W+(?:struct|volatile|const|unsigned|\W+)*(\w*)\W+([^\[; ]*)((?:\[[^\]]+\])*)([^\n]*)$/    USE_SPACE(\1,\2,\3~\4~\5/g' | \
perl -pnl -e 's%~(\[.*\])~(.*)$%,\1,"\1\2");%g' | \
perl -pnl -e 's%~~(.*)$%,[1],"\1");%g'

*/

    /* ------8<------8<------8<------8<------ */
    USE_SPACE(tinpuck/src/adc.c:48,int,tin_proximity_ambient,[8],"[8];");
    USE_SPACE(tinpuck/src/adc.c:49,int,tin_proximity_reflected,[8],"[8];");
    USE_SPACE(tinpuck/src/adc.c:50,int,tin_proximity_offset,[8],"[8] = {0, 0, 0, 0, 0, 0, 0, 0};");
    USE_SPACE(tinpuck/src/adc.c:52,int,tin_proximity_counter,[1]," = 0;");
    USE_SPACE(tinpuck/src/adc.c:128,int,counter,[1]," = 0;");
    USE_SPACE(tinpuck/src/adc.c:129,int,sensors,[1]," = 0;");
    USE_SPACE(tinpuck/src/com.c:27,TinPackage,tx_queue,[1]," = NULL;");
    USE_SPACE(tinpuck/src/com.c:28,char,tx_address,[1]," = 0;");
    USE_SPACE(tinpuck/src/com.c:29,int,tx_state,[1]," = STATE_SOURCE;");
    USE_SPACE(tinpuck/src/com.c:30,int,tx_position,[1]," = 0;");
    USE_SPACE(tinpuck/src/com.c:32,TinPackage,rx_package,[1],";");
    USE_SPACE(tinpuck/src/com.c:47,char,rx_data,[TIN_PACKAGE_MAX_LENGTH],"[TIN_PACKAGE_MAX_LENGTH]");
    USE_SPACE(tinpuck/src/com.c:50,char,rx_state,[1]," = STATE_SOURCE;");
    USE_SPACE(tinpuck/src/com.c:51,int,rx_position,[1]," = 0;");
    USE_SPACE(tinpuck/src/com.c:52,long,rx_start,[1]," = 0;");
    USE_SPACE(tinpuck/src/com.c:53,TinPackageCallback,rx_callbacks,[255],"[255] = {NULL};");
    USE_SPACE(tinpuck/src/com.c:55,TinTask,timeout_task,[1],";");
    USE_SPACE(tinpuck/src/motors.c:39,int,tin_left_motor_phase,[1]," = 0;");
    USE_SPACE(tinpuck/src/motors.c:40,int,tin_right_motor_phase,[1]," = 0;");
    USE_SPACE(tinpuck/src/motors.c:42,int,tin_motors_powersave,[1]," = 1;");
    USE_SPACE(tinpuck/src/motors.c:44,int,tin_left_speed,[1]," = 0;");
    USE_SPACE(tinpuck/src/motors.c:45,int,tin_right_speed,[1]," = 0;");
    USE_SPACE(tinpuck/src/motors.c:47,TinTask,tin_motor_left_task,[1],";");
    USE_SPACE(tinpuck/src/motors.c:48,TinTask,tin_motor_right_task,[1],";");
    USE_SPACE(tinpuck/src/motors.c:208,int,on,[1]," = 0;");
    USE_SPACE(tinpuck/src/motors.c:233,int,on,[1]," = 0;");
    USE_SPACE(tinpuck/src/scheduler.c:19,TinTask,tin_task_list_head,[1]," = NULL;");
    USE_SPACE(tinpuck/src/time.c:19,TinTask,tin_time_task,[1],";");
    /* ------>8------>8------>8------>8------ */
}

static void stat_it(const char* name, size_t top) {
    size_t total = 0;
    size_t i, top_total;

    qsort(space_entries, num_entries, sizeof(space_entry), compar);

    for (i = 0; i < num_entries; ++i) {
        total += space_entries[i].size;
    }
    printf("--- stats for %s ---\n"
        "All seen static declarations take up a total of %lu bytes on x86.\n",
        name,
        total);
    top_total = 0;
    top = (top < num_entries) ? top : num_entries;
    for (i = 0; i < top; ++i) {
        top_total += space_entries[i].size;
        printf("%5lu (%5.2f%%) in %-40s %s\n",
            space_entries[i].size,
            space_entries[i].size * 100.0 / total,
            space_entries[i].pos,
            space_entries[i].decl);
    }
    if (top < num_entries) {
        printf("These top %lu entries take up %lu bytes (%.2f%%) on x86.\n",
            top,
            top_total,
            top_total * 100.0 / total);
    }
    printf("--- end stats %s ---\n", name);
    num_entries = 0;
}

int main() {
    printf("===== BEGIN STATISTICS =====\n");

    collect_all();
    stat_it("<ALL>", 5);

    collect_tinpuck();
    stat_it("../../libraries/tinpuck/", 4);

    collect_tinbot();
    stat_it("tinbot.h", 5);

    collect_controller();
    stat_it("controller.h", 3);

    collect_bellman_ford();
    stat_it("bellman_ford.h", 3);

    printf("===== END STATISTICS =====\n");
    return 0;
}
