/**
 * @file main.c
 *
 * Application main module
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS.h"
#include "McRTOS_kernel_services.h"
#include "McRTOS_command_processor.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "frdm_board.h"
#include <networking.h>
#include <k64f_soc_enet.h>
#include <stdalign.h>

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#define MY_UDP_SERVER_PORT  8888

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    ACCELEROMETER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
};

static void app_hardware_init(void);
static void app_hardware_stop(void);
static void app_software_init(void);
static void demo_ping_command(void);
static void demo_udp_client_command(void);
static void demo_udp_server_command(void);
static void demo_ping_ipv6_command(void);
static void demo_udp_ipv6_client_command(void);
static void demo_udp_ipv6_server_command(void);

static fdc_error_t hello_world_thread_f(void *arg);
static fdc_error_t accelerometer_thread_f(void *arg);
static fdc_error_t demo_ping_thread_f(void *arg);
static fdc_error_t demo_udp_server_thread_f(void *arg);
static fdc_error_t demo_udp_client_thread_f(void *arg);
static fdc_error_t demo_ping_ipv6_thread_f(void *arg);
static fdc_error_t demo_udp_ipv6_server_thread_f(void *arg);
static fdc_error_t demo_udp_ipv6_client_thread_f(void *arg);

struct accelerometer_reading {
    /**
     * Latest X-axis acceleration reading
     */
    int16_t x_acceleration;

    /**
     * Latest Y-axis acceleration reading
     */
    int16_t y_acceleration;

    /**
     * Latest Z-axis acceleration reading
     */
    int16_t z_acceleration;
};

/**
 * Application global variables
 */
struct app_state_vars {
    alignas(MIN_MPU_REGION_ALIGNMENT) volatile uint32_t led_color_mask;
    struct accelerometer_reading accel_reading;
    struct rtos_mutex accel_reading_mutex;
    volatile bool accel_reading_ready;
    struct rtos_condvar accel_reading_condvar;
    bool demo_ping_started;
    bool demo_udp_client_started;
    bool demo_udp_server_started;
    bool demo_ping_ipv6_started;
    bool demo_udp_ipv6_client_started;
    bool demo_udp_ipv6_server_started;
    struct rtos_thread demo_ping_thread;
    struct rtos_thread demo_udp_client_thread;
    struct rtos_thread demo_udp_server_thread;
    struct rtos_thread demo_ping_ipv6_thread;
    struct rtos_thread demo_udp_ipv6_client_thread;
    struct rtos_thread demo_udp_ipv6_server_thread;
    struct ipv6_address remote_ipv6_addr_for_demo_ping6;
    struct ipv6_address remote_ipv6_addr_for_demo_client6;
};

C_ASSERT(sizeof(struct app_state_vars) % MIN_MPU_REGION_ALIGNMENT == 0);

/**
 * Demo UDP message
 */
struct accel_reading_msg {
    uint32_t seq_num;
    struct accelerometer_reading accel_reading;
};

static struct app_state_vars g_app = {
    .led_color_mask = LED_COLOR_RED,
    .accel_reading_ready = false,
};

#define RTOS_NUM_APP_THREADS 4

/**
 * Array of execution stacks for application threads
 */
static struct rtos_thread_execution_stack g_app_thread_execution_stacks[RTOS_NUM_APP_THREADS];

/**
 * Array of application threads
 */
struct rtos_thread g_app_threads[RTOS_NUM_APP_THREADS];

/**
 * Array of application thread creation parameters
 */
static const struct rtos_thread_creation_params g_app_thread_creation_params[] =
{
    [0] =
    {
	.p_name_p = "Hello World thread 1",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)1,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    },

    [1] =
    {
	.p_name_p = "Hello World thread 2",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)2,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    },

    [2] =
    {
        .p_name_p = "accelerometer thread",
        .p_function_p = accelerometer_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = ACCELEROMETER_THREAD_PRIORITY,
    },
};

C_ASSERT(ARRAY_SIZE(g_app_thread_creation_params) <=
         ARRAY_SIZE(g_app_thread_execution_stacks));

C_ASSERT(
    ARRAY_SIZE(g_app_thread_creation_params) <= RTOS_MAX_NUM_APP_THREADS);

/**
 * Array of application-specific console commands
 */
static const struct rtos_console_command g_app_console_commands[] =
{
    [0] =
    {
        .cmd_name_p = "demo-ping",
        .cmd_description_p = "Ping demo",
        .cmd_function_p = demo_ping_command,
    },

    [1] =
    {
        .cmd_name_p = "demo-client",
        .cmd_description_p = "UDP client demo",
        .cmd_function_p = demo_udp_client_command,
    },

    [2] =
    {
        .cmd_name_p = "demo-server",
        .cmd_description_p = "UDP server demo",
        .cmd_function_p = demo_udp_server_command,
    },

    [3] =
    {
        .cmd_name_p = "demo-ping6",
        .cmd_description_p = "Ping IPv6 demo",
        .cmd_function_p = demo_ping_ipv6_command,
    },

    [4] =
    {
        .cmd_name_p = "demo-client6",
        .cmd_description_p = "UDP IPv6 client demo",
        .cmd_function_p = demo_udp_ipv6_client_command,
    },

    [5] =
    {
        .cmd_name_p = "demo-server6",
        .cmd_description_p = "UDP IPV6 server demo",
        .cmd_function_p = demo_udp_ipv6_server_command,
    },
};


/**
 * McRTOS application startup configuration
 */
static const struct rtos_startup_app_configuration g_rtos_app_config =
{
    .stc_app_hardware_init_p = app_hardware_init,
    .stc_app_hardware_stop_p = app_hardware_stop,
    .stc_app_software_init_p = app_software_init,
    .stc_num_app_console_commands = ARRAY_SIZE(g_app_console_commands),
    .stc_app_console_commands_p = g_app_console_commands,
};


/**
 * Application's main()
 *
 * This function is invoked from the Reset exception handler for each CPU core.
 */
int
main(void)
{
    rtos_startup(&g_rtos_app_config);

    FDC_ASSERT(false, 0, 0);

    return -1;
}


static
void app_hardware_init(void)
{
    frdm_board_init();
    DEBUG_PRINTF("FRDM board initialized\n");
}


static
void app_hardware_stop(void)
{
    frdm_board_stop();
    DEBUG_PRINTF("FRDM board stopped\n");
}


static
void app_software_init(void)
{
    static const char g_app_version[] = "FRDM K64F board demo application v2.1";
    static const char g_app_build_timestamp[] = "built " __DATE__ " " __TIME__;

    fdc_error_t fdc_error;
    struct mpu_region_range old_comp_region;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    console_printf(
        "%s\n%s\n",
        g_app_version, g_app_build_timestamp);

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    rtos_mutex_init("Accelerometer reading mutex", &g_app.accel_reading_mutex);
    rtos_condvar_init("Accelerometer reading condvar", &g_app.accel_reading_condvar);

    /*
     * Create app threads for this CPU
     */
    for (uint8_t i = 0; i < ARRAY_SIZE(g_app_thread_creation_params); i ++)
    {
        rtos_thread_init(
            &g_app_thread_creation_params[i],
            &g_app_thread_execution_stacks[i],
            &g_app_threads[i]);

        console_printf("CPU core %u: %s started\n", cpu_id,
            g_app_thread_creation_params[i].p_name_p);
    }

    rtos_thread_restore_comp_region(&old_comp_region);

    networking_init(&g_enet_device0);
}


static bool
ping_remote_ip_addr(const struct ipv4_address *dest_ip_addr_p, uint16_t seq_num)
{
    fdc_error_t fdc_error;
    struct ipv4_address remote_ip_addr;
    uint16_t reply_seq_num;
    uint16_t reply_identifier;
    uint16_t identifier = (uintptr_t)rtos_thread_self();

    fdc_error = net_send_ipv4_ping_request(dest_ip_addr_p, identifier, seq_num);
    if (fdc_error != 0) {
        capture_fdc_msg_printf("net_send_ipv4_ping_request() failed with error %#x\n",
		               fdc_error);
        return false;
    }

    fdc_error = net_receive_ipv4_ping_reply(3000,
					    &remote_ip_addr,
					    &reply_identifier,
					    &reply_seq_num);

    if (fdc_error != 0) {
        CONSOLE_POS_PRINTF(39,1, "Ping %d for %u.%u.%u.%u timed-out",
                           seq_num,
                           dest_ip_addr_p->bytes[0],
                           dest_ip_addr_p->bytes[1],
                           dest_ip_addr_p->bytes[2],
                           dest_ip_addr_p->bytes[3]);
        return false;
    }

    FDC_ASSERT(remote_ip_addr.value == dest_ip_addr_p->value,
               remote_ip_addr.value, dest_ip_addr_p->value);
    FDC_ASSERT(reply_identifier == identifier,
               reply_identifier, identifier);
    FDC_ASSERT(reply_seq_num == seq_num,
               reply_seq_num, seq_num);

    CONSOLE_POS_PRINTF(38,1, "Ping %d replied by %u.%u.%u.%u",
                       reply_seq_num,
                       remote_ip_addr.bytes[0],
                       remote_ip_addr.bytes[1],
                       remote_ip_addr.bytes[2],
                       remote_ip_addr.bytes[3]);

    return true;
}


static struct rtos_thread_execution_stack demo_ping_thread_execution_stack;

static void
demo_ping_command(void)
{
    struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo ping thread",
        .p_function_p = demo_ping_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    };

    fdc_error_t fdc_error;
    struct mpu_region_range old_comp_region;
    struct ipv4_address local_ip_addr;
    struct ipv4_address remote_ip_addr;
    token_t token = get_next_token(&g_command_processor);

    if (token == HELP) {
	console_printf("\tdemo-ping <remote IPv4 address>\n\n");
        return;
    }

    if (token != DECIMAL_NUMBER) {
        goto syntax_error;
    }

    if (!parse_ip4_address(&remote_ip_addr)) {
        return;
    }

    net_get_local_ipv4_address(&local_ip_addr);
    if (local_ip_addr.value == IPV4_NULL_ADDR) {
        console_printf("Error: No local IP address defined\n");
        return;
    }

    if (local_ip_addr.value == remote_ip_addr.value) {
        console_printf("Error: Remote IP address must be different from local IP address\n");
        return;
    }

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    if (g_app.demo_ping_started) {
        console_printf("Ping demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_ping_started = true;

    /*
     * Create ping thread:
     */
    thread_params.p_function_arg_p = (void *)remote_ip_addr.value; /* pass IP add by value */
    rtos_thread_init(
        &thread_params,
        &demo_ping_thread_execution_stack,
        &g_app.demo_ping_thread);

    console_printf("%s started (destination IP address: %u.%u.%u.%u)\n",
                   thread_params.p_name_p,
                   remote_ip_addr.bytes[0],
                   remote_ip_addr.bytes[1],
                   remote_ip_addr.bytes[2],
                   remote_ip_addr.bytes[3]);

exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
    return;

syntax_error:
    console_printf("\'demo-ping\' command syntax error (type \'demo-ping help\')\n");
}


static struct rtos_thread_execution_stack demo_udp_client_thread_execution_stack;

static void
demo_udp_client_command(void)
{
    struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo UDP client thread",
        .p_function_p = demo_udp_client_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 3,
    };

    fdc_error_t fdc_error;
    struct mpu_region_range old_comp_region;
    struct ipv4_address local_ip_addr;
    struct ipv4_address remote_ip_addr;
    token_t token = get_next_token(&g_command_processor);

    if (token == HELP) {
	console_printf("\tdemo-client <remote IPv4 address>\n\n");
        return;
    }

    if (token != DECIMAL_NUMBER) {
        goto syntax_error;
    }

    if (!parse_ip4_address(&remote_ip_addr)) {
        return;
    }

    net_get_local_ipv4_address(&local_ip_addr);
    if (local_ip_addr.value == IPV4_NULL_ADDR) {
        console_printf("Error: No local IP address defined\n");
        return;
    }

    if (local_ip_addr.value == remote_ip_addr.value) {
        console_printf("Error: Remote IP address must be different from local IP address\n");
        return;
    }

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    if (g_app.demo_udp_client_started) {
        console_printf("UDP client demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_udp_client_started = true;

    /*
     * Create UDP client thread:
     */
    thread_params.p_function_arg_p = (void *)remote_ip_addr.value; /* pass IP add by value */
    rtos_thread_init(
        &thread_params,
        &demo_udp_client_thread_execution_stack,
        &g_app.demo_udp_client_thread);

    console_printf("%s started (server IP address: %u.%u.%u.%u)\n",
                   thread_params.p_name_p,
                   remote_ip_addr.bytes[0],
                   remote_ip_addr.bytes[1],
                   remote_ip_addr.bytes[2],
                   remote_ip_addr.bytes[3]);

exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
    return;

syntax_error:
	console_printf("\'demo-client\' command syntax error (type \'demo-client help\')\n");
}


static struct rtos_thread_execution_stack demo_udp_server_thread_execution_stack;

static void
demo_udp_server_command(void)
{
    static const struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo UDP server thread",
        .p_function_p = demo_udp_server_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 3,
    };

    struct mpu_region_range old_comp_region;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    if (g_app.demo_udp_server_started) {
        console_printf("UDP server demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_udp_server_started = true;

    /*
     * Create UDP server thread:
     */
    rtos_thread_init(
        &thread_params,
        &demo_udp_server_thread_execution_stack,
        &g_app.demo_udp_server_thread);

    console_printf("%s started\n", thread_params.p_name_p);

exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
}


static struct rtos_thread_execution_stack demo_ping_ipv6_thread_execution_stack;

static void
demo_ping_ipv6_command(void)
{
    struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo ping IPv6 thread",
        .p_function_p = demo_ping_ipv6_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    };

    fdc_error_t fdc_error;
    struct ipv6_address local_ipv6_addr;
    struct ipv6_address *remote_ipv6_addr_p;
    struct mpu_region_range old_comp_region;
    token_t token = get_next_token(&g_command_processor);

    if (token == HELP) {
	console_printf("\tdemo-ping6 <remote IPv6 address>\n\n");
        return;
    }

    if (token != HEXADECIMAL_NUMBER) {
        goto syntax_error;
    }

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    remote_ipv6_addr_p = &g_app.remote_ipv6_addr_for_demo_ping6;
    if (!parse_ip6_address(remote_ipv6_addr_p)) {
        return;
    }

    net_get_local_ipv6_address(&local_ipv6_addr);
    FDC_ASSERT(!IPV6_ADDRESSES_EQUAL(&local_ipv6_addr, &ipv6_unspecified_address),
	       0, 0);

    if (IPV6_ADDRESSES_EQUAL(&local_ipv6_addr, remote_ipv6_addr_p)) {
        console_printf("Error: Remote IPv6 address must be different from local IPv6 address\n");
        return;
    }

    if (g_app.demo_ping_ipv6_started) {
        console_printf("Ping IPv6 demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_ping_ipv6_started = true;

    /*
     * Create ping IPv6 thread:
     */
    thread_params.p_function_arg_p = remote_ipv6_addr_p;
    rtos_thread_init(
        &thread_params,
        &demo_ping_ipv6_thread_execution_stack,
        &g_app.demo_ping_ipv6_thread);

    console_printf("%s started (destination IPv6 address: %x:%x:%x:%x:%x:%x:%x:%x)\n",
                   thread_params.p_name_p,
                   remote_ipv6_addr_p->hwords[0],
                   remote_ipv6_addr_p->hwords[1],
                   remote_ipv6_addr_p->hwords[2],
                   remote_ipv6_addr_p->hwords[3],
                   remote_ipv6_addr_p->hwords[4],
                   remote_ipv6_addr_p->hwords[5],
                   remote_ipv6_addr_p->hwords[6],
                   remote_ipv6_addr_p->hwords[7]);

exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
    return;

syntax_error:
    console_printf("\'demo-ping6\' command syntax error (type \'demo-ping6 help\')\n");
}


static struct rtos_thread_execution_stack demo_udp_ipv6_client_thread_execution_stack;

static void
demo_udp_ipv6_client_command(void)
{
    struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo UDP IPv6 client thread",
        .p_function_p = demo_udp_ipv6_client_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 3,
    };

    fdc_error_t fdc_error;
    struct mpu_region_range old_comp_region;
    struct ipv6_address local_ipv6_addr;
    struct ipv6_address *remote_ipv6_addr_p;
    token_t token = get_next_token(&g_command_processor);

    if (token == HELP) {
	console_printf("\tdemo-client6 <remote IPv6 address>\n\n");
        return;
    }

    if (token != HEXADECIMAL_NUMBER) {
        goto syntax_error;
    }

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    remote_ipv6_addr_p = &g_app.remote_ipv6_addr_for_demo_client6;
    if (!parse_ip6_address(remote_ipv6_addr_p)) {
        goto exit_restore_region;
    }

    net_get_local_ipv6_address(&local_ipv6_addr);
    FDC_ASSERT(!IPV6_ADDRESSES_EQUAL(&local_ipv6_addr, &ipv6_unspecified_address),
	       0, 0);

    if (IPV6_ADDRESSES_EQUAL(&local_ipv6_addr, remote_ipv6_addr_p)) {
        console_printf("Error: Remote IPv6 address must be different from local IPv6 address\n");
        goto exit_restore_region;
    }

    if (g_app.demo_udp_ipv6_client_started) {
        console_printf("UDP IPv6 client demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_udp_ipv6_client_started = true;

    /*
     * Create UDP IPv6 client thread:
     */
    thread_params.p_function_arg_p = remote_ipv6_addr_p;
    rtos_thread_init(
        &thread_params,
        &demo_udp_ipv6_client_thread_execution_stack,
        &g_app.demo_udp_ipv6_client_thread);

    console_printf("%s started (server IPv6 address: %x:%x:%x:%x:%x:%x:%x:%x)\n",
                   thread_params.p_name_p,
                   remote_ipv6_addr_p->hwords[0],
                   remote_ipv6_addr_p->hwords[1],
                   remote_ipv6_addr_p->hwords[2],
                   remote_ipv6_addr_p->hwords[3],
                   remote_ipv6_addr_p->hwords[4],
                   remote_ipv6_addr_p->hwords[5],
                   remote_ipv6_addr_p->hwords[6],
                   remote_ipv6_addr_p->hwords[7]);


exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
    return;

syntax_error:
    console_printf("\'demo-client\' command syntax error (type \'demo-client help\')\n");
}


static struct rtos_thread_execution_stack demo_udp_ipv6_server_thread_execution_stack;

static void
demo_udp_ipv6_server_command(void)
{
    static const struct rtos_thread_creation_params thread_params = {
        .p_name_p = "Demo UDP IPv6 server thread",
        .p_function_p = demo_udp_ipv6_server_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 3,
    };

    struct mpu_region_range old_comp_region;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                &old_comp_region);

    if (g_app.demo_udp_ipv6_server_started) {
        console_printf("UDP server demo already started\n");
        goto exit_restore_region;
    }

    g_app.demo_udp_ipv6_server_started = true;

    /*
     * Create UDP IPv6 server thread:
     */
    rtos_thread_init(
        &thread_params,
        &demo_udp_ipv6_server_thread_execution_stack,
        &g_app.demo_udp_ipv6_server_thread);

    console_printf("%s started\n", thread_params.p_name_p);

exit_restore_region:
    rtos_thread_restore_comp_region(&old_comp_region);
}


static fdc_error_t
hello_world_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    bool fpu_enabled = false;

    FDC_ASSERT(arg != NULL, arg, cpu_id);

    int thread_id = (intptr_t)arg;

    if (thread_id == 1) {
	rtos_thread_enable_fpu();
	fpu_enabled = true;
    }

    float x = 0.1f;

    for ( ; ; ) {
	CONSOLE_POS_PRINTF(32, 60 + thread_id * 20, "Hello thread %1d", arg);
	rtos_thread_delay(500);
	CONSOLE_POS_PRINTF(32, 60 + thread_id * 20, "              ");
	if (thread_id == 1) {
	    x += 0.2f;
	    x -= 0.2f;
	    x *= 0.2f;
	    x /= 0.2f;
	}

	rtos_thread_delay(thread_id * 1000);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    if (fpu_enabled) {
	rtos_thread_disable_fpu();
    }

    return fdc_error;
}


static fdc_error_t
demo_ping_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);
    const struct ipv4_address dest_ip_addr = { .value = (uintptr_t)arg };
    uint32_t ping_count = 0;

    for ( ; ; ) {
	if (ping_remote_ip_addr(&dest_ip_addr, ping_count)) {
	    ping_count ++;
	}

	rtos_thread_delay(1500);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


/**
 * Accelerometer thread
 */
static fdc_error_t
accelerometer_thread_f(void *arg)
{
#   define ACCELEROMETER_SAMPLING_PERIOD_MS  50
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    bool accelerometer_started = false;

    FDC_ASSERT(arg == NULL, arg, cpu_id);
    console_printf("Initializing accelerometer sensing thread ...\n");

    accelerometer_init();
    accelerometer_started = true;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                NULL);

    g_app.accel_reading.x_acceleration = 0;
    g_app.accel_reading.y_acceleration = 0;
    g_app.accel_reading.z_acceleration = 0;

    for ( ; ; )
    {
#if 1
	int16_t x_acceleration;
	int16_t y_acceleration;
	int16_t z_acceleration;
	bool motion_detected;

        bool read_ok = accelerometer_read_status(
				&x_acceleration,
				&y_acceleration,
				&z_acceleration);

        if (read_ok) {
	    motion_detected = false;
	    if (g_app.accel_reading.x_acceleration != x_acceleration) {
	        g_app.accel_reading.x_acceleration = x_acceleration;
		motion_detected = true;
	    }

	    if (g_app.accel_reading.y_acceleration != y_acceleration) {
	        g_app.accel_reading.y_acceleration = y_acceleration;
		motion_detected = true;
	    }

	    if (g_app.accel_reading.z_acceleration != z_acceleration) {
	        g_app.accel_reading.z_acceleration = z_acceleration;
		motion_detected = true;
	    }

	    if (motion_detected) {
		CONSOLE_POS_PRINTF(41, 1, "Local accelerometer reading: x: %8d y: %8d z: %8d",
		    g_app.accel_reading.x_acceleration,
		    g_app.accel_reading.y_acceleration,
		    g_app.accel_reading.z_acceleration);

                rtos_mutex_acquire(&g_app.accel_reading_mutex);
                g_app.accel_reading_ready = true;
                rtos_mutex_release(&g_app.accel_reading_mutex);
                rtos_condvar_signal(&g_app.accel_reading_condvar);
	    }
        }
#else
        bool read_ok = accelerometer_detect_motion(
                        (int8_t *)&g_app.x_motion_detection,
                        (int8_t *)&g_app.y_motion_detection,
                        (int8_t *)&g_app.z_motion_detection);
        if (read_ok) {
#           if 0
            console_printf("x_motion: %d, y_motion: %d, z_motion: %d\n",
                g_app.x_motion_detection,
                g_app.y_motion_detection,
                g_app.z_motion_detection);
#           endif
        }
#endif

        rtos_thread_delay(ACCELEROMETER_SAMPLING_PERIOD_MS);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    if (accelerometer_started) {
	accelerometer_stop();
    }

    return fdc_error;
}


static fdc_error_t
demo_udp_server_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    uint32_t seq_num = 0;

    struct local_l4_end_point *server_end_point_p;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                NULL);

    fdc_error = net_create_local_l4_end_point(TRANSPORT_PROTO_UDP,
					      MY_UDP_SERVER_PORT,
					      &server_end_point_p);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(false);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;
	struct ipv4_address client_ip_addr;
	uint16_t client_port;
	struct accel_reading_msg *in_msg_p;
	uint32_t *out_msg_p;
	size_t in_msg_size;

	fdc_error = net_receive_ipv4_udp_datagram(server_end_point_p,
                                                  0,
                                                  &client_ip_addr,
                                                  &client_port,
                                                  &rx_packet_p);
        FDC_ASSERT(fdc_error == 0, fdc_error, 0);

        rtos_thread_set_tmp_region(rx_packet_p,
                                   sizeof *rx_packet_p,
                                   MPU_REGION_READ_ONLY);

	in_msg_size = net_get_udp_data_payload_length(rx_packet_p);
	FDC_ASSERT(in_msg_size == sizeof(struct accel_reading_msg), in_msg_size, 0);
	in_msg_p = net_get_udp_data_payload_area(rx_packet_p);
	seq_num = in_msg_p->seq_num;
        CONSOLE_POS_PRINTF(41, 81, "Remote accelerometer reading: x: %8d y: %8d z: %8d",
                           in_msg_p->accel_reading.x_acceleration,
                           in_msg_p->accel_reading.y_acceleration,
                           in_msg_p->accel_reading.z_acceleration);

	CONSOLE_POS_PRINTF(42, 81, "UDP server received message %10u", seq_num);

        rtos_thread_unset_tmp_region(); /* rx_packet_p */
	net_recycle_rx_packet(rx_packet_p);

        rtos_thread_set_tmp_region(tx_packet_p,
                                   sizeof *tx_packet_p,
                                   0);

	out_msg_p = net_get_udp_data_payload_area(tx_packet_p);
	*out_msg_p = seq_num;
        rtos_thread_unset_tmp_region(); /* tx_packet_p */

	fdc_error = net_send_ipv4_udp_datagram(server_end_point_p, &client_ip_addr,
                                               client_port,
                                               tx_packet_p, sizeof *out_msg_p);
        FDC_ASSERT(fdc_error == 0, fdc_error, tx_packet_p);

	CONSOLE_POS_PRINTF(43, 81, "UDP server sent ack for message %10u", seq_num);
    }

    net_free_tx_packet(tx_packet_p);

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


static fdc_error_t
demo_udp_client_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);
    const struct ipv4_address dest_ip_addr = { .value = (uintptr_t)arg };
    uint32_t seq_num = 0;

    struct local_l4_end_point *client_end_point_p;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                NULL);

    fdc_error = net_create_local_l4_end_point(TRANSPORT_PROTO_UDP, 0,
					      &client_end_point_p);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(false);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;
	struct ipv4_address server_ip_addr;
	uint16_t server_port;
	struct accel_reading_msg *out_msg_p;
	uint32_t *in_msg_p;
	size_t in_msg_size;

        rtos_thread_set_tmp_region(tx_packet_p,
                                   sizeof *tx_packet_p,
                                   0);

	out_msg_p = net_get_udp_data_payload_area(tx_packet_p);
	out_msg_p->seq_num = seq_num;

        rtos_mutex_acquire(&g_app.accel_reading_mutex);
        while (!g_app.accel_reading_ready) {
            rtos_condvar_wait(&g_app.accel_reading_condvar, &g_app.accel_reading_mutex, NULL);
        }

        out_msg_p->accel_reading = g_app.accel_reading;
        g_app.accel_reading_ready = false;
        rtos_mutex_release(&g_app.accel_reading_mutex);

        rtos_thread_unset_tmp_region(); /* tx_packet_p */

	fdc_error = net_send_ipv4_udp_datagram(client_end_point_p, &dest_ip_addr,
		                   MY_UDP_SERVER_PORT,
		                   tx_packet_p, sizeof *out_msg_p);

        if (fdc_error != 0) {
	    capture_fdc_msg_printf("net_send_ipv4_udp_datagram() failed with error %#x\n",
                                   fdc_error);
            break;
        }

	CONSOLE_POS_PRINTF(42, 1, "UDP client sent message %10u", seq_num);

	fdc_error = net_receive_ipv4_udp_datagram(client_end_point_p,
                                                  5000,
                                                  &server_ip_addr,
                                                  &server_port,
                                                  &rx_packet_p);
	if (fdc_error != 0) {
	    CONSOLE_POS_PRINTF(44, 1,
                               "UDP client receive timeout waiting for ack for message %10u",
                               seq_num);
	    continue;
	}

	FDC_ASSERT(server_ip_addr.value == dest_ip_addr.value,
	           server_ip_addr.value, dest_ip_addr.value);
	FDC_ASSERT(server_port == MY_UDP_SERVER_PORT,
		   server_port, 0);

        rtos_thread_set_tmp_region(rx_packet_p,
                                   sizeof *rx_packet_p,
                                   MPU_REGION_READ_ONLY);

	in_msg_size = net_get_udp_data_payload_length(rx_packet_p);
	FDC_ASSERT(in_msg_size == sizeof(uint32_t), in_msg_size, 0);
	in_msg_p = net_get_udp_data_payload_area(rx_packet_p);
	FDC_ASSERT(*in_msg_p == seq_num, *in_msg_p, seq_num);

	CONSOLE_POS_PRINTF(43, 1, "UDP client received ack for message %10u", *in_msg_p);

        rtos_thread_unset_tmp_region(); /* rx_packet_p */
	net_recycle_rx_packet(rx_packet_p);

	seq_num ++;
    }

    net_free_tx_packet(tx_packet_p);
    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


static bool
ping_remote_ipv6_addr(const struct ipv6_address *dest_ipv6_addr_p, uint16_t seq_num)
{
    fdc_error_t fdc_error;
    struct ipv6_address remote_ipv6_addr;
    uint16_t reply_seq_num;
    uint16_t reply_identifier;
    uint16_t identifier = (uintptr_t)rtos_thread_self();

    fdc_error = net_send_ipv6_ping_request(dest_ipv6_addr_p, identifier, seq_num);
    if (fdc_error != 0) {
        capture_fdc_msg_printf("net_send_ipv6_ping_request() failed with error %#x\n",
		               fdc_error);
        return false;
    }

    fdc_error = net_receive_ipv6_ping_reply(3000,
					    &remote_ipv6_addr,
					    &reply_identifier,
					    &reply_seq_num);

    if (fdc_error != 0) {
        CONSOLE_POS_PRINTF(39,20, "Ping IPv6 %d for %x:%x:%x:%x:%x:%x:%x:%x timed-out",
                           seq_num,
                           dest_ipv6_addr_p->hwords[0],
                           dest_ipv6_addr_p->hwords[1],
                           dest_ipv6_addr_p->hwords[2],
                           dest_ipv6_addr_p->hwords[3],
                           dest_ipv6_addr_p->hwords[4],
                           dest_ipv6_addr_p->hwords[5],
                           dest_ipv6_addr_p->hwords[6],
                           dest_ipv6_addr_p->hwords[7]);
        return false;
    }

    FDC_ASSERT(IPV6_ADDRESSES_EQUAL(&remote_ipv6_addr, dest_ipv6_addr_p),
               remote_ipv6_addr.words[2], remote_ipv6_addr.words[3]);
    FDC_ASSERT(reply_identifier == identifier,
               reply_identifier, identifier);
    FDC_ASSERT(reply_seq_num == seq_num,
               reply_seq_num, seq_num);

    CONSOLE_POS_PRINTF(38, 20, "Ping IPv6 %d replied by %x:%x:%x:%x:%x:%x:%x:%x",
                       reply_seq_num,
                       remote_ipv6_addr.hwords[0],
                       remote_ipv6_addr.hwords[1],
                       remote_ipv6_addr.hwords[2],
                       remote_ipv6_addr.hwords[3],
                       remote_ipv6_addr.hwords[4],
                       remote_ipv6_addr.hwords[5],
                       remote_ipv6_addr.hwords[6],
                       remote_ipv6_addr.hwords[7]);

    return true;
}


static fdc_error_t
demo_ping_ipv6_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);
    const struct ipv6_address *dest_ipv6_addr_p = (struct ipv6_address *)arg;
    uint32_t ping_count = 0;

    for ( ; ; ) {
	if (ping_remote_ipv6_addr(dest_ipv6_addr_p, ping_count)) {
	    ping_count ++;
	}

	rtos_thread_delay(1500);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


static fdc_error_t
demo_udp_ipv6_server_thread_f(void *arg)
{
    FDC_ASSERT(arg == NULL, arg, 0);

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    uint32_t seq_num = 0;

    struct local_l4_end_point *server_end_point_p;

    fdc_error = net_create_local_l4_end_point(TRANSPORT_PROTO_UDP,
					      MY_UDP_SERVER_PORT,
					      &server_end_point_p);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(false);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;
	struct ipv6_address client_ipv6_addr;
	uint16_t client_port;
	struct accel_reading_msg *in_msg_p;
	uint32_t *out_msg_p;
	size_t in_msg_size;

	fdc_error = net_receive_ipv6_udp_datagram(server_end_point_p,
                                                  0,
                                                  &client_ipv6_addr,
                                                  &client_port,
                                                  &rx_packet_p);
        FDC_ASSERT(fdc_error == 0, fdc_error, 0);

        rtos_thread_set_tmp_region(rx_packet_p,
                                   sizeof *rx_packet_p,
                                   MPU_REGION_READ_ONLY);

	in_msg_size = net_get_udp_data_payload_length(rx_packet_p);
	FDC_ASSERT(in_msg_size == sizeof(struct accel_reading_msg), in_msg_size, 0);
	in_msg_p = net_get_udp_data_payload_area(rx_packet_p);
	seq_num = in_msg_p->seq_num;
        CONSOLE_POS_PRINTF(44, 81, "Remote accelerometer reading (IPv6): x: %8d y: %8d z: %8d",
                           in_msg_p->accel_reading.x_acceleration,
                           in_msg_p->accel_reading.y_acceleration,
                           in_msg_p->accel_reading.z_acceleration);

	CONSOLE_POS_PRINTF(45, 81, "UDP IPv6 server received message %10u", seq_num);

        rtos_thread_unset_tmp_region(); /* rx_packet_p */
	net_recycle_rx_packet(rx_packet_p);

        rtos_thread_set_tmp_region(tx_packet_p,
                                   sizeof *tx_packet_p,
                                   0);

	out_msg_p = net_get_udp_data_payload_area(tx_packet_p);
	*out_msg_p = seq_num;
        rtos_thread_unset_tmp_region(); /* tx_packet_p */

	fdc_error = net_send_ipv6_udp_datagram(server_end_point_p, &client_ipv6_addr,
                                               client_port,
                                               tx_packet_p, sizeof *out_msg_p);
        FDC_ASSERT(fdc_error == 0, fdc_error, tx_packet_p);

	CONSOLE_POS_PRINTF(43, 81, "UDP server sent ack for message %10u", seq_num);
    }

    net_free_tx_packet(tx_packet_p);

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


static fdc_error_t
demo_udp_ipv6_client_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);
    const struct ipv6_address *dest_ipv6_addr_p = (const struct ipv6_address *)arg;
    uint32_t seq_num = 0;

    struct local_l4_end_point *client_end_point_p;

    rtos_thread_set_comp_region(&g_app,
                                sizeof g_app,
                                0,
                                NULL);

    fdc_error = net_create_local_l4_end_point(TRANSPORT_PROTO_UDP, 0,
					      &client_end_point_p);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(false);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;
	struct ipv6_address server_ipv6_addr;
	uint16_t server_port;
	struct accel_reading_msg *out_msg_p;
	uint32_t *in_msg_p;
	size_t in_msg_size;

        rtos_thread_set_tmp_region(tx_packet_p,
                                   sizeof *tx_packet_p,
                                   0);

	out_msg_p = net_get_udp_data_payload_area(tx_packet_p);
	out_msg_p->seq_num = seq_num;

        rtos_mutex_acquire(&g_app.accel_reading_mutex);
        while (!g_app.accel_reading_ready) {
            rtos_condvar_wait(&g_app.accel_reading_condvar, &g_app.accel_reading_mutex, NULL);
        }

        out_msg_p->accel_reading = g_app.accel_reading;
        g_app.accel_reading_ready = false;
        rtos_mutex_release(&g_app.accel_reading_mutex);

        rtos_thread_unset_tmp_region(); /* tx_packet_p */

	fdc_error = net_send_ipv6_udp_datagram(client_end_point_p, dest_ipv6_addr_p,
		                   MY_UDP_SERVER_PORT,
		                   tx_packet_p, sizeof *out_msg_p);

        if (fdc_error != 0) {
	    capture_fdc_msg_printf("net_send_ipv6_udp_datagram() failed with error %#x\n",
                                   fdc_error);
            break;
        }

	CONSOLE_POS_PRINTF(45, 1, "UDP IPv6 client sent message %10u", seq_num);

	fdc_error = net_receive_ipv6_udp_datagram(client_end_point_p,
                                                  5000,
                                                  &server_ipv6_addr,
                                                  &server_port,
                                                  &rx_packet_p);
	if (fdc_error != 0) {
	    CONSOLE_POS_PRINTF(47, 1,
                               "UDP IPv6 client receive timeout waiting for ack for message %10u",
                               seq_num);
	    continue;
	}

	FDC_ASSERT(IPV6_ADDRESSES_EQUAL(&server_ipv6_addr, dest_ipv6_addr_p),
	           server_ipv6_addr.words[2], server_ipv6_addr.words[3]);
	FDC_ASSERT(server_port == MY_UDP_SERVER_PORT,
		   server_port, 0);

        rtos_thread_set_tmp_region(rx_packet_p,
                                   sizeof *rx_packet_p,
                                   MPU_REGION_READ_ONLY);

	in_msg_size = net_get_udp_data_payload_length(rx_packet_p);
	FDC_ASSERT(in_msg_size == sizeof(uint32_t), in_msg_size, 0);
	in_msg_p = net_get_udp_data_payload_area(rx_packet_p);
	FDC_ASSERT(*in_msg_p == seq_num, *in_msg_p, seq_num);

	CONSOLE_POS_PRINTF(46, 1, "UDP IPv6 client received ack for message %10u", *in_msg_p);

        rtos_thread_unset_tmp_region(); /* rx_packet_p */
	net_recycle_rx_packet(rx_packet_p);

	seq_num ++;
    }

    net_free_tx_packet(tx_packet_p);
    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

