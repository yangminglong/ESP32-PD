

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_partition.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"
#include "cmd_wifi.h"
#include "cmd_nvs.h"
#include "argtable3/argtable3.h"

#include "pd.h"

#define PROMPT_STR "usb_pd"
#define TAG "cmd"
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH 64

#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

esp_console_repl_t *repl = NULL;
esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

static struct
{
    struct arg_str *object;
    struct arg_str *voltage_mv;
    struct arg_str *current_ma;
    struct arg_end *end;
} cmd_req_pps_args;

static struct
{
    struct arg_str *object;
    struct arg_str *current_ma;
    struct arg_end *end;
} cmd_req_obj_args;

static int cmd_req_pps(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&cmd_req_pps_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, cmd_req_pps_args.end, argv[0]);
        return 1;
    }

    const char *object = cmd_req_pps_args.object->sval[0];
    const char *voltage_mv = cmd_req_pps_args.voltage_mv->sval[0];
    const char *current_ma = cmd_req_pps_args.current_ma->sval[0];

    uint16_t arg_obj = atoi(object);
    uint16_t arg_mv = atoi(voltage_mv);
    uint16_t arg_ma = atoi(current_ma);

    pd_request_pps(arg_obj, arg_mv, arg_ma, 0);

    return 0;
}

static int cmd_req_obj(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&cmd_req_obj_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, cmd_req_obj_args.end, argv[0]);
        return 1;
    }

    const char *object = cmd_req_obj_args.object->sval[0];
    const char *current_ma = cmd_req_obj_args.current_ma->sval[0];

    uint16_t arg_obj = atoi(object);
    uint16_t arg_ma = atoi(current_ma);

    pd_request(arg_obj, arg_ma, 0);

    return 0;
}

static int cmd_get_src_cap(int argc, char **argv)
{
    pd_send_control(PD_CONTROL_GET_SOURCE_CAP);

    return 0;
}



static int list_partitions(int argc, char **argv)
{
    /* Get the iterator for all partitions */
    const esp_partition_t *partition;
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);

    if (it == NULL)
    {
        printf("No partitions found.\n");
        return ESP_OK;
    }

    printf("Partitions:\n");
    printf("------------------------------------------------------------\n");
    printf("| Type        | Subtype     | Address    | Size    | Label |\n");
    printf("------------------------------------------------------------\n");

    /* Iterate through all partitions */
    while (it != NULL)
    {
        partition = esp_partition_get(it);
        printf("| %-11s | %-11s | 0x%08" PRIx32 " | %-7" PRId32 " | %-5s\n",
               partition->type == ESP_PARTITION_TYPE_APP ? "app" : "data",
               partition->subtype == ESP_PARTITION_SUBTYPE_ANY ? "any" : "specific",
               partition->address,
               partition->size,
               partition->label);

        it = esp_partition_next(it);
    }

    printf("------------------------------------------------------------\n");

    /* Free the iterator */
    esp_partition_iterator_release(it);
    return ESP_OK;
}

const esp_console_cmd_t req_get_src_cap_cmd = {
    .command = "get_src_cap",
    .help = "Request source capabilities report."
            "t.b.d\n",
    .hint = NULL,
    .func = &cmd_get_src_cap,
    .argtable = NULL};
    
const esp_console_cmd_t req_pps_cmd = {
    .command = "req_pps",
    .help = "Request a PPS mode."
            "t.b.d\n",
    .hint = NULL,
    .func = &cmd_req_pps,
    .argtable = &cmd_req_pps_args};

const esp_console_cmd_t req_obj_cmd = {
    .command = "req_obj",
    .help = "Request a normal mode."
            "t.b.d\n",
    .hint = NULL,
    .func = &cmd_req_obj,
    .argtable = &cmd_req_obj_args};

const esp_console_cmd_t cmd_list_partitions = {
    .command = "list_partitions",
    .help = "List all partitions in the device",
    .hint = NULL,
    .func = &list_partitions,
};


void cmd_init()
{
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    /* Register commands */
    esp_console_register_help_command();
    register_system_common();
#if SOC_LIGHT_SLEEP_SUPPORTED
    register_system_light_sleep();
#endif
#if SOC_DEEP_SLEEP_SUPPORTED
    register_system_deep_sleep();
#endif
#if (CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_HOST_WIFI_ENABLED)
    register_wifi();
#endif
    register_nvs();

    cmd_req_pps_args.object = arg_str1(NULL, NULL, "<object>", "object index");
    cmd_req_pps_args.voltage_mv = arg_str1(NULL, NULL, "<voltage_mv>", "requested voltage");
    cmd_req_pps_args.current_ma = arg_str1(NULL, NULL, "<current_ma>", "maximum current");
    cmd_req_pps_args.end = arg_end(2);

    cmd_req_obj_args.object = arg_str1(NULL, NULL, "<object>", "object index");
    cmd_req_obj_args.current_ma = arg_str1(NULL, NULL, "<current_ma>", "maximum current");
    cmd_req_obj_args.end = arg_end(2);

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_list_partitions));
    ESP_ERROR_CHECK(esp_console_cmd_register(&req_pps_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&req_obj_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&req_get_src_cap_cmd));
}

void cmd_main(void)
{
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
