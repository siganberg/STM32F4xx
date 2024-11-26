/*

  probe_plugin.c

  Part of grblHAL

  grblHAL is
  Copyright (c) 2022-2023 Terje Io

  Probe Protection code is
  Copyright (c) 2023 Expatria Technologies

  Public domain.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  M401   - Set probe connected.
  M402   - Clear probe Connected.

  NOTES: The symbol TOOLSETTER_RADIUS (defined in grbl/config.h, default 5.0mm) is the tolerance for checking "@ G59.3".
         When $341 tool change mode 1 or 2 is active it is possible to jog to/from the G59.3 position.
         Automatic hard-limit switching when probing at the G59.3 position requires the machine to be homed (X and Y).

  Tip: Set default mode at startup by adding M401 to a startup script ($N0 or $N1)

*/

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "probe_plugin.h"

#define RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy
#define PROBE_DEBOUNCE 25 // ms - increase if probe is slow and/or bouncy

#ifndef DEFAULT_TLS_SIGNAL_INVERT
#define DEFAULT_TLS_SIGNAL_INVERT 0
#endif



//add function pointers for tool number and pulse start


static uint8_t n_ports;
static char max_port[4];

typedef struct {
    bool tls_invert;
} probe_plugin_settings_t;

static probe_state_t tls_input = {
    .connected = On
};

static probe_state_t state = {
    .connected = On
};

static bool probe_away;

static uint8_t tool_probe_port;
static nvs_address_t nvs_address;
static probe_plugin_settings_t probe_plugin_settings;
static on_report_options_ptr on_report_options;
static probe_get_state_ptr SLB_get_state = NULL;
static probe_configure_ptr SLB_probeConfigure = NULL;

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.
static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_Probing, "Probe Protection"}
};

static const setting_detail_t user_settings[] = {
    { Setting_InvertTLSPin, Group_Probing, "Invert TLS input", NULL, Format_Bool, "#0", "0", NULL, Setting_NonCore, &probe_plugin_settings.tls_invert, NULL, NULL },  
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t probe_plugin_settings_descr[] = {
    { Setting_InvertTLSPin, "Invert the TLS input ahead of the OR function.\\n\\n"
                            "NOTE: A reset of the controller is required after changing this setting."
    }, 
};

#endif

// redirected probing function for SLB OR.
static probe_state_t probeSLBGetState (void)
{
    //get the probe state from the HAL
    state = SLB_get_state();
    //get the probe state from the plugin
    tls_input.triggered = hal.port.wait_on_input(Port_Digital, tool_probe_port, WaitMode_Immediate, 0.0f) ^ tls_input.inverted;

    //OR the result and return, unless it is an away probe in which case AND the result.
    if(probe_away)
        state.triggered &= tls_input.triggered;
    else
        state.triggered |= tls_input.triggered;

    return state;
}

// redirected probing function for SLB OR.
static void probeSLBConfigure (bool is_probe_away, bool probing)
{
    tls_input.inverted = is_probe_away ? !probe_plugin_settings.tls_invert : probe_plugin_settings.tls_invert;

    tls_input.triggered = Off;

    tls_input.is_probing = probing;

    probe_away = is_probe_away;

    //call the HAL function.
    if(SLB_probeConfigure)
        SLB_probeConfigure(is_probe_away, probing);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:SLB Probing v0.01]" ASCII_EOL);
    }       
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Probe plugin failed to initialize!", Message_Warning);
}

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe_plugin_settings, sizeof(probe_plugin_settings_t), true);
}

static void warning_no_port (uint_fast16_t state)
{
    report_message("Probe plugin: configured port number is not available", Message_Warning);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void plugin_settings_restore (void)
{
    probe_plugin_settings.tls_invert = DEFAULT_TLS_SIGNAL_INVERT;
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&probe_plugin_settings, sizeof(probe_plugin_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&probe_plugin_settings, nvs_address, sizeof(probe_plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = probe_plugin_settings_descr,
    .n_descriptions = sizeof(probe_plugin_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = plugin_settings_save,
    .load = plugin_settings_load,
    .restore = plugin_settings_restore
};

void probe_protect_init (void)
{
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Input) && (nvs_address = nvs_alloc(sizeof(probe_plugin_settings_t))));

    SLB_get_state = hal.probe.get_state;
    hal.probe.get_state = probeSLBGetState;

    SLB_probeConfigure = hal.probe.configure;
    hal.probe.configure = probeSLBConfigure;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    // Used for setting value validation
    strcpy(max_port, uitoa(n_ports - 1));

    tool_probe_port = SLB_TLS_AUX_INPUT;

    settings_register(&setting_details);

    //claim the TLS pin
    if(ioport_claim(Port_Digital, Port_Input, &tool_probe_port, "Toolsetter Pin")) {
    } else
        protocol_enqueue_rt_command(warning_no_port);    
    //Not an interrupt pin.

    if(!ok)
        protocol_enqueue_rt_command(warning_msg);
}

