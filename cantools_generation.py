import subprocess
import cantools
import re
import os
import shutil
import sys
from collections import OrderedDict

# ==============================================================================
# CONFIGURATION: MULTI-BUS SETUP
# ==============================================================================

# Define your buses here. 
# Prefix: Used for file names, struct names, and function prefixes.
# DBC: The filename in the project root.
BUS_CONFIG = {
#    "inverter": {
#        "dbc": "Inverter.dbc",
#        "rx_receiver": "ECU",
#        "tx_sender": "ECU"
#    },
    "sensor": {
        "dbc": "Sensor.dbc",
        "rx_receiver": "ECU",
        "tx_sender": "ECU"
    }
}

# The component directory where all files will be placed
OUTPUT_DIR = "components/CAN_handler/CAN_generated/"

# Headers required by the generated files
EXTRA_INCLUDES = ["CANTX.h"]

# ==============================================================================
# FORMATTING HELPERS
# ==============================================================================

def to_snake_case(value: str) -> str:
    value = re.sub(r'(.)([A-Z][a-z]+)', r'\1_\2', value)
    value = re.sub(r'(_+)', '_', value)
    value = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', value).lower()
    value = re.sub(r'[^a-zA-Z0-9]', '_', value)
    return value

# ==============================================================================
# GENERATOR ENGINE
# ==============================================================================

def generate_bus_code(bus_prefix, config):
    dbc_path = config["dbc"]
    rx_receiver = config["rx_receiver"]
    tx_sender = config["tx_sender"]
    
    if not os.path.isfile(dbc_path):
        print(f"  [ERROR] DBC file '{dbc_path}' not found. Skipping bus '{bus_prefix}'.")
        return

    # Cantools internal name
    ct_prefix = to_snake_case(os.path.splitext(os.path.basename(dbc_path))[0].lower())
    upper_ct_prefix = ct_prefix.upper()

    # 1. Generate base cantools C source
    subprocess.run([
        sys.executable, "-m", "cantools", "generate_c_source", "--use-float",
        "--database-name", ct_prefix, dbc_path
    ], check=True)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for ext in [".c", ".h"]:
        fname = ct_prefix + ext
        shutil.move(fname, os.path.join(OUTPUT_DIR, fname))
    
    print(f"  [DBC] Successfully generated {ct_prefix}.h and {ct_prefix}.c")

    db = cantools.database.load_file(dbc_path)

    # --------------------------------------------------------------------------
    # RX DATA COLLECTION
    # --------------------------------------------------------------------------
    rx_msgs = [msg for msg in db.messages if not rx_receiver or rx_receiver.lower() in [r.lower() for r in msg.receivers]]
    
    struct_decls = []
    can_data_fields = OrderedDict()
    registry_entries = []
    handler_prototypes = []
    handler_functions = []

    for msg in rx_msgs:
        msg_snake = to_snake_case(msg.name)
        msg_upper = msg_snake.upper()
        func_name = f"handle_{bus_prefix}_{msg_snake}"
        
        # Internal instances for unpacking
        struct_decls.append(f"static struct {ct_prefix}_{msg_snake}_t {bus_prefix}_{msg_snake}_msg;")
        
        # Forward declarations for static handlers (Fixes compilation order)
        handler_prototypes.append(f"static void {func_name}(const void* msg);")

        # Vertical Registry Entry for readability
        entry = (
            f"    {{\n"
            f"        .id     = {upper_ct_prefix}_{msg_upper}_FRAME_ID,\n"
            f"        .sz     = {upper_ct_prefix}_{msg_upper}_LENGTH,\n"
            f"        .ptr    = &{bus_prefix}_{msg_snake}_msg,\n"
            f"        .unp    = (int (*)(void*, const uint8_t*, size_t)){ct_prefix}_{msg_snake}_unpack,\n"
            f"        .hnd    = {func_name}\n"
            f"    }},"
        )
        registry_entries.append(entry)

        # Static Handler Implementation
        h = [f"static void {func_name}(const void* msg) {{"]
        h.append(f"    const struct {ct_prefix}_{msg_snake}_t* m = (const struct {ct_prefix}_{msg_snake}_t*)msg;")
        h.append("")
        h.append(f"    if ({bus_prefix}_data_mutex != NULL) {{")
        h.append(f"        if (xSemaphoreTake({bus_prefix}_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {{")
        
        for sig in msg.signals:
            if not rx_receiver or rx_receiver.lower() in [r.lower() for r in sig.receivers]:
                sig_snake = to_snake_case(sig.name)
                c_type = "bool" if sig.length == 1 else "float"
                
                if sig_snake not in can_data_fields:
                    can_data_fields[sig_snake] = c_type
                
                decode_call = f"{ct_prefix}_{msg_snake}_{sig_snake}_decode(m->{sig_snake})"
                cast = "(bool)" if c_type == "bool" else ""
                h.append(f"            {bus_prefix}_can_data.{sig_snake} = {cast}{decode_call};")
        
        h.append(f"            xSemaphoreGive({bus_prefix}_data_mutex);")
        h.append("        }")
        h.append("    }")
        h.append("}\n")
        handler_functions.append("\n".join(h))

    # --------------------------------------------------------------------------
    # WRITE RX HEADER
    # --------------------------------------------------------------------------
    rx_h_name = f"can_{bus_prefix}_rx_generated.h"
    rx_h = [
        f"#ifndef CAN_{bus_prefix.upper()}_RX_GENERATED_H",
        f"#define CAN_{bus_prefix.upper()}_RX_GENERATED_H",
        "",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        '#include "freertos/FreeRTOS.h"',
        '#include "freertos/semphr.h"',
        f'#include "{ct_prefix}.h"',
        *[f'#include "{inc}"' for inc in EXTRA_INCLUDES],
        "",
        "// ----------------------------------------------------------------------",
        f"// {bus_prefix.upper()} BUS DATA STRUCTURE",
        "// ----------------------------------------------------------------------",
        "typedef struct {",
    ]
    for field, c_type in can_data_fields.items():
        rx_h.append(f"    {c_type:<10} {field};")
    
    rx_h.extend([
        f"}} {bus_prefix}_data_t;",
        "",
        f"extern {bus_prefix}_data_t {bus_prefix}_can_data;",
        f"extern SemaphoreHandle_t {bus_prefix}_data_mutex;",
        "",
        "// Public API",
        f"void {bus_prefix}_can_data_init(void);",
        f"int unpack_{bus_prefix}_message(uint32_t id, const uint8_t *data, size_t len);",
        "",
        f"#endif // CAN_{bus_prefix.upper()}_RX_GENERATED_H"
    ])

    with open(os.path.join(OUTPUT_DIR, rx_h_name), "w") as f:
        f.write("\n".join(rx_h))

    # --------------------------------------------------------------------------
    # WRITE RX SOURCE
    # --------------------------------------------------------------------------
    rx_c_name = f"can_{bus_prefix}_rx_generated.c"
    rx_c = [
        f'#include "{rx_h_name}"',
        "",
        f"{bus_prefix}_data_t {bus_prefix}_can_data;",
        f"SemaphoreHandle_t {bus_prefix}_data_mutex = NULL;",
        "",
        "// ----------------------------------------------------------------------",
        "// INTERNAL FORWARD DECLARATIONS",
        "// ----------------------------------------------------------------------",
        "\n".join(handler_prototypes),
        "",
        "// Static instances for cantools unpacking",
        "\n".join(struct_decls),
        "",
        "typedef struct {",
        "    uint32_t id;",
        "    size_t   sz;",
        "    void    *ptr;",
        "    int    (*unp)(void*, const uint8_t*, size_t);",
        "    void   (*hnd)(const void*);",
        "} msg_desc_t;",
        "",
        "// ----------------------------------------------------------------------",
        "// MESSAGE REGISTRY",
        "// ----------------------------------------------------------------------",
        f"static const msg_desc_t {bus_prefix}_registry[] = {{",
        "\n".join(registry_entries),
        "};",
        "",
        f"void {bus_prefix}_can_data_init(void) {{",
        f"    if ({bus_prefix}_data_mutex == NULL) {{",
        f"        {bus_prefix}_data_mutex = xSemaphoreCreateMutex();",
        "    }",
        "}",
        "",
        f"int unpack_{bus_prefix}_message(uint32_t id, const uint8_t *data, size_t len) {{",
        f"    for (int i = 0; i < (sizeof({bus_prefix}_registry)/sizeof(msg_desc_t)); i++) {{",
        f"        if ({bus_prefix}_registry[i].id == id) {{",
        f"            if (len != {bus_prefix}_registry[i].sz) return -1;",
        f"            if ({bus_prefix}_registry[i].unp({bus_prefix}_registry[i].ptr, data, len) == 0) {{",
        f"                {bus_prefix}_registry[i].hnd({bus_prefix}_registry[i].ptr);",
        "                return 0;",
        "            }",
        "            return -1;",
        "        }",
        "    }",
        "    return -1;",
        "}",
        "",
        "// ----------------------------------------------------------------------",
        "// STATIC HANDLER IMPLEMENTATIONS",
        "// ----------------------------------------------------------------------",
        "\n".join(handler_functions)
    ]

    with open(os.path.join(OUTPUT_DIR, rx_c_name), "w") as f:
        f.write("\n".join(rx_c))
    
    print(f"  [RX]  Successfully generated {rx_h_name} and {rx_c_name}")

    # --------------------------------------------------------------------------
    # TX GENERATION
    # --------------------------------------------------------------------------
    tx_msgs = [msg for msg in db.messages if not tx_sender or tx_sender in msg.senders]
    tx_funcs_c, tx_funcs_h = [], []
    
    for msg in tx_msgs:
        msg_snake = to_snake_case(msg.name)
        msg_upper = msg_snake.upper()
        args, pack_calls = [], []
        
        for sig in msg.signals:
            sig_snake = to_snake_case(sig.name)
            c_type = "float" if (sig.is_float or sig.conversion.scale % 1 != 0) else "bool" if sig.length == 1 else "uint32_t" if sig.length > 16 else "uint16_t"
            args.append(f"{c_type} {sig_snake}")
            pack_calls.append(f"    m.{sig_snake} = {ct_prefix}_{msg_snake}_{sig_snake}_encode((float){sig_snake});")

        func_sig = f"void can_tx_{bus_prefix}_{msg_snake}({', '.join(args)})"
        tx_funcs_h.append(f"{func_sig};")
        
        body = [
            f"{func_sig} {{",
            f"    struct {ct_prefix}_{msg_snake}_t m;",
            "",
            "\n".join(pack_calls),
            "",
            f"    uint8_t d[{upper_ct_prefix}_{msg_upper}_LENGTH];",
            f"    {ct_prefix}_{msg_snake}_pack(d, &m, sizeof(d));",
            f"    send_{bus_prefix}_can_message({upper_ct_prefix}_{msg_upper}_FRAME_ID, d, {upper_ct_prefix}_{msg_upper}_LENGTH);",
            "}"
        ]
        tx_funcs_c.append("\n".join(body))

    tx_h_content = [
        f"#ifndef CAN_{bus_prefix.upper()}_TX_GENERATED_H",
        f"#define CAN_{bus_prefix.upper()}_TX_GENERATED_H",
        "",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        *[f'#include "{inc}"' for inc in EXTRA_INCLUDES],
        "",
        "\n\n".join(tx_funcs_h),
        "",
        f"#endif // CAN_{bus_prefix.upper()}_TX_GENERATED_H"
    ]
    
    tx_c_content = [
        f'#include "can_{bus_prefix}_tx_generated.h"',
        f'#include "{ct_prefix}.h"',
        "",
        "\n\n".join(tx_funcs_c)
    ]

    with open(os.path.join(OUTPUT_DIR, f"can_{bus_prefix}_tx_generated.h"), "w") as f:
        f.write("\n".join(tx_h_content))
    with open(os.path.join(OUTPUT_DIR, f"can_{bus_prefix}_tx_generated.c"), "w") as f:
        f.write("\n".join(tx_c_content))

    print(f"  [TX]  Successfully generated can_{bus_prefix}_tx_generated.h and .c")
    print("-" * 60)

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================

print(f"\nStarting CAN generation in: {OUTPUT_DIR}\n" + "=" * 60)

for bus, config in BUS_CONFIG.items():
    generate_bus_code(bus, config)

print(f"Success! Clean multi-bus files generated.")