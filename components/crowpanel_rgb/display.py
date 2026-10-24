"""crowpanel_rgb: tear-free double-buffered RGB display for the Elecrow CrowPanel 2.1" knob.

Same config as ESPHome's st7701s platform (pins, timing, init sequence); only the frame
pipeline differs. See crowpanel_rgb.h for how it works.
"""

import esphome.codegen as cg
from esphome.components import spi
from esphome.components.st7701s import display as st7701s
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["st7701s"]
DEPENDENCIES = ["spi", "esp32", "lvgl"]

CONF_LOG_FPS = "log_fps"

crowpanel_rgb_ns = cg.esphome_ns.namespace("crowpanel_rgb")
CrowpanelRGB = crowpanel_rgb_ns.class_("CrowpanelRGB", st7701s.ST7701S)

# st7701s.CONFIG_SCHEMA is cv.All(schema, esp32 checks...): reuse it, swapping in our class
_base, *_checks = st7701s.CONFIG_SCHEMA.validators
CONFIG_SCHEMA = cv.All(
    _base.extend(
        {
            cv.GenerateID(): cv.declare_id(CrowpanelRGB),
            cv.Optional(CONF_LOG_FPS, default=False): cv.boolean,
        }
    ),
    *_checks,
)

FINAL_VALIDATE_SCHEMA = spi.final_validate_device_schema(
    "crowpanel_rgb", require_miso=False, require_mosi=True
)


async def to_code(config):
    await st7701s.to_code(config)  # creates our class (declared above) and wires pins/timing
    var = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_log_fps(config[CONF_LOG_FPS]))
