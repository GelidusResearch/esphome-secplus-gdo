"""
/*
 * Copyright (C) 2024  Konnected Inc.
 * Copyright (C) 2024  Gelidus Research
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 """

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["preferences"]
MULTI_CONF = True

secplus_gdo_ns = cg.esphome_ns.namespace("secplus_gdo")
SECPLUS_GDO = secplus_gdo_ns.class_("GDOComponent", cg.Component)

CONF_OUTPUT_GDO = "output_gdo_pin"
DEFAULT_OUTPUT_GDO = ("1")
CONF_INPUT_GDO = "input_gdo_pin"
DEFAULT_INPUT_GDO = ("2")
CONF_INPUT_OBST = "input_obst_pin"
CONF_RF_OUTPUT_PIN = "rf_tx_pin"
CONF_RF_INPUT_PIN = "rf_rx_pin"
CONF_DC_OPEN_PIN = "dc_open_pin"
CONF_DC_CLOSE_PIN = "dc_close_pin"
CONF_TOF_DISTANCE_SENSOR = "tof_distance_sensor"
CONF_SECPLUS_GDO_ID = "secplus_gdo_id"


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SECPLUS_GDO),
        cv.Required(CONF_OUTPUT_GDO): pins.gpio_output_pin_schema,
        cv.Required(CONF_INPUT_GDO): pins.gpio_input_pin_schema,
        cv.Optional(CONF_RF_OUTPUT_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_RF_INPUT_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_INPUT_OBST): cv.Any(cv.none, pins.gpio_input_pin_schema),
        cv.Optional(CONF_DC_OPEN_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_DC_CLOSE_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_TOF_DISTANCE_SENSOR): cv.use_id(sensor.Sensor),
    }
).extend(cv.COMPONENT_SCHEMA)

SECPLUS_GDO_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SECPLUS_GDO_ID): cv.use_id(SECPLUS_GDO),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add_define("GDO_UART_TX_PIN", config[CONF_OUTPUT_GDO]["number"])
    cg.add_define("GDO_UART_RX_PIN", config[CONF_INPUT_GDO]["number"])
    if CONF_RF_OUTPUT_PIN in config and config[CONF_RF_OUTPUT_PIN]:
        cg.add_define("GDO_RF_TX_PIN", config[CONF_RF_OUTPUT_PIN]["number"])
    if CONF_RF_INPUT_PIN in config and config[CONF_RF_INPUT_PIN]:
        cg.add_define("GDO_RF_RX_PIN", config[CONF_RF_INPUT_PIN]["number"])
    if CONF_DC_OPEN_PIN in config and config[CONF_DC_OPEN_PIN]:
        cg.add_define("GDO_DC_OPEN_PIN", config[CONF_DC_OPEN_PIN]["number"])
    if CONF_DC_CLOSE_PIN in config and config[CONF_DC_CLOSE_PIN]:
        cg.add_define("GDO_DC_CLOSE_PIN", config[CONF_DC_CLOSE_PIN]["number"])
    if CONF_INPUT_OBST in config and config[CONF_INPUT_OBST]:
        cg.add_define("GDO_OBST_INPUT_PIN", config[CONF_INPUT_OBST]["number"])
        cg.add_define("GDO_OBST_FROM_STATE", False)
    else:
        cg.add_define("GDO_OBST_FROM_STATE", True)

    if CONF_TOF_DISTANCE_SENSOR in config:
        tof_sensor = await cg.get_variable(config[CONF_TOF_DISTANCE_SENSOR])
        cg.add(var.set_external_tof_sensor(tof_sensor))

    cg.add_library(
        name="GDOLIB",
        repository="https://github.com/gelidusresearch/gdolib.git",
        version="1.2.8",
    )
