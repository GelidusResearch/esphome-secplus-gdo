"""
/*
 * Copyright (C) 2024  Konnected Inc.
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
from esphome.components import number
from esphome.const import CONF_ID

from .. import SECPLUS_GDO_CONFIG_SCHEMA, secplus_gdo_ns, CONF_SECPLUS_GDO_ID
from .. import CONF_TOF_SDA_PIN

DEPENDENCIES = ["secplus_gdo"]

GDONumber = secplus_gdo_ns.class_("GDONumber", number.Number, cg.Component)
CONF_TYPE = "type"
TYPES = {
    "open_duration": "register_open_duration",
    "close_duration": "register_close_duration",
    "client_id": "register_client_id",
    "rolling_code": "register_rolling_code",
    "min_command_interval": "register_min_command_interval",
    "time_to_close": "register_time_to_close",
    "vehicle_parked_threshold": "register_vehicle_parked_threshold",
}

CONFIG_SCHEMA = (
    number.number_schema(GDONumber)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
            cv.Optional('min_command_interval', default=50): cv.uint32_t,
            cv.Optional('time_to_close', default=300): cv.uint16_t,
            cv.Optional('vehicle_parked_threshold', default=100): cv.uint16_t,
        }
    )
    .extend(SECPLUS_GDO_CONFIG_SCHEMA)
)




async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_SECPLUS_GDO_ID])
    if parent.get(CONF_TOF_SDA_PIN) is None:
        print("TOF sensor")
        if config[CONF_TYPE] != "vehicle_parked_threshold":
            raise cv.Invalid("TOF sensor is only valid with vehicle_parked_threshold")
    if config[CONF_TYPE] == "vehicle_parked_threshold":
        await number.register_number(var, config, min_value=10, max_value=400, step=10)
    elif config[CONF_TYPE] == "duration":
        await number.register_number(var, config, min_value=0x0, max_value=0xffff, step=1)
    elif config[CONF_TYPE] == "client_id":
        await number.register_number(var, config, min_value=0x666, max_value=0x7ff666, step=1)
    elif config[CONF_TYPE] == "min_command_interval":
        await number.register_number(var, config, min_value=50, max_value=1000, step=50)
    elif config[CONF_TYPE] == "time_to_close":
        await number.register_number(var, config, min_value=0, max_value=65535, step=60)
    else:
        await number.register_number(var, config, min_value=0x0, max_value=0xffffffff, step=1)

    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_SECPLUS_GDO_ID])
    fcall = str(parent) + "->" + str(TYPES[config[CONF_TYPE]])
    text = fcall + "(" + str(var) + ")"
    cg.add((cg.RawExpression(text)))
    text = "gdo_set_" + str(config[CONF_TYPE])
    cg.add(var.set_control_function(cg.RawExpression(text)))