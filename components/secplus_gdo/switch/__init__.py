import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from .. import SECPLUS_GDO_CONFIG_SCHEMA, secplus_gdo_ns, CONF_SECPLUS_GDO_ID

DEPENDENCIES = ["secplus_gdo"]

GDOSwitch = secplus_gdo_ns.class_("GDOSwitch", switch.Switch, cg.Component)

CONF_TYPE = "type"
TYPES = {
    "learn": "register_learn",
    "toggle_only": "register_toggle_only",
    "obst_override": "register_obst_override",
    "close_notification": "register_close_notification",
}


CONFIG_SCHEMA = (
    switch.switch_schema(GDOSwitch)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(SECPLUS_GDO_CONFIG_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if "learn" in str(config[CONF_TYPE]):
        cg.add_define("GDO_LEARN")
        await switch.register_switch(var, config)
    elif "toggle_only" in str(config[CONF_TYPE]):
        cg.add_define("GDO_TOGGLE_ONLY")
        await switch.register_switch(var, config)
    elif "close_notification" in str(config[CONF_TYPE]):
        cg.add_define("GDO_CLOSE_NOTIFY")
        await switch.register_switch(var, config)
    elif "obst_override" in str(config[CONF_TYPE]):
        cg.add_define("GDO_OBST_OVERRIDE")
        await switch.register_switch(var, config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_SECPLUS_GDO_ID])
    fcall = str(parent) + "->" + str(TYPES[config[CONF_TYPE]])
    text = fcall + "(" + str(var) + ")"
    cg.add((cg.RawExpression(text)))
    text = "secplus_gdo::SwitchType::" + str(config[CONF_TYPE]).upper()
    cg.add(var.set_type(cg.RawExpression(text)))
    cg.add_define("GDO_SWITCH")
