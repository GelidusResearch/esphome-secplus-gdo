import esphome.codegen as cg
from esphome.components import button
from esphome.const import CONF_ID

from .. import SECPLUS_GDO_CONFIG_SCHEMA, secplus_gdo_ns

DEPENDENCIES = ["secplus_gdo"]

GDOButton = secplus_gdo_ns.class_("GDOButton", button.Button)

CONFIG_SCHEMA = button.button_schema(GDOButton).extend(SECPLUS_GDO_CONFIG_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await button.register_button(var, config)
