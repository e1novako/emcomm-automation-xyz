#ifdef C4NODEMCU_MAIN
// Device commands...
const char *COMMAND_RESTART                   = "restart";

const char *COMMAND_TOGGLE                    = "toggle";
const char *COMMAND_FIBARO_ADD                = "fibaro_add";
const char *COMMAND_AEOTEC_LIGHT_ADD          = "aeotec_light_add";
const char *COMMAND_AEOTEC_LIGHT_REMOVE       = "aeotec_light_remove";
const char *COMMAND_FIBARO_TRIG_FLOOD         = "fibaro_flood_trigger";

const char *COMMAND_ZEN77_ADD                 = "zen77_add";
const char *COMMAND_ZEN77_REMOVE              = "zen77_remove";
const char *COMMAND_ZEN76_ADD                 = "zen76_add";
const char *COMMAND_ZEN76_REMOVE              = "zen76_remove";
const char *COMMAND_ZEN72_ADD                 = "zen72_add";
const char *COMMAND_ZEN72_REMOVE              = "zen72_remove";
const char *COMMAND_ZEN71_ADD                 = "zen71_add";
const char *COMMAND_ZEN71_REMOVE              = "zen71_remove";
const char *COMMAND_VESTA162_ADD              = "vesta162_add";
const char *COMMAND_VESTA162_REMOVE           = "vesta162_remove";

const char *COMMAND_CONTROL4_IDENTIFY         = "control4_identify";
const char *COMMAND_CONTROL4_CHANNEL          = "control4_channel";
const char *COMMAND_CONTROL4_REBOOT           = "control4_reboot";
const char *COMMAND_CONTROL4_FACTORY_RESET    = "control4_factory_reset";
const char *COMMAND_CONTROL4_LEAVE_MESH       = "control4_leave_mesh";
const char *COMMAND_CONTROL4_PUCK_LEAVE_MESH  = "control4_puck_leave_mesh";

const char *COMMAND_IKEA_TRETAKT_IDENTIFY     = "ikea_tretakt_identify";
const char *COMMAND_IKEA_TRETAKT_TOGGLE       = "ikea_tretakt_toggle";
const char *COMMAND_IKEA_TRETAKT_FACTORY_RESET = "ikea_tretakt_factory_reset";

const char *COMMAND_MOVE_AXIS                 = "move_axis";

#else
extern  const char    *COMMAND_RESTART, *COMMAND_TOGGLE, *COMMAND_FIBARO_ADD;
extern  const char    *COMMAND_AEOTEC_LIGHT_ADD, *COMMAND_AEOTEC_LIGHT_REMOVE, *COMMAND_FIBARO_TRIG_FLOOD;
extern  const char    *COMMAND_ZEN77_ADD, *COMMAND_ZEN77_REMOVE, *COMMAND_ZEN76_ADD, *COMMAND_ZEN76_REMOVE;
extern  const char    *COMMAND_ZEN72_ADD, *COMMAND_ZEN72_REMOVE, *COMMAND_ZEN71_ADD, *COMMAND_ZEN71_REMOVE;
extern  const char    *COMMAND_CONTROL4_IDENTIFY, *COMMAND_CONTROL4_CHANNEL, *COMMAND_CONTROL4_REBOOT;
extern  const char    *COMMAND_CONTROL4_FACTORY_RESET, *COMMAND_CONTROL4_LEAVE_MESH;
extern  const char    *COMMAND_VESTA162_ADD, *COMMAND_VESTA162_REMOVE, *COMMAND_CONTROL4_PUCK_LEAVE_MESH;
extern  const char    *COMMAND_IKEA_TRETAKT_IDENTIFY, *COMMAND_IKEA_TRETAKT_TOGGLE, *COMMAND_IKEA_TRETAKT_FACTORY_RESET;
extern  const char    *MOVE_AXIS;
#endif
