
#ifndef C4NODEMCU_GPIO
extern  void    gpioSet(int numRelay, int newState), angle(int m, int a);
extern  boolean gpioState(int numRelay), is_valid_gpio(String which);
extern  String  relayState(int numRelay);
#endif

#ifndef C4NODEMCU_COMMANDS
extern  void    toggle(int m), aeotec_light_add(int m), aeotec_light_remove(int m);
extern  void    fibaro_add(int m), vesta162(int m), fibaro_flood_trigger(int m);
extern  void    control4_reboot(int m), control4_channel(int m), control4_leave_mesh(int m);
extern  void    control4_identify(int m), control4_puck_leave_mesh(int m), control4_factory_reset(int m);
extern  void    ikea_tretakt_identify(int m), ikea_tretakt_toggle(int m), ikea_tretakt_factory_reset(int m);
#endif

#ifndef C4NODEMCU_OTA
extern  void    printProgress(int prog, int len);
extern  void    http_redirect(AsyncWebServerRequest *request, String location, String rfrsh_time, String msg);
extern  void    handleDoUpdate(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final);
#endif

#ifndef C4NODEMCU_NETWORK
extern  void    start_network_services(void);
#endif