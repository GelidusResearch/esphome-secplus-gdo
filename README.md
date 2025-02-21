# Secplus GDO for ESPHome

Secplus GDO is an ESPHome component for controlling Security+ and Security+ 2.0 garage openers
made by Chamberlain Group, including garage door openers sold since 1997 under the Chamberlain,
LiftMaster, Craftsman and Merlin brands.

## Adapted from [RATGDO](https://github.com/ratgdo) and [Secplus GDO](https://github.com/konnected-io)



### Project Goals
The ratgdo and [secplus](https://github.com/argilo/secplus) developers found a way to communicate with Security+ garage door openers over a reverse-engineered serial wireline and RF protocols. This novel development enabled owners of Security+ garage openers to read and send commands directly to the garage unit.
At Gelidus we are seeking to reverse engineer the MyQ smart controllers further and develop RF based controllers.

## Dependencies

1. [gdolib](https://github.com/gelidusresearch/gdolib)