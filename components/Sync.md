Sync Flag Logic

The sync flag starts as true and is set to false when any of the following conditions are met:

Door State Unknown: If door_state == DoorState::UNKNOWN

Triggers query_status() and sets synced = false
Openings Count Zero: If status->openings == 0

Triggers query_openings() and sets synced = false
Paired Devices Unknown: For each of the following paired device types, if they equal PAIRED_DEVICES_UNKNOWN:

paired_total - triggers query_paired_devices(PairedDevice::ALL)
paired_remotes - triggers query_paired_devices(PairedDevice::REMOTE)
paired_keypads - triggers query_paired_devices(PairedDevice::KEYPAD)
paired_wall_controls - triggers query_paired_devices(PairedDevice::WALL_CONTROL)
paired_accessories - triggers query_paired_devices(PairedDevice::ACCESSORY)
When Sync is Complete
The sync flag remains true (meaning sync is complete) only when ALL of the following conditions are met:

Door state is NOT DoorState::UNKNOWN
Openings count is NOT 0
All paired device counts are NOT PAIRED_DEVICES_UNKNOWN
If synced remains true after all these checks, the function returns immediately, indicating successful synchronization.