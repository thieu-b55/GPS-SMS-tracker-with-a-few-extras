# GPS-SMS-tracker-with-a-few-extras
Tracking a vehicle's position via SMS

Built using an A7670E 4G SIM module and an STM32F411 board.

Functionality:

Reports the module's location via SMS.

Includes 5 inputs and 5 outputs that can be queried or controlled via SMS.

Provides location notifications in the following scenarios:

When the module is powered on.

When the status of any of the 5 inputs changes.

If the module remains powered on:

Sends a notification if the module is moved after a configurable time period (e.g., for a parked
vehicle).

Upon receiving a specific SMS query.

Upon restart, outputs are set to <0> or restored to the state they were in
prior to power-down.

Automatic notifications can be disabled.

Automatically switches to 'parked' status if no programmable movement has occurred within a
configurable period of time.
