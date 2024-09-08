Prints propane Level, Battery, and Temp from a Mopeka Pro Check Sensor.  You will need to edit the DEV_ADR for your device.

Alternatively edit the code to use:   
// if (peripheral.advertisedServiceUuid() == AD_UUID)  by un-commenting and
then comment out 'if (peripheral.address() == DEV_ADR)'.

The data is advertised in the manufacturer data.

Developed on an old esp32 from doit.
