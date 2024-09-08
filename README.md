Decodes Mopeka Pro Check Level, Battery, and Temp measurement data from advertised manufacturer data.  

You will need to edit the DEV_ADR for your device's address.

Alternatively edit the code to use:   
// if (peripheral.advertisedServiceUuid() == AD_UUID)  by un-commenting and
then comment out 'if (peripheral.address() == DEV_ADR)'.

Developed on an esp32s.
