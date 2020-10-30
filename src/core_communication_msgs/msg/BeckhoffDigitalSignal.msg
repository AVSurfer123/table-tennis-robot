# This message contains the state of a specific out or input on a Beckhoff terminal.
string terminal # terminal type (e.g. EL1014)
uint8 position # relative position of the terminal in the ethercat bus (0-n)
uint8 port # specifies the port on the selected terminal (0-n)
bool state # the state of the port
