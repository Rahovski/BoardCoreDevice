# Board-Control-Manager
Arduino Board Software (Version 0.9.9.2)

Struct for command:

1)-g\n - take a information from board about actual variables(Current motor angle, singal on output pin and signal from buttons)
2)-e\n - -g\n + system info about the pins to which these devices are connected
3)-c\n - list of all devices,connected to control board
4)-p\n - ping control board

For new version of board control Manager need to add: 
1) Dynamic MAC in board 
2) It need to interrupts for reading encoder data.
