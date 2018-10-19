# Clone
Clone with `git clone --recurse-submodules`
# Build
libopencm3 needs to be built first, this needs to be done only once:
```
$ cd libopencm3
$ make
$ cd ..
```
Regular workflow:
```
$ cd src
$ make
$ make flash
```

# The Serial Protocol
The serial protocol used by the device is a text protocol. The valid
message and response types are listed below. Each message is terminated
by a CRLF sequence. Maximum length of a message is 256 bytes, including
the CRLF terminator. The messages and responses are expressed as `printf`
format strings. The message and response termination sequences are
omitted from the descriptions. This document is written from the
perspective of the host system, messages are sent by the host to the
motor controller and responses are sent by the motor controller to the
host. Aside from critical faults, the motor controller shall not send
any messages to the host system unless such response is specified as
a response to a message sent by the host system. The joint indexing in
the messages starts from one. The valid commands are listed below, in
case of an invalid command following response is sent: `"invalid command"`.

In case of a critical fault in the CPU following message is sent by the
motor controller: `"hard fault, spinning"`. Also the motors will be
halted and brakes engaged.


## Pose Status Query

### Message
`
` This is an empty message which is terminated by the CRLF sequence.

### Response
`"angle: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i"`


## Command

### Message
`"angle: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f, safemode: %i, brake: %i, gripper: %i", dt: %d`
The state specified by the message is placed into the command queue.
The transformation into the specified state is performed by
interpolation from the current states in the time expressed in milliseconds
specified by the `dt` field.

## Response
`"buffer used: %zu/%zu"` Where the first variable is the number of
commands currently pending in the command queue and the latter is the
total capacity of the command queue.


## Buffer Status Query

### Message
`"buffer"` May be abbreviated down to `"b"`

### Response
`"buffer used: %zu/%zu"` Where the first variable is the number of
commands currently pending in the command queue and the latter is the
total capacity of the command queue.


## Current Measurement

### Message
`"current"' May be abbreviated down to '"c"`

### Response
`"current: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f"`


## Set PID Parameters

### Message
`"pid: j: %d, p: %f, i: %f, d: %f, i_max: %f"`
The `j` value in the message is the joint index.

### Response
`"pid: j: %d, p: %f, i: %f, d: %f, i_max: %f"`
The `j` value in the response is the joint index.


## Save PID Parameters

### Message
`"save"` May be abbreviated down to `"s"`
This message saves the currently set PID parameters to the persistent
flash memory.

### Response
The saved PID parameters are listed for each joint in the following
format: `"pid: j: %d, p: %f, i: %f, d: %f, i_max: %f"`


## Load PID Parameters

### Message
`"load"` May be abbreviated down to `"l"`
This message loads the last saved PID parameters from the persistent
flash memory.

### Response
The loaded PID parameters are listed for each joint in the following
format: `"pid: j: %d, p: %f, i: %f, d: %f, i_max: %f"`
