# MSP (MultiWii Serial Protocol) sink (writing side)

This component is responsible for MSP (MultiWii Serial Protocol) messages on a serial port.

It can be used standalone or in pair with the MSP source component for the reading side.

## Configuration

Example in your Copper configuration file:

```RON
    tasks: [
        (
            id: "mspsink",
            type: "cu_msp_sink::MSPSink",
            config: {
                "device": "/dev/ttyS4",
                "baudrate": 1_000_000,
            },
        ),
   ]
```

### Input

MspRequestBatch: a set of MspRequests to be sent to the serial port.

