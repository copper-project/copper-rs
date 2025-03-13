# MSP (MultiWii Serial Protocol) source (reading side)

This component is responsible for reading the MSP (MultiWii Serial Protocol) messages from the serial port.

It can be used standalone or in pair with the MSP sink component for the writing side.

## Configuration

Example in your Copper configuration file:

```RON
    tasks: [
        (
            id: "mspsrc",
            type: "cu_msp_src::MSPSrc",
            config: {
                "device": "/dev/ttyS4",
                "baudrate": 1_000_000,
            },
        ),
   ]
```

### Output

It will feed a MspResponseBatch with all the MspResponses it can read from the serial port at that time.
