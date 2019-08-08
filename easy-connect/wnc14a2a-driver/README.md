
# Easy Connect with the wnc14a2a-library

## Specifying the connectivity method

Add the following to your `mbed_app.json` file:

```json
{
    "config": {
        "network-interface":{
            "help": "options are ETHERNET, WIFI_ESP8266, WIFI_IDW0XX1, WIFI_ODIN, WIFI_RTW, WIFI_WIZFI310, WIFI_ISM43362, MESH_LOWPAN_ND, MESH_THREAD, CELLULAR_ONBOARD, CELLULAR_WNC14A2A",
            "value": "CELLULAR_WNC14A2A"
        }
    },
    }
}
```

## Debug settings
You may also want to specify the following to obtain debug output. This is added to the "config" section:

```json
    "config": {
       "wnc_debug": {
           "help" : "enable or disable WNC debug messages.",
           "value": 0
       },
       "wnc_debug_setting": {
           "help" : "bit value 1 and/or 2 enable WncController debug output, bit value 4 enables mbed driver debug output.",
           "value": "4"
       }
    }
```

where the wnc_debug is either 'true' or 'false' to enable or disable debug output and the `wnc_debug_setting` variable is a 
bit-field with the following settings:
     0x01 - Basic WNC driver debug output
     0x02 - Comprehensive WNC driver debug output
     0x04 - Driver Enter/Exit debug information
     0x08 - Driver Event Queue debug information
     0x10 - SMS debug information
     0x20 - Dump Driver Arrays

## Overriding settings
You need to increase the size of the Tx and Rx buffers used when communicating to the WNC device by adding (increas the buffer
size to 4KB from 256 Bytes):


```json
    "target_overrides": {
        "*": {
            "drivers.uart-serial-txbuf-size": 4096,
            "drivers.uart-serial-rxbuf-size": 4096
        }
```

