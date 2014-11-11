# BLE Gateway example 
This example implements a basic gateway application for the rebroadcasting mesh by utilizing the S110 Softdevice. It allows external Bluetooth Low Energy enabled devices to access the mesh-global GATT service, by adding a softdevice application to the basic application in the LED mesh example. Each node in the mesh will be able to act as an access point for external BLE devices. 

The example is made for nRF51x22 Evaluation and Development kits (PCA10000, PCA10001, PCA10003, both dongles and boards), and the two characteristics provided in the mesh Service represents LED 0 and LED 1 on the board, and RGB RED and GREEN on the dongles. 

## GATT Server
The framework adds a custom Mesh service to the softdevice GATT server, represented by the 128 bit UUID 0x2A1E0001-FD51-D882-8BA8-B98C0000CD1E. The service holds 1 metadata characteristic, represented by the 128 bit UUID 0x2A1E0002-FD51-D882-8BA8-B98C0000CD1E. This characteristic holds the following information

| Data | Bytes | Description |
|------|-------|-------------|
| Access Address | [0-3] | The operational access address of the mesh |
| Adv. Int. | [4-7] | The minimum advertisement interval of the mesh |
| Char. count | [8] | The amount of variable characteristics in the service |
| Channel | [9] | BLE channel the mesh operates on |

This metadata information is static, and will be the same between all nodes in the mesh, apart from adv. int, which may be configured differently without breaking communication with other nodes. External devices will be presented with the local adv. int. when connecting to a node. 

After the metadata characteristic, there is an array of value characteristics. These are shared across the mesh, and may carry a user defined payload of up to 27 bytes each. Each value characteristic comes with a [standard namespace descriptor](https://developer.bluetooth.org/gatt/Pages/GattNamespaceDescriptors.aspx), that carries an enumeration from 0 to 155. Each value characteristic in the service has their own assigned number, and this number is used as handle for the values in the mesh. Each handle-value pair will be the same in every node (when the value has been propagated to all nodes in the mesh).

## LED example
This example implements the mesh with two value characteristics, one for each LED to control. Only the first byte of the payload is checked, a 0 indicates no light, any other value indicates light.
