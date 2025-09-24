[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. 

In the CM33 non-secure application, the clocks and system resources are initialized by the BSP initialization function. The retarget-io middleware is configured to use the debug UART. BTstack is initialized and callback functions are registered. Events handlers are written to handle the btstack callback events.

The code example configures the device as a Bluetooth&reg; LE GAP (Generic Access Profile) Peripheral and GATT (Generic Attribute Profile) Client. The example implements a Alert Notification Profile to act as an Alert Client.

Upon reset, the application starts automatically and initializes BTstack and other device peripherals. The user select option from menu to start advertisement to connect with peer server device. Once a Bluetooth&reg; LE connection is established, the peer client device can control its alerts using ANCP and then client can receive notifications once server generates alert from the supported alert categories.

- **Bluetooth&reg; Configurator:** The Bluetooth&reg; peripheral has an additional configurator called the “Bluetooth&reg; Configurator” that is used to generate the AIROC&trade; Bluetooth&reg; LE GATT database and various Bluetooth&reg; settings for the application. These settings are stored in the file named *design.cybt*

   > **Note:** that unlike the Device Configurator, the Bluetooth&reg; Configurator settings and files are local to each respective application.

   For detailed information on how to use the Bluetooth&reg; Configurator, see the [Bluetooth&reg; Configurator guide](https://www.infineon.com/dgdl/Infineon-ModusToolbox_Bluetooth_Configurator_Guide_3-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d718a49017d99aaf5b231be).

