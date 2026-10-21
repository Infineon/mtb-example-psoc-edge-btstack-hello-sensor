# PSOC&trade; Edge MCU: Bluetooth&reg; LE hello sensor

The code example demonstrates the use of a custom Bluetooth&reg; Low Energy service to send notifications from a peripheral device (PSOC&trade; Edge E84 Evaluation kit) to a central device (smartphone) over a user button press on the peripheral device. Additionally, the code enables the blinking of user LED1 on the peripheral device based on the input provided in the central device. This code example uses PSOC&trade; Edge MCU coupled with AIROC&trade; CYW55513/CYW55512 Wi-Fi & Bluetooth&reg; combo chip in the ModusToolbox&trade; software environment.

This code example has a three project structure: CM33 secure, CM33 non-secure, and CM55 projects. All three projects are programmed to the external QSPI flash and executed in Execute in Place (XIP) mode. Extended boot launches the CM33 secure project from a fixed location in the external flash, which then configures the protection settings and launches the CM33 non-secure application. Additionally, CM33 non-secure application enables CM55 CPU and launches the CM55 application.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-psoc-edge-btstack-hello-sensor)

[Provide feedback on this code example.](https://yourvoice.infineon.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzkxOTkiLCJTcGVjIE51bWJlciI6IjAwMi0zOTE5OSIsIkRvYyBUaXRsZSI6IlBTT0MmdHJhZGU7IEVkZ2UgTUNVOiBCbHVldG9vdGgmcmVnOyBMRSBoZWxsbyBzZW5zb3IiLCJyaWQiOiJhcnZpbmRrdW1hci5zdXJlc2hrdW1hckBpbmZpbmVvbi5jb20iLCJEb2MgdmVyc2lvbiI6IjIuMS4wIiwiRG9jIExhbmd1YWdlIjoiRW5nbGlzaCIsIkRvYyBEaXZpc2lvbiI6Ik1DRCIsIkRvYyBCVSI6IklDVyIsIkRvYyBGYW1pbHkiOiJQU09DIn0=)

See the [Design and implementation](docs/design_and_implementation.md) for the functional description of this code example.


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.7 or later (tested with v3.9)
- Board support package (BSP) minimum required version: 1.4.0
- Programming language: C
- Associated parts: All [PSOC&trade; Edge MCU](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm) parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)
- IAR C/C++ Compiler v9.70.4 (`IAR`)
- Arm&reg; Toolchain for Embedded v22.1 (`LLVM_ARM`)


## Supported kits (make variable 'TARGET')

- [PSOC&trade; Edge E84 Evaluation Kit](https://www.infineon.com/KIT_PSE84_EVAL) (`KIT_PSE84_EVAL_EPC2`) – Default value of `TARGET`
- [PSOC&trade; Edge E84 Evaluation Kit](https://www.infineon.com/KIT_PSE84_EVAL) (`KIT_PSE84_EVAL_EPC4`)
- [PSOC&trade; Edge E846 Evaluation Kit](https://www.infineon.com/KIT_PSE846_EVAL) (`KIT_PSE846_EVAL_EPC2`)


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

Ensure the following jumper and pin configuration on the board.
- BOOT SW must be in the HIGH/ON position
- J20 and J21 must be in the tristate/not connected (NC) position (for the PSOC&trade; Edge E84 Evaluation Kit)
- All SW8 DIP switches in OFF position (for the PSOC&trade; Edge E846 Evaluation Kit)


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Install a terminal emulator if you do not have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

Download and install the AIROC&trade; Bluetooth&reg; Connect App on your [iOS](https://apps.apple.com/in/app/airoc-bluetooth-connect-app/id6443702288) or [Android](https://play.google.com/store/apps/details?id=com.infineon.airocbluetoothconnect) phone.

Scan the following QR codes from your mobile phone to download the AIROC&trade; Bluetooth&reg; Connect App.

**Figure 1. AIROC&trade; Bluetooth&reg; Connect App QR code**

![](./images/qr.png)

This example requires no additional software or tools.


## Operation

See [Using the code example](docs/using_the_code_example.md) for instructions on creating a project, opening it in various supported IDEs, and performing tasks, such as building, programming, and debugging the application within the respective IDEs.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud

3. After programming, the application starts automatically. Observe the Bluetooth&reg; stack and application trace messages on the UART terminal connected via KitProg3 COM port <br>

   **Figure 2** shows the output when,
   
   - The kit is programmed for the first time where the identity keys need to be generated newly and is not available in the NVM <br>
   or 
   - The device has been paired before and the identity keys (bonding information) are cleared by long press with User Button 1

   **Figure 2. Log messages on KitProg3 COM port (device not paired)**

   ![](images/serial-log1.png)

   **Figure 3** shows the output when the kit is programmed, where the device was paired before and the identity keys are available in the database

   **Figure 3. Log messages on KitProg3 COM port (device paired)**

   ![](images/serial-log.png)

4. To test using the AIROC&trade; Bluetooth&reg; Connect App, do the following (see the equivalent AIROC&trade; Bluetooth&reg; Connect App screenshot in **Figure 6**):

   1. Turn ON Bluetooth&reg; on your Android or iOS device

   2. Launch the AIROC&trade; Bluetooth&reg; Connect App

   3. Swipe down on the AIROC&trade; Bluetooth&reg; Connect App home screen to start scanning for Bluetooth&reg; LE peripheral devices. Your device named “Hello" displays in the AIROC&trade; Bluetooth&reg; Connect App home screen. Select your device to establish a Bluetooth&reg; LE connection. Once the connection is established, you will see log messages related to connection as follows:

      **Figure 4. Log messages on KitProg3 COM port for connection event**

      ![](./images/connection-status.png)

   4. In the **Services** tab, choose the first characteristic to enable notifications/indications. Register for notifications or indications. This will, in turn, initiate the pairing process

   5. Press the User Button 1 in the kit and observe that a notification/indication is being received in the mobile app

      **Figure 5. Log messages on KitProg3 COM port for first characteristic**

      ![](./images/notify-indicate.png)

      **Figure 6. Testing with the AIROC&trade; Bluetooth&reg; Connect App on Android**

      ![](./images/airoc.png)

   6. You can choose the second characteristic, write a numeric value into it, and observe that the USER LED1 on the board will blink as many times as the number written in the 'Blink' characteristic
   
      > **Note:** The numeric value written should range from 0 to 9 only and has to be written in the **Hex** value field and not in the ASCII field

      **Figure 7. Log messages on KitProg3 COM port for second characteristic**

      ![](./images/read-write-operation.png)

5. To forget a bonded device, long press USER BTN1 for 3 seconds. Ensure that the device is not in a connected state before performing this

   After clearing the pairing information, ensure to forget the device named "Hello" from your phone's Bluetooth&reg; settings. If not done the subsequent connection will fail

   **Figure 8. Log messages on KitProg3 COM port for bond erase event**

   ![](./images/reset-kv-store.png)


## Enable BTSpy logs

1. **airoc-hci-transport** library is required to enable BTSpy logs. Open Library Manager from Quick Panel, click on Add Library, and add **airoc-hci-transport** under **proj_cm33_ns** as shown in **Figure 9** and click **OK** to update

   **Figure 9. Add hci-airoc-transport library**

   ![](./images/airoc-hci-transport.png)

2. Delete **retarget_io** library from the mtb-shared folder using Library Manager tool.

3. Navigate to the application *Makefile* inside the *proj_cm33_ns*. Find the *Makefile* variable `ENABLE_SPY_TRACES` and set it to the value *1* as shown:
```
ENABLE_SPY_TRACES = 1
```
4. Open Device Configurator from the Quick Panel and make the following modifications for **Serial Communication Block (SCB) 2** (under **Peripheral** section) as shown in **Figure 10**
   - Change the **Oversample** value to '8'
   - Select **Enable flow control** to turn on flow control
   - Select the **Clock** as "16.5 bit Divider 0 clk (CYBSP_BT_UART_CLK_DIV)"
   - Select appropriate CTS and RTS pins

   **Figure 10. Enable Flow Control in Debug UART**

   ![](./images/airoc-hci-transport1.png)

5. Save the *Makefile* and Device Configurator. Then build and program the application to the board

6. Download and launch the [BTSpy](https://github.com/Infineon/btsdk-utils/tree/master/BTSpy) tool

7. Click on serial port setup and select **Enable Serial Port**. Then select the correct baudrate, port number, and enable hardware flow control as shown in **Figure 11**

   **Figure 11. Enable BTSpy logs**

   ![](./images/btspy.png)

8. Press and release the reset button (XRES) on the board to get the BTSpy logs on the BTSpy tool

   **Figure 12. BTSpy logs**

   ![](./images/btspy-logs.png)

9. For more traces, call the following function in `application main()`

```
cybt_platform_set_trace_level(CYBT_TRACE_ID_STACK,CYBT_TRACE_LEVEL_DEBUG);
```
This function is defined in *cybt_platform_trace.c* in [btstack-integration](https://github.com/Infineon/btstack-integration) <br>
Set the trace category ID and trace level as per the requirement

> **Note:** Revert back the changes done in **Step 2** , **Step3** (add retarget_io library back using Library Manager tool) and **Step 4** and delete the **airoc-hci-transport** library using the Library Manager tool to switch back to Debug UART logs from BTSpy logs

### Additional steps for other Bluetooth&reg; Low Energy applications 

Perform the below steps additionally for other code examples. This is already implemented in this (BLE Hello Sensor) code example.

1. Make sure to remove retarget_io related files and header informations from your application

2. Add the below code in the main.c file of CM33_NS project by replacing the existing retarget_io initialization.

``` 
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif 

#ifdef ENABLE_BT_SPY_LOG
cybt_debug_uart_config_t config =
{
   .baud_rate = DEBUG_UART_BAUDRATE,
   .flow_control = TRUE
};
cybt_debug_uart_init(&config, NULL);
#else
/* Initialize retarget-io middleware */
init_retarget_io();
#endif
```
 
3. Add the below code in event_handler implementation file. In this case it is app_bt_event_handler.c file

```
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, 
uint8_t* p_data)
{
   cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/* place the below code inside app_bt_application_init()   
   or similar function */

#ifdef ENABLE_BT_SPY_LOG
wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif
```
4. Add the below define in the makefile of CM33_NS project. 

```
# Optionally enable app and Bluetooth protocol traces and route to BTSpy
ENABLE_SPY_TRACES = 1
ifeq ($(ENABLE_SPY_TRACES),1)
DEFINES+=ENABLE_BT_SPY_LOG DEBUG_UART_BAUDRATE=3000000
else
DEFINES+=ENABLE_AIROC_HCI_TRANSPORT_PRINTF=0
endif
```

   
## Resources and settings

This section explains the ModusToolbox&trade; software resources and their configurations as used in this code example. Note that all the configuration explained in this section has already been implemented in the code example.

- **Bluetooth&reg; Configurator:** The Bluetooth&reg; peripheral has a configurator called the “Bluetooth&reg; Configurator” that is used to generate the Bluetooth&reg; LE GATT database and various Bluetooth&reg; settings for the application. These settings are stored in the file named *design.cybt*.

See the [Bluetooth&reg; Configurator guide](https://www.infineon.com/ModusToolboxBLEConfig) for more details.


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge E8 MCU on ModusToolbox&trade; software <br> [AN236697](https://www.infineon.com/AN236697) – Getting started with PSOC&trade; MCU and AIROC&trade; Connectivity devices
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSOC&trade; Edge MCU datasheets](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents) <br> [PSOC&trade; Edge MCU reference manuals](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries  | [mtb-dsl-pse8xxgp](https://github.com/Infineon/mtb-dsl-pse8xxgp) – Device support library for PSE8XXGP <br> [mtb-dsl-pse8x6](https://github.com/Infineon/mtb-dsl-pse8x6) – Device support library for PSE8X6
<br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port <br> [btstack-integration](https://github.com/Infineon/btstack-integration) – The btstack-integration hosts platform adaptation layer (porting layer) between AIROC&trade; BTSTACK and Infineon's different hardware platforms <br> [kv-store](https://github.com/Infineon/kv-store) – This library provides a convenient way to store information as key-value pairs in non-volatile storage
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE239199* – *PSOC&trade; Edge MCU: Bluetooth&reg; LE hello sensor*

 Version | Description of change
 ------- | ---------------------
 1.x.0   | New code example <br> Early access release
 2.0.0   | GitHub release
 2.0.1   | Updated README.md 
 2.1.0   | Updated to support btstack-integration v7.X <br> Added support for KIT_PSE846_EVAL_EPC2

<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

(c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
This software, associated documentation and materials ("Software") is owned by Infineon Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection, worldwide copyright laws, and international treaty provisions. Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software. If no license agreement applies, then any use, reproduction, modification, translation, or compilation of this Software is prohibited without the express written permission of Infineon.
<br>
Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. Infineon reserves the right to make changes to the Software without notice. You are responsible for properly designing, programming, and testing the functionality and safety of your intended application of the Software, as well as complying with any legal requirements related to its use. Infineon does not guarantee that the Software will be free from intrusion, data theft or loss, or other breaches (“Security Breaches”), and Infineon shall have no liability arising out of any Security Breaches. Unless otherwise explicitly approved by Infineon, the Software may not be used in any application where a failure of the Product or any consequences of the use thereof can reasonably be expected to result in personal injury.
