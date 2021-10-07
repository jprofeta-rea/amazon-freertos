<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<smc>
<general version="2.0.0.0">
<configuration active="true" id="${configurationTypeReleaseID}">
<property id="com.renesas.smc.service.project.buildArtefactType" values="com.renesas.smc.service.project.buildArtefactType.exe"/>
<toolchain id="${toolChainID}">
<option id="com.renesas.smc.toolchain.option.buildArtefactType" key="com.renesas.smc.toolchain.option.buildArtefactType.exe"/>
<option id="com.renesas.smc.toolchain.option.rtos" key="com.renesas.smc.toolchain.option.rtos.freertos">
<item id="com.renesas.smc.toolchain.option.rtos.freertos" value="AmazonFreeRTOS_RX_${packageVersion}"/>
</option>
</toolchain>
</configuration>
<platform id="${targetDevice}"/>
<option id="board" value="Custom User Board"/>
</general>
<tool id="Summary" version="1.0.0.0">
<option id="com.renesas.smc.code.type" value="Normal Folder"/>
</tool>
<tool id="SWComponent" version="1.0.0.0">
<configuration inuse="true" name="r_bsp">
<component display="r_bsp" id="r_bsp6.11" version="6.11">
<gridItem id="BSP_CFG_CODE_FLASH_BANK_MODE" selectedIndex="0"/>
<gridItem id="BSP_CFG_USER_STACK_ENABLE" selectedIndex="1"/>
<gridItem id="BSP_CFG_SWINT_UNIT2_ENABLE" selectedIndex="1"/>
</component>
<source id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="r_wifi_sx_ulpgn">
<component description="Dependency : r_bsp version(s) 5.20&#10;Dependency : r_byteq version(s) 1.80&#10;Dependency : r_sci_rx version(s) 3.20&#10;Use UART to control SX-ULPGN and connect to the Internet." detailDescription="WI-FI Module control functions for Renesas MCUs." display="r_wifi_sx_ulpgn" id="r_wifi_sx_ulpgn1.11" version="1.11">
<gridItem id="WIFI_CFG_SCI_CHANNEL" selectedIndex="6"/>
<gridItem id="WIFI_CFG_SCI_SECOND_CHANNEL" selectedIndex="11"/>
<gridItem id="WIFI_CFG_SCI_INTERRUPT_LEVEL" selectedIndex="4"/>
<gridItem id="WIFI_CFG_SCI_BAUDRATE" selectedIndex="460800"/>
<gridItem id="WIFI_CFG_SCI_USE_FLOW_CONTROL" selectedIndex="1"/>
<gridItem id="WIFI_CFG_RESET_PORT" selectedIndex="7"/>
<gridItem id="WIFI_CFG_RESET_PIN" selectedIndex="4"/>
<gridItem id="WIFI_CFG_RTS_PORT" selectedIndex="0"/>
<gridItem id="WIFI_CFG_RTS_PIN" selectedIndex="2"/>
<gridItem id="WIFI_CFG_CREATABLE_SOCKETS" selectedIndex="4"/>
<gridItem id="WIFI_CFG_SOCKETS_RECEIVE_BUFFER_SIZE" selectedIndex="1024"/>
<gridItem id="WIFI_CFG_USE_CALLBACK_FUNCTION" selectedIndex="0"/>
<gridItem id="WIFI_CFG_CALLBACK_FUNCTION_NAME" selectedIndex="NULL"/>
</component>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="r_s12ad_rx">
<component display="r_s12ad_rx" id="r_s12ad_rx4.80" version="4.80">
<gridItem id="S12AD0" selectedIndex="1"/>
<gridItem id="S12AD0" selectedIndex="0"/>
</component>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="r_flash_rx">
<component display="r_flash_rx" id="r_flash_rx4.70" version="4.70">
<gridItem id="FLASH_CFG_PARAM_CHECKING_ENABLE" selectedIndex="1"/>
<gridItem id="FLASH_CFG_CODE_FLASH_ENABLE" selectedIndex="1"/>
<gridItem id="FLASH_CFG_DATA_FLASH_BGO" selectedIndex="1"/>
<gridItem id="FLASH_CFG_CODE_FLASH_BGO" selectedIndex="1"/>
<gridItem id="FLASH_CFG_CODE_FLASH_RUN_FROM_ROM" selectedIndex="1"/>
</component>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="r_sci_rx">
<component display="r_sci_rx" id="r_sci_rx3.80" version="3.80">
<gridItem id="SCI_CFG_CH6_INCLUDED" selectedIndex="1"/>
<gridItem id="SCI_CFG_CH10_INCLUDED" selectedIndex="1"/>
<gridItem id="SCI_CFG_CH11_INCLUDED" selectedIndex="1"/>
<gridItem id="SCK6" selectedIndex="0"/>
<gridItem id="RXD6/SMISO6/SSCL6" selectedIndex="1"/>
<gridItem id="TXD6/SMOSI6/SSDA6" selectedIndex="1"/>
<gridItem id="CTS6#/RTS6#/SS6#" selectedIndex="1"/>
<gridItem id="SCI6" selectedIndex="1"/>
<gridItem id="SCK10" selectedIndex="0"/>
<gridItem id="RXD10/SMISO10/SSCL10" selectedIndex="1"/>
<gridItem id="TXD10/SMOSI10/SSDA10" selectedIndex="1"/>
<gridItem id="SS10#" selectedIndex="0"/>
<gridItem id="CTS10#" selectedIndex="0"/>
<gridItem id="RTS10#" selectedIndex="0"/>
<gridItem id="SCI10" selectedIndex="1"/>
<gridItem id="SCK11" selectedIndex="0"/>
<gridItem id="RXD11/SMISO11/SSCL11" selectedIndex="1"/>
<gridItem id="TXD11/SMOSI11/SSDA11" selectedIndex="1"/>
<gridItem id="SS11#" selectedIndex="0"/>
<gridItem id="CTS11#" selectedIndex="0"/>
<gridItem id="RTS11#" selectedIndex="0"/>
<gridItem id="SCI11" selectedIndex="1"/>
<gridItem id="SCI_CFG_CH6_TX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_CH10_TX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_CH11_TX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_CH6_RX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_CH10_RX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_CH11_RX_BUFSIZ" selectedIndex="2048"/>
<gridItem id="SCI_CFG_TEI_INCLUDED" selectedIndex="1"/>
</component>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="r_byteq">
<component display="r_byteq" id="r_byteq1.90" version="1.90"/>
<source description="Components supporting Firmware Integration Technology" display="Firmware Integration Technology" id="com.renesas.smc.tools.swcomponent.fit.source"/>
</configuration>
<configuration inuse="true" name="FreeRTOS_Application">
<component display="FreeRTOS_Application" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.application">
<gridItem id="CONNECTION" selectedIndex="0"/>
<gridItem id="OTA" selectedIndex="0"/>
</component>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="FreeRTOS_Object">
<component display="FreeRTOS_Object" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.object"/>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="FreeRTOS_Kernel">
<component display="FreeRTOS_Kernel" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.kernel">
<gridItem id="configTOTAL_HEAP_SIZE" selectedIndex="( size_t ) ( 256U * 1024U )"/>
</component>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="AWS_device_shadow">
<component display="AWS_device_shadow" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.device_shadow"/>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="AWS_tcp_ip">
<component display="AWS_tcp_ip" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.tcp_ip">
<gridItem id="ipconfigINCLUDE_FULL_INET_ADDR" selectedIndex="0"/>
<gridItem id="ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS" selectedIndex="8"/>
<gridItem id="ipconfigUSE_TCP_WIN" selectedIndex="0"/>
</component>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="AWS_mqtt">
<component display="AWS_mqtt" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.mqtt">
<gridItem id="mqttconfigMQTT_TASK_STACK_DEPTH" selectedIndex="6114"/>
</component>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="AWS_secure_socket">
<component display="AWS_secure_socket" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.secure_socket"/>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
<configuration inuse="true" name="AWS_ggd">
<component display="AWS_ggd" id="com.renesas.smc.tools.swcomponent.rtosconfigurator.freertos.amazon.ggd"/>
<source id="com.renesas.smc.tools.swcomponent.rtosconfigurator.source"/>
</configuration>
</tool>
<tool id="Clock">
<option enabled="false" id="hocofrequency" selection="frq20"/>
<option enabled="true" id="hocooscenable" selection="uncheck"/>
<option enabled="true" id="mainclockenable" selection="uncheck"/>
<option enabled="false" id="mainsourcebox" selection="srcR"/>
<option enabled="false" id="mainfrequency" selection="textinputitem"/>
<option enabled="false" id="mainwaittime" selection="textinputitem"/>
<option enabled="true" id="hococlockenable" selection="check"/>
<option enabled="true" id="uckdivider" selection="comboBox1-5">
<item enabled="false" id="comboBox1-2"/>
<item enabled="false" id="comboBox1-3"/>
<item enabled="false" id="comboBox1-4"/>
<item enabled="true" id="comboBox1-5"/>
</option>
<option enabled="true" id="hocooscenable" selection="uncheck"/>
<option enabled="true" id="pllswitcher" selection="pllhoco">
<item enabled="false" id="pllmain" input="" value="24.0"/>
<item enabled="true" id="pllhoco" input="" value="20"/>
</option>
<option enabled="true" id="plldivider" selection="div1-1">
<item enabled="true" id="div1-1"/>
<item enabled="true" id="div1-2"/>
<item enabled="false" id="div1-3"/>
</option>
<option enabled="true" id="pllmul" selection="mul12-1">
<item enabled="true" id="mul10_5-1"/>
<item enabled="true" id="mul11-1"/>
<item enabled="true" id="mul11_5-1"/>
<item enabled="true" id="mul12-1"/>
</option>
<option enabled="true" id="sckswitcher" selection="pll">
<item enabled="false" id="main" input="" value="2.4E7"/>
<item enabled="true" id="hoco" input="" value="20000000"/>
</option>
<option enabled="true" id="uclk" selection="textoutputitem">
<item enabled="false" id="textoutputitem" input="" value="48.0"/>
</option>
<option enabled="true" id="cachclk" selection="textoutputitem">
<item enabled="true" id="textoutputitem" input="" value="20"/>
</option>
<option enabled="false" id="cacmclk" selection="textoutputitem">
<item enabled="true" id="textoutputitem" input="" value="24.0"/>
</option>
<Item current="true" id="uckdivider.selectBox"/>
<Item current="240.0 MHz" id="uckdivider.InputValue"/>
<Item current="48.0 MHz" id="uckdivider.OutputValue"/>
<Item current="sckselector.radiobutton.swtpll" id="uckdivider.Condition"/>
<Item current="comboBox1-2" error="false" id="uckdivider.comboBox"/>
</tool>
<tool id="Pins" version="1.0.1.0">
<pinItem allocation="20" comments="" direction="None" id="XTAL" isUsedBySoftware="false" locked="false" status="0"/>
<pinItem allocation="39" comments="" direction="None" id="TXD10" isUsedBySoftware="true" locked="false" status="0"/>
<pinItem allocation="68" comments="" direction="None" id="TXD11" isUsedBySoftware="true" locked="false" status="4"/>
<pinItem allocation="7" comments="" direction="None" id="RXD6" isUsedBySoftware="false" locked="false" status="0"/>
<pinItem allocation="8" comments="" direction="None" id="TXD6" isUsedBySoftware="true" locked="false" status="0"/>
<pinItem allocation="41" comments="" direction="None" id="RXD10" isUsedBySoftware="false" locked="false" status="0"/>
<pinItem allocation="69" comments="" direction="None" id="RXD11" isUsedBySoftware="false" locked="false" status="4"/>
<pinItem allocation="22" comments="" direction="None" id="EXTAL" isUsedBySoftware="false" locked="false" status="0"/>
<pinItem allocation="13" comments="" direction="None" id="CTS6#" isUsedBySoftware="false" locked="false" status="0"/>
</tool>
</smc>
