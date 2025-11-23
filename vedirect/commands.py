from .vedirect import VeDirectDev

class vedirectDEV(VeDirectDev):
    def get_model_name(self) -> str:
        """
        Reads the Model Name.
        Returns:
            str: The device model name.
        """
        return self.get_register('MODEL_NAME')

    def get_serial_number(self) -> str:
        """
        Reads the Serial Number.
        Returns:
            str: The device serial number.
        """
        return self.get_register('SERIAL_NUMBER')

    def get_device_state(self) -> str:
        """
        Reads the Device State (0x0201).
        Returns:
            str: The human-readable device state.
        """
        return self.get_register('DEVICE_STATE')

    def get_device_state_link(self) -> str:
        """
        Reads the Remote Control Device State (0x200C).
        Returns:
            str: The human-readable device state for remote control.
        """
        return self.get_register('DEVICE_STATE_LINK')

    def get_charger_error_code(self) -> str:
        """
        Reads the Charger Error Code (0xEDDA).
        Returns:
            str: The human-readable error message.
        """
        return self.get_register('CHARGER_ERROR_CODE')

    def get_charger_voltage(self) -> float:
        """
        Reads the Charger Voltage.
        Returns:
            float: The voltage in Volts.
        """
        return self.get_register('CHARGER_VOLTAGE')

    def get_charger_current(self) -> float:
        """
        Reads the Charger Current.
        Returns:
            float: The current in Amps.
        """
        return self.get_register('CHARGER_CURRENT')

    def get_panel_voltage(self) -> float:
        """
        Reads the Panel Voltage.
        Returns:
            float: The panel voltage in Volts.
        """
        return self.get_register('PANEL_VOLTAGE')
    
    def get_panel_current(self) -> float:
        """
        Reads the Panel Current (0xEDBD).
        Returns:
            float: The panel current in Amps.
        """
        return self.get_register('PANEL_CURRENT')

    def get_panel_power(self) -> float:
        """
        Reads the panel power (0xEDBC).
        Returns:
            float: panel power in W
        """
        return self.get_register('PANEL_POWER')

    def get_yield_today(self) -> float:
        """
        Reads the Yield Today (0xEDD3).
        Returns:
            float: The yield in kWh.
        """
        return self.get_register('YIELD_TODAY')

    def get_maximum_power_today(self) -> int:
        """
        Reads the Maximum Power Today (0xEDD2).
        Returns:
            int: The max power in Watts.
        """
        return self.get_register('MAXIMUM_POWER_TODAY')
    
    def get_maximum_power_yesterday(self) -> int:
        """
        Reads the Maximum Power Yesterday (0xEDD0).
        Returns:
            int: The max power in Watts.
        """
        return self.get_register('MAXIMUM_POWER_YESTERDAY')

    def get_charger_internal_temp(self) -> float:
        """
        Reads the Charger Internal Temperature (0xEDDB).
        Returns:
            float: The temperature in Celsius.
        """
        return self.get_register('CHARGER_INTERNAL_TEMP')

    def get_battery_voltage_setting(self) -> int:
        """
        Reads the Battery Voltage Setting (0xEDEF).
        Returns:
            int: The voltage setting (e.g., 0=Auto, 12, 24).
        """
        return self.get_register('BATTERY_VOLTAGE_SETTING')

    def get_load_output_control(self) -> str:
        """
        Reads the Load Output Control Mode (0xEDAB).
        Returns:
            str: The human-readable load output mode.
        """
        return self.get_register('LOAD_OUTPUT_CONTROL')

    def get_network_info(self) -> int:
        """
        Reads the Network Info bitmask (0x200D).
        
        Returns:
            int: The raw bitmask value.
        """
        return self.get_register('NETWORK_INFO')

    def set_load_output_control(self, mode: str) -> bool:
        """
        Sets the Load Output Control Mode (0xEDAB).
        Args:
            mode (str): The desired mode (e.g., "OFF", "ON", "AUTO (Batterylife)").
        Returns:
            bool: True on success.
        """
        return self.set_register('LOAD_OUTPUT_CONTROL', mode)

    def set_battery_voltage_setting(self, voltage: int) -> bool:
        """
        Sets the Battery Voltage Setting (0xEDEF).
        Args:
            voltage (int): The desired voltage (e.g., 0=Auto, 12, 24).
        Returns:
            bool: True on success.
        """
        return self.set_register('BATTERY_VOLTAGE_SETTING', voltage)

    def set_device_state_link(self, state: str) -> bool:
        """
        Sets the Remote Control Device State (0x200C).
        Used for slave mode.
        Args:
            state (str): The desired state (e.g., "Bulk", "Float").
        Returns:
            bool: True on success.
        """
        return self.set_register('DEVICE_STATE_LINK', state)

    def set_device_in_external_control_mode(self) -> bool:
        """
        Sets the device to External Control Mode (0x200E bit 2).
        Bit 2 = External control mode (Remote control of vset / iset).
        Set 0x200E to b0000101, write to registers 0x2001 and/or 0x2015
        0x2001 Charge voltage set-point
        0x2015 Charge current limit
        Returns:
            bool: True on success.
        """
        return self.set_register('NETWORK_MODE', int(0b0000101))

    def set_device_in_stand_alone_mode(self) -> bool:
        """
        Sets the device to Stand-alone mode.
        Sets 0x200E to 0.
        Returns:
            bool: True on success.
        """
        return self.set_register('NETWORK_MODE', 0)