
double Check_LED_BT(const unsigned int LED_BT_PIN_l, double LED_BT_Voltage_l, int* p_count_l) {

  LED_BT_Voltage_l += analogRead(LED_BT_PIN_l) * 3.3 / 4096;
  (*p_count_l)++;

  if (*(p_count_l) >= 3) {

    LED_BT_Voltage_l = LED_BT_Voltage_l / (*(p_count_l));
    *(p_count_l) = 0;

    if ( LED_BT_Voltage_l <= 2 && flag_done_once_bt == false) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DISCONNECTED!  ");
      Serial.print("VOLTAGE : ");
      Serial.println(LED_BT_Voltage_l);

      bluetooth.flush();
      flag_done_once_bt = true;

    }

    if (LED_BT_Voltage_l > 2.5 && flag_done_once_bt == true) {
      flag_done_once_bt = false;
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RECONNECTED!  ");
      Serial.print("VOLTAGE : ");
      Serial.println(LED_BT_Voltage_l);
    }
  }

  return analogRead(LED_BT_PIN_l) * 3.3 / 4096;
}
