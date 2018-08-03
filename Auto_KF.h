int flag_auto_KF = 0;

void Auto_KF() {

	// take error in state 3
	if (left_leg->state == 3) {
		//    left_leg->Input is the average of the measured torque
		//    left_leg->PID_Stepoint is the reference
		Serial.print(" Left Error ");
		Serial.println(left_leg->PID_Setpoint - left_leg->Input );
		left_leg->ERR += (left_leg->PID_Setpoint - left_leg->Input );
		left_leg->count_err++;
	}

	if (left_leg->state == 1) {

		left_leg->ERR = left_leg->ERR / left_leg->count_err;
		if ((left_leg->count_err != 0)) {
			Serial.print("Left ERR ");
			Serial.println(left_leg->ERR);
		}
		else {

		}
		left_leg->count_err = 0;



		if (left_leg->ERR > max_ERR) {
			left_leg->KF += 0.05;
		}
		else if (left_leg->ERR < min_ERR) {
			left_leg->KF -= 0.05;
		}
		else {}

		if (left_leg->KF >= left_leg->max_KF)
			left_leg->KF = left_leg->max_KF;
		else if (left_leg->KF <= left_leg->min_KF)
			left_leg->KF = left_leg->min_KF;
		else {}

		Serial.print("New left_leg->KF ");
		Serial.println(left_leg->KF);
		left_leg->ERR = 0;
	}


	if (right_leg->state == 3) {
		//    right_leg->Input is the average of the measured torque
		//    right_leg->PID_Stepoint is the reference
		Serial.print(" Right Error ");
		Serial.println(right_leg->PID_Setpoint - right_leg->Input );
		right_leg->ERR += (right_leg->PID_Setpoint - right_leg->Input );
		right_leg->count_err++;
	}
	if (right_leg->state == 1) {

		right_leg->ERR = -right_leg->ERR / right_leg->count_err; // because the right has a different sign
		if ((right_leg->count_err != 0)) {
			Serial.print(" Right ERR ");
			Serial.println(right_leg->ERR);
		}
		else {

		}
		right_leg->count_err = 0;


		if (right_leg->ERR > max_ERR) {
			right_leg->KF += 0.05;
		}
		else if (right_leg->ERR < min_ERR) {
			right_leg->KF -= 0.05;
		}
		else {}

		if (right_leg->KF >= right_leg->max_KF)
			right_leg->KF = right_leg->max_KF;
		else if (right_leg->KF <= right_leg->min_KF)
			right_leg->KF = right_leg->min_KF;
		else {}

		Serial.print("New right_leg->KF ");
		Serial.println(right_leg->KF);
		right_leg->ERR = 0;
	}


	//  left_leg->ERR += ()
	// adjust KF

	return;
}
