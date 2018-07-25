// In this file we adjust he torque reference as a function of the steps as introduced in the A_EXO_s_2_0_2
#ifndef REFERENCE_ADJ_HEADER
#define REFERENCE_ADJ_HEADER

void R_ref_step_adj() {
	if (right_leg->activate_in_3_steps == 1) {

		if (right_leg->first_step == 1) {
			right_leg->coef_in_3_steps = 0;
			right_leg->first_step = 0;
		}

		if ((right_leg->state == 3) && (right_leg->state_old == 1) && (right_leg->start_step == 0)) {
			right_leg->start_step = 1;
			right_leg->start_time = millis();
		}

		if (right_leg->start_step == 1) {
			if ((right_leg->state == 1) && (right_leg->state_old == 3)) {
				right_leg->start_step = 0;
				if (millis() - right_leg->start_time >= step_time_length) { // if the transition from 3 to 1 lasted more than 0.3 sec it was a step
					right_leg->num_3_steps += 1;

					Serial.println(right_leg->coef_in_3_steps);
				}
			}
		}

		right_leg->coef_in_3_steps = right_leg->num_3_steps / 6;


		if (right_leg->coef_in_3_steps >= 1) {
			right_leg->coef_in_3_steps = 1;
			right_leg->activate_in_3_steps = 0;
			right_leg->first_step = 1;
			right_leg->num_3_steps = 0;
			right_leg->start_step = 0;
		}


	}
	return;
}

void L_ref_step_adj() {

	if (left_leg->activate_in_3_steps == 1) {

		if (left_leg->first_step == 1) {
			left_leg->coef_in_3_steps = 0;
			left_leg->coef_in_3_steps_Pctrl = 1;
			left_leg->first_step = 0;
		}

		if ((left_leg->state == 3) && (left_leg->state_old == 1) && (left_leg->start_step == 0)) {
			left_leg->start_time = millis();
			left_leg->start_step = 1;
		}

		if (left_leg->start_step == 1) {
			if ((left_leg->state == 1) && (left_leg->state_old == 3)) {

				left_leg->start_step = 0;
				if (millis() - left_leg->start_time >= step_time_length) {
					left_leg->num_3_steps += 1;

					Serial.print("Left adj/step ");
					Serial.println(left_leg->coef_in_3_steps);
				}
			}
		}

		left_leg->coef_in_3_steps = left_leg->num_3_steps / 6;

		if (left_leg->num_3_steps >= 1) {
			left_leg->coef_in_3_steps_Pctrl = 1;
		}
		if (left_leg->coef_in_3_steps >= 1) {
			left_leg->coef_in_3_steps = 1;
			left_leg->activate_in_3_steps = 0;
			left_leg->first_step = 1;
			left_leg->num_3_steps = 0;

		}


	}


	return;
}
#endif
