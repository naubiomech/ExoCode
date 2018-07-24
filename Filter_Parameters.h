//To filter the reference (not used right now but still available)
double array_ref[3];
double array_out[3];
double *p_array_ref = array_ref;
double *p_array_out = array_out;
double a_vect[3] = {1.0, -1.864734299516908, 0.869308501948704};
double *p_a_vect = a_vect;
double b_vect[3] = {0.001143550607949, 0.002287101215898, 0.001143550607949};
double *p_b_vect = b_vect;
int flag = 0;
float filterVal = .8;
double smoothedVal = 0;
int clean_flag = 1;
