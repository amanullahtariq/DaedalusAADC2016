struct SensorData {
    float value_ir_l_fc;
    float value_ir_l_fl;
    float value_ir_l_fr;
    float value_ir_s_fc;
    float value_ir_s_fl;
    float value_ir_s_fr;
    float value_ir_s_l;
    float value_ir_s_r;
    float value_ir_s_rc;
    float value_us_f_l;
    float value_us_f_r;
    float value_us_r_l;
    float value_us_r_r;
};

enum SensorType {
  IR_L_FC,
  IR_L_FL,
  IR_L_FR,
  IR_S_FC,
  IR_S_FL,
  IR_S_FR,
  IR_S_L,
  IR_S_R,
  IR_S_RC,
  US_F_L,
  US_F_R,
  US_R_L,
  US_R_R
};
