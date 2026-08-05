# -----------------------------------------------------------------------------------
# Lift detector detailed state
# -----------------------------------------------------------------------------------

# Whether lift has been triggered and latched by the detector.
bool is_lifted

# Lift probability after logit smoothing and sigmoid.
float32 probability

# Probability threshold used by the detector.
float32 threshold
