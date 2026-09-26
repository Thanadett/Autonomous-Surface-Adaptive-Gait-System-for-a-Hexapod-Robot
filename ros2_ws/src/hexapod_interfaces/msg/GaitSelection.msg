uint8 TRIPOD = 0
uint8 RIPPLE = 1
uint8 WAVE = 2

std_msgs/Header header
uint8 gait
float64[3] probabilities   # [tripod, ripple, wave], sums to ~1.0
float64 confidence         # max(probabilities); safety filter gates on this
string model_version        # e.g. onnx model file name + hash, for logging
float64 inference_ms
bool safety_override         # true if the safety filter changed the gait
string source                 # "model" | "rule_based" | "manual"
