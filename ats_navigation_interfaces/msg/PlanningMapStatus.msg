# adapter 对单次 planning-grid publication 的结构化状态。
std_msgs/Header header
bool ready
uint64 localization_epoch
uint64 rog_generation
uint64 publication_sequence
string message
