int64 label  	# 0: false negative detection; 1: true detection; 2: cluster detection
int64 id   	# target id
string x_class    # target detection type
float32 x_pos_x   # target ground truth position x in global frame
float32 x_pos_y   # target ground truth position y in global frame
string z_class    # target detection type
float32 z_range   # target detection position range in sensor frame
float32 z_bearing # target detection position bearing in sensor frame
int64 z_xmin
int64 z_ymin
int64 z_xmax
int64 z_ymax
