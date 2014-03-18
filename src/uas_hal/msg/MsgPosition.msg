time    tick      # Time
string  source	  # Source, eg. 'Differential GPS - L1 / L2'
string  status	  # Status, eg. 'Fix (5 satellites)''
string  cframe    # Reference frame (cartesian, WGS84)
float64 x         # Position X Estimate
float64 y	      # Position Y Estimate
float64 z	      # Position Z Estimate
float64 dx        # Velocity X Estimate
float64 dy        # Velocity Y Estimate
float64 dz  	  # Velocity Z Estimate
float64 err_x	  # Position X Precision
float64 err_y	  # Position Y Precision
float64 err_z	  # Position Z Precision
float64 err_dx	  # Velocity X Precision
float64 err_dy	  # Velocity Y Precision
float64 err_dz	  # Velocity Z Precision
float64 err_clk	  # Clock Precision