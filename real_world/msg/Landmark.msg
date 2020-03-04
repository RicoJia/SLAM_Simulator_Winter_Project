#custom message type for landmark
#position_covariance: assume diagonal standard deviation, [range_stddev, bearing_stddev]


time last_update
uint8 landmark_id
float64 range
float64 bearing
float64[2] position_stddev