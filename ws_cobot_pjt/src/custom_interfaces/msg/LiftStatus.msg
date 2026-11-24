# Turn-lift firmware state (lift package, firmware/car_turn_lift.py): what the open-loop firmware believes
float64 heading    # deg, clockwise seen from above (0 = car front toward the robot)
string station     # top | front
float64 lift_deg   # lift servo angle (firmware DOWN_ANGLE lowered, UP_ANGLE raised)
float64 lift_mm    # holder top above the lift base bottom, from lift_table.json (NaN without the table)
float64 stand_top_mm   # holder top above the table, the virtual turntable included (NaN on the real stand)
float64 vehicle_z_mm   # vehicle frame origin (wheel bottoms) z in BASE on the virtual stand (NaN on the real stand)
