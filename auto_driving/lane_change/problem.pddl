(define (problem driving01)
	(:domain auto-driving)
	(:objects
		q0 q1 q0_1_1 q0_1_2 q0_1_3 q0_1_4 q0_1_5 q0_1_6 q0_1_7 q0_1_8 q0_1_9 q0_1_10 q0_1_11 q0_1_12 q0_1_13 q0_1_14 q0_1_15 q0_1_16 q0_1_17 q0_1_18 q0_1_19  - conf
		obs0 obs1 obs2 obs3 obs4 obs5  - obstacles
	)
	(:init
		(ego_at q0)
		(= (curr_time) 0)
		(idle)

		(on_right_lane)

		(traj q0 q1)

		(left_traj q0 q1)

		(next q0 q0_1_1 q1)
		(= (at_x q0_1_1) 434.20 )
		(= (at_y q0_1_1) 51.75 )
		(= (at_time q0_1_1) 0.20 )
		(next q0_1_1 q0_1_2 q1)

		(= (at_x q0_1_2) 440.19 )
		(= (at_y q0_1_2) 51.76 )
		(= (at_time q0_1_2) 0.40 )
		(next q0_1_2 q0_1_3 q1)

		(= (at_x q0_1_3) 446.19 )
		(= (at_y q0_1_3) 51.77 )
		(= (at_time q0_1_3) 0.60 )
		(next q0_1_3 q0_1_4 q1)

		(= (at_x q0_1_4) 452.19 )
		(= (at_y q0_1_4) 51.78 )
		(= (at_time q0_1_4) 0.80 )
		(next q0_1_4 q0_1_5 q1)

		(= (at_x q0_1_5) 458.18 )
		(= (at_y q0_1_5) 51.78 )
		(= (at_time q0_1_5) 1.00 )
		(next q0_1_5 q0_1_6 q1)

		(= (at_x q0_1_6) 464.18 )
		(= (at_y q0_1_6) 51.78 )
		(= (at_time q0_1_6) 1.20 )
		(next q0_1_6 q0_1_7 q1)

		(= (at_x q0_1_7) 470.18 )
		(= (at_y q0_1_7) 51.78 )
		(= (at_time q0_1_7) 1.40 )
		(next q0_1_7 q0_1_8 q1)

		(= (at_x q0_1_8) 476.18 )
		(= (at_y q0_1_8) 51.78 )
		(= (at_time q0_1_8) 1.60 )
		(next q0_1_8 q0_1_9 q1)

		(= (at_x q0_1_9) 482.18 )
		(= (at_y q0_1_9) 51.78 )
		(= (at_time q0_1_9) 1.80 )
		(next q0_1_9 q0_1_10 q1)

		(= (at_x q0_1_10) 488.18 )
		(= (at_y q0_1_10) 51.77 )
		(= (at_time q0_1_10) 2.00 )
		(next q0_1_10 q0_1_11 q1)

		(= (at_x q0_1_11) 494.18 )
		(= (at_y q0_1_11) 51.77 )
		(= (at_time q0_1_11) 2.20 )
		(next q0_1_11 q0_1_12 q1)

		(= (at_x q0_1_12) 500.18 )
		(= (at_y q0_1_12) 51.76 )
		(= (at_time q0_1_12) 2.40 )
		(next q0_1_12 q0_1_13 q1)

		(= (at_x q0_1_13) 506.18 )
		(= (at_y q0_1_13) 51.76 )
		(= (at_time q0_1_13) 2.60 )
		(next q0_1_13 q0_1_14 q1)

		(= (at_x q0_1_14) 512.18 )
		(= (at_y q0_1_14) 51.76 )
		(= (at_time q0_1_14) 2.80 )
		(next q0_1_14 q0_1_15 q1)

		(= (at_x q0_1_15) 518.18 )
		(= (at_y q0_1_15) 51.75 )
		(= (at_time q0_1_15) 3.00 )
		(next q0_1_15 q0_1_16 q1)

		(= (at_x q0_1_16) 524.18 )
		(= (at_y q0_1_16) 51.75 )
		(= (at_time q0_1_16) 3.20 )
		(next q0_1_16 q0_1_17 q1)

		(= (at_x q0_1_17) 530.18 )
		(= (at_y q0_1_17) 51.75 )
		(= (at_time q0_1_17) 3.40 )
		(next q0_1_17 q0_1_18 q1)

		(= (at_x q0_1_18) 536.18 )
		(= (at_y q0_1_18) 51.75 )
		(= (at_time q0_1_18) 3.60 )
		(next q0_1_18 q0_1_19 q1)

		(= (at_x q0_1_19) 542.18 )
		(= (at_y q0_1_19) 51.75 )
		(= (at_time q0_1_19) 3.80 )
		(next q0_1_19 q1 q1)

		(= (time_of_traj q0 q1 ) 4.00)


		(= (at_x q0) 428.21 )
		(= (at_y q0) 51.73 )
		(= (at_time q0) 0.00 )

		(= (at_x q1) 542.18 )
		(= (at_y q1) 51.75 )
		(= (at_time q1) 200.00 )



		(= (obst_at_x obs0) 492.31)
		(= (obst_at_y obs0) 48.25)
		(= (obst_at_speed obs0) 30.23)
		(= (obst_at_x obs1) 480.37)
		(= (obst_at_y obs1) 51.75)
		(= (obst_at_speed obs1) 37.05)
		(= (obst_at_x obs2) 525.53)
		(= (obst_at_y obs2) 51.75)
		(= (obst_at_speed obs2) 38.24)
		(= (obst_at_x obs3) 282.20)
		(= (obst_at_y obs3) 51.75)
		(= (obst_at_speed obs3) 0.99)
		(= (obst_at_x obs4) 528.70)
		(= (obst_at_y obs4) 51.75)
		(= (obst_at_speed obs4) 39.91)
		(= (obst_at_x obs5) 446.38)
		(= (obst_at_y obs5) 55.25)
		(= (obst_at_speed obs5) 26.68)
	)
	(:goal (and (on_left_lane)  ))
	(:metric minimize (curr_time))
)
