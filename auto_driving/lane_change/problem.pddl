(define (problem driving01)
	(:domain auto-driving)
	(:objects
		q0 q1 q0_1_1 q0_1_2 q0_1_3 q0_1_4 q0_1_5 q0_1_6 q0_1_7 q0_1_8 q0_1_9 q0_1_10 q0_1_11 q0_1_12 q0_1_13 q0_1_14 q0_1_15 q0_1_16 q0_1_17 q0_1_18 q0_1_19 q0_1_20 q0_1_21 q0_1_22 q0_1_23 q0_1_24 q0_1_25 q0_1_26 q0_1_27 q0_1_28 q0_1_29 q0_1_30  - conf
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
		(= (at_x q0_1_1) 770.23 )
		(= (at_y q0_1_1) 50.01 )
		(= (at_time q0_1_1) 0.20 )
		(next q0_1_1 q0_1_2 q1)

		(= (at_x q0_1_2) 776.00 )
		(= (at_y q0_1_2) 50.01 )
		(= (at_time q0_1_2) 0.40 )
		(next q0_1_2 q0_1_3 q1)

		(= (at_x q0_1_3) 781.77 )
		(= (at_y q0_1_3) 50.01 )
		(= (at_time q0_1_3) 0.60 )
		(next q0_1_3 q0_1_4 q1)

		(= (at_x q0_1_4) 787.54 )
		(= (at_y q0_1_4) 50.02 )
		(= (at_time q0_1_4) 0.80 )
		(next q0_1_4 q0_1_5 q1)

		(= (at_x q0_1_5) 793.32 )
		(= (at_y q0_1_5) 50.04 )
		(= (at_time q0_1_5) 1.00 )
		(next q0_1_5 q0_1_6 q1)

		(= (at_x q0_1_6) 799.11 )
		(= (at_y q0_1_6) 50.07 )
		(= (at_time q0_1_6) 1.20 )
		(next q0_1_6 q0_1_7 q1)

		(= (at_x q0_1_7) 804.90 )
		(= (at_y q0_1_7) 50.11 )
		(= (at_time q0_1_7) 1.40 )
		(next q0_1_7 q0_1_8 q1)

		(= (at_x q0_1_8) 810.69 )
		(= (at_y q0_1_8) 50.17 )
		(= (at_time q0_1_8) 1.60 )
		(next q0_1_8 q0_1_9 q1)

		(= (at_x q0_1_9) 816.50 )
		(= (at_y q0_1_9) 50.23 )
		(= (at_time q0_1_9) 1.80 )
		(next q0_1_9 q0_1_10 q1)

		(= (at_x q0_1_10) 822.32 )
		(= (at_y q0_1_10) 50.31 )
		(= (at_time q0_1_10) 2.00 )
		(next q0_1_10 q0_1_11 q1)

		(= (at_x q0_1_11) 828.14 )
		(= (at_y q0_1_11) 50.40 )
		(= (at_time q0_1_11) 2.20 )
		(next q0_1_11 q0_1_12 q1)

		(= (at_x q0_1_12) 833.98 )
		(= (at_y q0_1_12) 50.50 )
		(= (at_time q0_1_12) 2.40 )
		(next q0_1_12 q0_1_13 q1)

		(= (at_x q0_1_13) 839.83 )
		(= (at_y q0_1_13) 50.60 )
		(= (at_time q0_1_13) 2.60 )
		(next q0_1_13 q0_1_14 q1)

		(= (at_x q0_1_14) 845.69 )
		(= (at_y q0_1_14) 50.71 )
		(= (at_time q0_1_14) 2.80 )
		(next q0_1_14 q0_1_15 q1)

		(= (at_x q0_1_15) 851.56 )
		(= (at_y q0_1_15) 50.82 )
		(= (at_time q0_1_15) 3.00 )
		(next q0_1_15 q0_1_16 q1)

		(= (at_x q0_1_16) 857.45 )
		(= (at_y q0_1_16) 50.93 )
		(= (at_time q0_1_16) 3.20 )
		(next q0_1_16 q0_1_17 q1)

		(= (at_x q0_1_17) 863.34 )
		(= (at_y q0_1_17) 51.05 )
		(= (at_time q0_1_17) 3.40 )
		(next q0_1_17 q0_1_18 q1)

		(= (at_x q0_1_18) 869.25 )
		(= (at_y q0_1_18) 51.16 )
		(= (at_time q0_1_18) 3.60 )
		(next q0_1_18 q0_1_19 q1)

		(= (at_x q0_1_19) 875.17 )
		(= (at_y q0_1_19) 51.26 )
		(= (at_time q0_1_19) 3.80 )
		(next q0_1_19 q0_1_20 q1)

		(= (at_x q0_1_20) 881.10 )
		(= (at_y q0_1_20) 51.35 )
		(= (at_time q0_1_20) 4.00 )
		(next q0_1_20 q0_1_21 q1)

		(= (at_x q0_1_21) 887.04 )
		(= (at_y q0_1_21) 51.44 )
		(= (at_time q0_1_21) 4.20 )
		(next q0_1_21 q0_1_22 q1)

		(= (at_x q0_1_22) 892.99 )
		(= (at_y q0_1_22) 51.52 )
		(= (at_time q0_1_22) 4.40 )
		(next q0_1_22 q0_1_23 q1)

		(= (at_x q0_1_23) 898.95 )
		(= (at_y q0_1_23) 51.59 )
		(= (at_time q0_1_23) 4.60 )
		(next q0_1_23 q0_1_24 q1)

		(= (at_x q0_1_24) 904.92 )
		(= (at_y q0_1_24) 51.64 )
		(= (at_time q0_1_24) 4.80 )
		(next q0_1_24 q0_1_25 q1)

		(= (at_x q0_1_25) 910.90 )
		(= (at_y q0_1_25) 51.68 )
		(= (at_time q0_1_25) 5.00 )
		(next q0_1_25 q0_1_26 q1)

		(= (at_x q0_1_26) 916.89 )
		(= (at_y q0_1_26) 51.71 )
		(= (at_time q0_1_26) 5.20 )
		(next q0_1_26 q0_1_27 q1)

		(= (at_x q0_1_27) 922.88 )
		(= (at_y q0_1_27) 51.73 )
		(= (at_time q0_1_27) 5.40 )
		(next q0_1_27 q0_1_28 q1)

		(= (at_x q0_1_28) 928.87 )
		(= (at_y q0_1_28) 51.74 )
		(= (at_time q0_1_28) 5.60 )
		(next q0_1_28 q0_1_29 q1)

		(= (at_x q0_1_29) 934.87 )
		(= (at_y q0_1_29) 51.75 )
		(= (at_time q0_1_29) 5.80 )
		(next q0_1_29 q0_1_30 q1)

		(= (at_x q0_1_30) 940.87 )
		(= (at_y q0_1_30) 51.75 )
		(= (at_time q0_1_30) 6.00 )
		(next q0_1_30 q1 q1)

		(= (time_of_traj q0 q1 ) 6.20)


		(= (at_x q0) 764.45 )
		(= (at_y q0) 50.01 )
		(= (at_time q0) 0.00 )

		(= (at_x q1) 940.87 )
		(= (at_y q1) 51.75 )
		(= (at_time q1) 200.00 )



		(= (obst_at_x obs0) 764.43)
		(= (obst_at_y obs0) 48.25)
		(= (obst_at_speed obs0) 27.15)
		(= (obst_at_x obs1) 903.97)
		(= (obst_at_y obs1) 51.75)
		(= (obst_at_speed obs1) 41.94)
		(= (obst_at_x obs2) 1747.44)
		(= (obst_at_y obs2) 51.75)
		(= (obst_at_speed obs2) 103.15)
		(= (obst_at_x obs3) 868.39)
		(= (obst_at_y obs3) 51.75)
		(= (obst_at_speed obs3) 35.82)
		(= (obst_at_x obs4) 1416.09)
		(= (obst_at_y obs4) 51.75)
		(= (obst_at_speed obs4) 84.65)
		(= (obst_at_x obs5) 735.14)
		(= (obst_at_y obs5) 55.25)
		(= (obst_at_speed obs5) 30.20)
	)
	(:goal (and (on_left_lane)  ))
	(:metric minimize (curr_time))
)
