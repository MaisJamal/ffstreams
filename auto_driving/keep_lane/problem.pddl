(define (problem driving01)
	(:domain auto-driving)
	(:objects
		q0 q1 q2 q0_1_1 q0_1_2 q0_1_3 q0_1_4  q0_1_5 q0_2_1 q0_2_2  - conf
		obs0  - obstacles
	)
	(:init
		(ego_at q0)
		(= (curr_time) 0)
		(idle)
		(= (cost) 0)
		(on_init_lane)

		(traj q0 q1)

		(yield_traj q0 q1)

		(next q0 q0_1_1 q1)
		(= (at_x q0_1_1) 5 )
		(= (at_y q0_1_1) 48.25 )
		(= (at_time q0_1_1) 0.5 )
		(next q0_1_1 q0_1_2 q1)

		(= (at_x q0_1_2) 10 )
		(= (at_y q0_1_2) 48.25 )
		(= (at_time q0_1_2) 1 )
		(next q0_1_2 q0_1_3 q1)

		(= (at_x q0_1_3) 15 )
		(= (at_y q0_1_3) 48.25 )
		(= (at_time q0_1_3) 1.5 )
		(next q0_1_3 q0_1_4 q1)

		(= (at_x q0_1_4) 20 )
		(= (at_y q0_1_4) 48.25 )
		(= (at_time q0_1_4) 2 )
		(next q0_1_4 q0_1_5 q1)


		(= (at_x q0_1_5) 25 )
		(= (at_y q0_1_5) 48.25 )
		(= (at_time q0_1_5) 2.5 )
		(next q0_1_5 q1 q1)

		(= (time_of_traj q0 q1 ) 2.5)


		(traj q0 q2)

		(keep_speed_traj q0 q2)

		(next q0 q0_2_1 q2)
		(= (at_x q0_2_1) 10 )
		(= (at_y q0_2_1) 48.25 )
		(= (at_time q0_2_1) 0.5 )
		(next q0_2_1 q0_2_2 q2)

		(= (at_x q0_2_2) 20 )
		(= (at_y q0_2_2) 48.25)
		(= (at_time q0_2_2) 1)
		(next q0_2_2 q2 q2)

		(= (time_of_traj q0 q2 ) 1)

		(= (at_x q0) 0 )
		(= (at_y q0) 48.25 )
		(= (at_time q0) 0.00 )

		(= (at_x q1) 30 )
		(= (at_y q1) 48.25 )
		(= (at_time q1) 3)

		(= (at_x q2) 30 )
		(= (at_y q2) 48.25 )
		(= (at_time q2) 1.5 )



		(= (obst_at_x obs0) 10)
		(= (obst_at_y obs0) 48.25)
		(= (obst_at_speed obs0) 10)

	)
	(:goal (and (on_init_lane)(moved_forward)  ))
	(:metric minimize (cost))
)
