(define (problem driving01)
	(:domain auto-driving)
	(:objects
		q0 q1 q0_1_1 q0_1_2 q0_1_3 q0_1_4 q0_1_5 q0_1_6 q0_1_7 q0_1_8 q0_1_9 q0_1_10 q0_1_11 q0_1_12 q0_1_13 q0_1_14 q0_1_15 q0_1_16 q0_1_17 q0_1_18 q0_1_19  - conf
		obs0 obs1 obs2 obs3 obs4 obs5 obs6 obs7 obs8 obs9 obs10  - obstacles
	)
	(:init
		(ego_at q0)
		(= (curr_time) 0)
		(idle)

		(on_right_lane)

		(traj q0 q1)

		(left_traj q0 q1)

		(next q0 q0_1_1 q1)
		(= (at_x q0_1_1) 406.98 )
		(= (at_y q0_1_1) 51.74 )
		(= (at_time q0_1_1) 0.20 )
		(next q0_1_1 q0_1_2 q1)

		(= (at_x q0_1_2) 412.96 )
		(= (at_y q0_1_2) 51.76 )
		(= (at_time q0_1_2) 0.40 )
		(next q0_1_2 q0_1_3 q1)

		(= (at_x q0_1_3) 418.95 )
		(= (at_y q0_1_3) 51.77 )
		(= (at_time q0_1_3) 0.60 )
		(next q0_1_3 q0_1_4 q1)

		(= (at_x q0_1_4) 424.94 )
		(= (at_y q0_1_4) 51.78 )
		(= (at_time q0_1_4) 0.80 )
		(next q0_1_4 q0_1_5 q1)

		(= (at_x q0_1_5) 430.93 )
		(= (at_y q0_1_5) 51.78 )
		(= (at_time q0_1_5) 1.00 )
		(next q0_1_5 q0_1_6 q1)

		(= (at_x q0_1_6) 436.92 )
		(= (at_y q0_1_6) 51.78 )
		(= (at_time q0_1_6) 1.20 )
		(next q0_1_6 q0_1_7 q1)

		(= (at_x q0_1_7) 442.92 )
		(= (at_y q0_1_7) 51.78 )
		(= (at_time q0_1_7) 1.40 )
		(next q0_1_7 q0_1_8 q1)

		(= (at_x q0_1_8) 448.91 )
		(= (at_y q0_1_8) 51.78 )
		(= (at_time q0_1_8) 1.60 )
		(next q0_1_8 q0_1_9 q1)

		(= (at_x q0_1_9) 454.91 )
		(= (at_y q0_1_9) 51.77 )
		(= (at_time q0_1_9) 1.80 )
		(next q0_1_9 q0_1_10 q1)

		(= (at_x q0_1_10) 460.91 )
		(= (at_y q0_1_10) 51.77 )
		(= (at_time q0_1_10) 2.00 )
		(next q0_1_10 q0_1_11 q1)

		(= (at_x q0_1_11) 466.91 )
		(= (at_y q0_1_11) 51.77 )
		(= (at_time q0_1_11) 2.20 )
		(next q0_1_11 q0_1_12 q1)

		(= (at_x q0_1_12) 472.91 )
		(= (at_y q0_1_12) 51.76 )
		(= (at_time q0_1_12) 2.40 )
		(next q0_1_12 q0_1_13 q1)

		(= (at_x q0_1_13) 478.91 )
		(= (at_y q0_1_13) 51.76 )
		(= (at_time q0_1_13) 2.60 )
		(next q0_1_13 q0_1_14 q1)

		(= (at_x q0_1_14) 484.91 )
		(= (at_y q0_1_14) 51.76 )
		(= (at_time q0_1_14) 2.80 )
		(next q0_1_14 q0_1_15 q1)

		(= (at_x q0_1_15) 490.91 )
		(= (at_y q0_1_15) 51.75 )
		(= (at_time q0_1_15) 3.00 )
		(next q0_1_15 q0_1_16 q1)

		(= (at_x q0_1_16) 496.91 )
		(= (at_y q0_1_16) 51.75 )
		(= (at_time q0_1_16) 3.20 )
		(next q0_1_16 q0_1_17 q1)

		(= (at_x q0_1_17) 502.91 )
		(= (at_y q0_1_17) 51.75 )
		(= (at_time q0_1_17) 3.40 )
		(next q0_1_17 q0_1_18 q1)

		(= (at_x q0_1_18) 508.91 )
		(= (at_y q0_1_18) 51.75 )
		(= (at_time q0_1_18) 3.60 )
		(next q0_1_18 q0_1_19 q1)

		(= (at_x q0_1_19) 514.91 )
		(= (at_y q0_1_19) 51.75 )
		(= (at_time q0_1_19) 3.80 )
		(next q0_1_19 q1 q1)

		(= (time_of_traj q0 q1 ) 4.00)


		(= (at_x q0) 401.00 )
		(= (at_y q0) 51.72 )
		(= (at_time q0) 0.00 )

		(= (at_x q1) 514.91 )
		(= (at_y q1) 51.75 )
		(= (at_time q1) 200.00 )



		(= (obst_at_x obs0) 422.24)
		(= (obst_at_y obs0) 48.25)
		(= (obst_at_speed obs0) 26.91)
		(= (obst_at_x obs1) 572.90)
		(= (obst_at_y obs1) 51.75)
		(= (obst_at_speed obs1) 56.45)
		(= (obst_at_x obs2) 217.78)
		(= (obst_at_y obs2) 51.75)
		(= (obst_at_speed obs2) 11.19)
		(= (obst_at_x obs3) 651.45)
		(= (obst_at_y obs3) 51.75)
		(= (obst_at_speed obs3) 62.53)
		(= (obst_at_x obs4) 241.82)
		(= (obst_at_y obs4) 51.75)
		(= (obst_at_speed obs4) -1.57)
		(= (obst_at_x obs5) 281.35)
		(= (obst_at_y obs5) 55.25)
		(= (obst_at_speed obs5) 26.44)
		(= (obst_at_x obs6) 457.24)
		(= (obst_at_y obs6) 48.25)
		(= (obst_at_speed obs6) 26.91)
		(= (obst_at_x obs7) 450.24)
		(= (obst_at_y obs7) 48.25)
		(= (obst_at_speed obs7) 26.91)
		(= (obst_at_x obs8) 443.24)
		(= (obst_at_y obs8) 48.25)
		(= (obst_at_speed obs8) 26.91)
		(= (obst_at_x obs9) 436.24)
		(= (obst_at_y obs9) 48.25)
		(= (obst_at_speed obs9) 26.91)
		(= (obst_at_x obs10) 429.24)
		(= (obst_at_y obs10) 48.25)
		(= (obst_at_speed obs10) 26.91)
	)
	(:goal (and (on_left_lane)  ))
	(:metric minimize (curr_time))
)
