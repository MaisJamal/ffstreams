(define (domain auto-driving)
(:requirements :typing :existential-preconditions :universal-preconditions :conditional-effects :disjunctive-preconditions :fluents :equality :negative-preconditions )
; requirements to use with processes and events:    :time  :timed-initial-literals ;:conditional-effects
(:types 
    conf obstacles)

(:predicates  
        (next ?q1 ?q2 ?q_end - conf) (traj ?q1 ?q2 - conf) (idle)(is_first ?q - conf ?o - obstacles) (is_last ?q - conf ?o - obstacles)(ego_at ?q - conf)
        (checking_traj ?q1 ?q2 - conf ?o - obstacles)(checked_traj ?q1 ?q2 - conf ?o - obstacles) (moved_forward)
        (there_is_front_obs)(on_init_lane)(on_second_lane)(yield_traj ?q1 ?q2)(keep_speed_traj ?q1 ?q2)
        (overtake_traj ?q1 ?q2)(left_traj ?q1 ?q2)(right_traj ?q1 ?q2))
(:functions (cost)(curr_time) (time_of_traj ?q1 ?q2 - conf)(at_x ?q - conf)(at_y ?q - conf) (at_time ?q - conf)
            (obst_at_x ?o - obstacles)(obst_at_y ?o - obstacles) (obst_at_speed ?o - obstacles)
)





(:action begin_check
    :parameters (?first_q ?last_q - conf ?o - obstacles)
    :precondition (and 
                    (idle)(ego_at ?first_q)
                    (traj ?first_q ?last_q )
                    ( not (checked_traj ?first_q ?last_q ?o ))
                    )
    :effect (and 
           (is_first ?first_q ?o)
           (is_last ?last_q ?o)
           (checking_traj ?first_q ?last_q ?o)
           (not (idle))
    )
)


(:action check_forword_same_lane_ego_front
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and (is_first ?first ?o)
                    (is_last ?last ?o)
                    (next ?first ?after_first ?last)
                    (checking_traj ?most_first ?last ?o)
                    (not (= ?first ?last)) 
                    (= (at_y ?after_first)(obst_at_y ?o)) 
                    (>= (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                    (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 7)
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )

)

(:action check_forword_same_lane_ego_behind
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and (is_first ?first ?o)
                    (is_last ?last ?o)
                    (next ?first ?after_first ?last)
                    (checking_traj ?most_first ?last ?o)
                    (not (= ?first ?last)) 
                    (= (at_y ?after_first)(obst_at_y ?o))  
                    (< (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                    (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 7) 
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)

(:action check_forword_diff_lane_upper_bigger_delta_y
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last))
                     (> (at_y ?after_first)(obst_at_y ?o))
                     (>= (- (at_y ?after_first) (obst_at_y ?o) ) 3.25)  
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)

(:action check_forword_diff_lane_upper_less_delta_y_ego_behind
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last))
                     (> (at_y ?after_first)(obst_at_y ?o))
                     (< (- (at_y ?after_first) (obst_at_y ?o) ) 3.25)  
                     (< (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 7)
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)

(:action check_forword_diff_lane_upper_less_delta_y_ego_front
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last))
                     (> (at_y ?after_first)(obst_at_y ?o))
                     (< (- (at_y ?after_first) (obst_at_y ?o) ) 3.25)  
                     (>= (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 7)
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)


(:action check_forword_diff_lane_lower_bigger_delta_y
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and  (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last)) 
                     (< (at_y ?after_first)(obst_at_y ?o)) 
                     (>= (- (obst_at_y ?o) (at_y ?after_first) ) 3.25)   
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)

(:action check_forword_diff_lane_lower_less_delta_y_ego_front
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and  (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last)) 
                     (< (at_y ?after_first)(obst_at_y ?o))  
                     (< (- (obst_at_y ?o) (at_y ?after_first) ) 3.25)
                     (>= (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 7)
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)

(:action check_forword_diff_lane_lower_less_delta_y_ego_behind
    :parameters (?most_first ?first ?after_first ?last  - conf ?o - obstacles)
    :precondition (and  (is_first ?first ?o)
                     (is_last ?last ?o)
                     (next ?first ?after_first ?last)
                     (checking_traj ?most_first ?last ?o)
                     (not (= ?first ?last)) 
                     (< (at_y ?after_first)(obst_at_y ?o))  
                     (< (- (obst_at_y ?o) (at_y ?after_first) ) 3.25)
                     (< (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 7)
                )
    :effect (and
                (not (is_first ?first ?o)) ; update is_first
                (is_first ?after_first ?o)
                 )
)


(:action end_check
   :parameters (?first_first ?element - conf ?o - obstacles)
   :precondition (and  (is_first ?element ?o)
                    (is_last ?element ?o)
                    (checking_traj ?first_first ?element ?o))
   :effect (and 
            (not (is_first ?element ?o))
            (not (is_last ?element ?o))
            (not(checking_traj ?first_first ?element ?o) )
            (checked_traj ?first_first ?element ?o )
            (idle)
           )
)

(:action keep_speed
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (keep_speed_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_init_lane)
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)
                 (not (ego_at ?q1))
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (increase (cost)  5)
                 (moved_forward)
                )
)

(:action keep_lane_yield
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (yield_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_init_lane)
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)
                 (not (ego_at ?q1))
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (increase (cost)  10)
                 (moved_forward)
                )
)

(:action left_change
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (left_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_init_lane)
                 (there_is_front_obs)
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)
                 (not (ego_at ?q1))
                 (not (on_init_lane))
                 (increase (cost)  1)
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (moved_forward) )
                )

(::action overtake
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (overtake_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_init_lane)
                 ;(there_is_front_obs)
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)(on_second_lane)
                 (not (ego_at ?q1))
                 (increase (cost)  1)
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (moved_forward) )
                )
)
