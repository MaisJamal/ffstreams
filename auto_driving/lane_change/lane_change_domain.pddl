(define (domain auto-driving)
(:requirements :typing :existential-preconditions :universal-preconditions :conditional-effects :disjunctive-preconditions :fluents :equality :negative-preconditions )
; requirements to use with processes and events:    :time  :timed-initial-literals ;:conditional-effects
(:types 
    conf obstacles)

(:predicates  
        (next ?q1 ?q2 ?q_end - conf) (traj ?q1 ?q2 - conf) (idle)(is_first ?q - conf ?o - obstacles) (is_last ?q - conf ?o - obstacles)(ego_at ?q - conf)
        (checking_traj ?q1 ?q2 - conf ?o - obstacles)(checked_traj ?q1 ?q2 - conf ?o - obstacles) (finished)
        (left_traj ?q1 ?q2 - conf) (on_left_lane) (right_traj ?q1 ?q2 - conf)(on_right_lane))
(:functions (curr_time) (time_of_traj ?q1 ?q2 - conf)(at_x ?q - conf)(at_y ?q - conf) (at_time ?q - conf)
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
                    (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 6.5)
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
                    (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 6.5) 
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
                     (>= (- (at_y ?after_first) (obst_at_y ?o) ) 3.5)  
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
                     (< (- (at_y ?after_first) (obst_at_y ?o) ) 3.5)  
                     (< (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 6.5)
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
                     (< (- (at_y ?after_first) (obst_at_y ?o) ) 3.5)  
                     (>= (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 6.5)
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
                     (>= (- (obst_at_y ?o) (at_y ?after_first) ) 3.5)   
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
                     (< (- (obst_at_y ?o) (at_y ?after_first) ) 3.5)
                     (>= (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (- (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) )   ) 6.5)
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
                     (< (- (obst_at_y ?o) (at_y ?after_first) ) 3.5)
                     (< (at_x ?after_first) (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time) )) ) )
                     (>= (-  (+ (obst_at_x ?o) (* (obst_at_speed ?o) (+ (at_time ?after_first) (curr_time))) ) (at_x ?after_first)  ) 6.5)
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

(:action move_left
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (left_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_right_lane) 
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)
                 (not (ego_at ?q1))
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (finished) (on_left_lane) (not (on_right_lane) )
                )
)


(:action move_right
    :parameters (?q1 ?q2 - conf)
    :precondition (and 
                 (ego_at ?q1)
                 (traj ?q1 ?q2)
                 (right_traj ?q1 ?q2)
                 (forall (?o - obstacles)
                    (checked_traj ?q1 ?q2 ?o)
                 )
                 (on_left_lane) 
                 (idle)
               )
    :effect (and 
                 (ego_at ?q2)
                 (not (ego_at ?q1))
                 (increase (curr_time)  (time_of_traj ?q1 ?q2))
                 (finished) (on_right_lane) (not (on_left_lane) )
                )
)



)
