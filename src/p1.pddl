(define (problem p1) (:domain kitchen)
(:objects 
    su - sugar
    sp - spam
    h - handle
)

(:init
    (on-burner su)
    (on-counter sp)
    (gripper-empty)
    (gripper-away-from-objects)
)

(:goal (and
    (in-drawer sp)
    (on-counter su)
    (gripper-empty)
    (base-at-counter)
    (gripper-away-from-objects)
    (not (drawer-open))
))

)
