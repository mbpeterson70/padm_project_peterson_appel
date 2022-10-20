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
)

(:goal (and
    (in-drawer sp)
    (on-counter su)
    (gripper-empty)
    (base-at-counter)
    (forall(o? object) (not(gripper-at-object ?o)))
    (not (drawer-open))
))

)
