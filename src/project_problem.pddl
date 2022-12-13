(define (problem p1) (:domain kitchen)
(:objects 
    su - item
    sp - item
    h - handle
    g - gripper
    d - drawer
    on-counter - location
    on-burner - location
    away-from-objects - location
    at-handle - location
)

(:init
    (at-location su on-burner)
    (at-location sp on-counter)
    (at-location g away-from-objects)
    (at-location h at-handle)
    (location-empty d)
    (gripper-empty)
)

(:goal (and
    (at-location sp d)
    (at-location su on-counter)
    (at-location g away-from-objects)
    (gripper-empty)
    (base-at-counter)
    (not (drawer-open))
))

)
