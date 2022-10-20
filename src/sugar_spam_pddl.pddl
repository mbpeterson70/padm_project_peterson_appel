(define
	(domain kitchen)
	
	(:requirements :strips :typing)
	
	(:types
		item handle - objects
		sugar spam - item
	)
	
	(:predicates
		(on-burner ?i - item)
		(in-drawer ?i - item)
		(on-counter ?i - item)
		
		(base-at-counter)
		
		(gripper-at-object ?o - object)
		(in-gripper ?o - object)
		(gripper-empty)
		
		(drawer-open)
	)
	
	(:action grab-object
		:parameters (?o - object)
		:precondition (and (gripper-empty) (gripper-at-object ?o) (base-at-counter))
		:effect (and (in-gripper ?o) (not(gripper-empty)))
	)
	
	(:action release-object
		:parameters (?o - object)
		:precondition (and (in-gripper ?o) (gripper-at-object ?o) (base-at-counter))
		:effect (and (gripper-empty) (not(in-gripper ?o)))
	)
	
	(:action move-to-object
		:parameters (?o - object)
		:precondition 
			(and (forall(?o - object)
				(not(gripper-at-object ?o)))
			(base-at-counter))
		:effect (gripper-at-object ?o)
	)
	
	(:action move-away-from-object
		:parameters (?o - object)
		:precondition (and (gripper-at-object ?o) (gripper-empty) (base-at-counter))
		:effect (not(gripper-at-object ?o))
	)
		
	(:action move-object-to-burner
		:parameters (?i - item)
		:precondition (and (in-gripper ?i) (not(on-burner ?i)) (base-at-counter))
		:effect (and (on-burner ?i) (not(in-drawer ?i)) (not(on-counter ?i)))
	)
		
	(:action move-object-to-counter
		:parameters (?i - item)
		:precondition (and (in-gripper ?i) (not(on-counter ?i)) (base-at-counter))
		:effect (and (on-counter ?i) (not(in-drawer ?i)) (not(on-burner ?i)))
	)
		
	(:action move-object-to-drawer
		:parameters (?i - item)
		:precondition (and (in-gripper ?i) (drawer-open) (not(in-drawer ?i)) (base-at-counter))
		:effect (and (in-drawer ?i) (not(on-burner ?i)) (not(on-counter ?i)))
	)
	
	(:action open-drawer
		:parameters (?h - handle)
		:precondition (and (in-gripper ?h) (not(drawer-open)) (base-at-counter))
		:effect (drawer-open)
	)	
	
	(:action close-drawer
		:parameters (?h - handle)
		:precondition (and (in-gripper ?h) (drawer-open) (base-at-counter))
		:effect (not(drawer-open))
	)

	(:action move-to-base
		:precondition (and 
			(not(base-at-counter))
		)
		:effect (and 
			(base-at-counter)
		)
	)
	
		
		
	
	
		
	
