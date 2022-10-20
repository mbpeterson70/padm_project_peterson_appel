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
		:precondition (and (gripper-empty) (gripper-at-object ?o))
		:effect (and (in-gripper ?o) (not(gripper-empty))
	)
	
	(:action release-object
		:parameters (?o - object)
		:precondition (and (in-gripper ?o) (gripper-at-object ?o))
		:effect (and (gripper-empty) (not(in-gripper ?o)))
	)
	
	(:action move-to-object
		:parameters (?o - object)
		:precondition 
			(forall(?o - object)
				(not(gripper-at-object ?o)
			)
		:effect (gripper-at-object ?o)
	)
	
	(:action move-away-from-object
		:parameters (?o - object)
		:precondition (and (gripper-at-object ?o) (gripper-empty))
		:effect (not(gripper-at-object ?o))
	)
		
	(:action move-object-to-burner
		:parameters (?i - item)
		:preconditions (and (in-gripper ?i) (not(on-burner ?i))
		:effects (and (on-burner ?i) (not(in-drawer ?i)) (not(on-counter ?i)))
	)
		
	(:action move-object-to-counter
		:parameters (?i - item)
		:preconditions (and (in-gripper ?i) (not(on-counter ?i))
		:effects (and (on-counter ?i) (not(in-drawer ?i)) (not(on-burner ?i)))
	)
		
	(:action move-object-to-drawer
		:parameters (?i - item)
		:preconditions (and (in-gripper ?i) (drawer-open) (not(in-drawer ?i))
		:effects (and (in-drawer ?i) (not(on-burner ?i)) (not(on-counter ?i)))
	)
	
	(:action open-drawer
		:parameters (?h - handle)
		:preconditions (and (in-gripper ?h) (not(drawer-open))
		:effects (drawer-open)
	)
	
	(:action close-drawer
		:parameters (?h - handle)
		:preconditions (and (in-gripper ?h) (drawer-open))
		:effects (not(drawer-open))
	)
	
		
		
	
	
		
	
