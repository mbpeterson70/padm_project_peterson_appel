(define
	(domain kitchen)
	
	(:requirements :strips :typing)
	
	(:types
		item
		handle
		location
		drawer
		gripper
	)
	
	(:predicates
		(at-location ?o ?l)
		(location-empty ?l)

		(base-at-counter)
		
		(in-gripper ?o)
		(gripper-empty)
		
		(drawer-open)
	)
	
	(:action grab-item
		:parameters (?g - gripper ?i - item ?l - location)
		:precondition (and (gripper-empty) (at-location ?i ?l) (at-location ?g ?l) (base-at-counter))
		:effect (and (in-gripper ?i) (not(gripper-empty)))
	)

	(:action grab-handle
		:parameters (?g - gripper ?h - handle ?l - location)
		:precondition (and (gripper-empty) (at-location ?h ?l) (at-location ?g ?l) (base-at-counter))
		:effect (and (in-gripper ?h) (not(gripper-empty)))
	)
	
	(:action release-object
		:parameters (?g - gripper ?l ?o)
		:precondition (and (in-gripper ?o) (at-location ?o ?l) (at-location ?g ?l) (base-at-counter))
		:effect (and (gripper-empty) (not(in-gripper ?o)))
	)
	
	(:action move-gripper
		:parameters (?g - gripper ?l1 - location ?l2 - location)
		:precondition (and (gripper-empty) (at-location ?g ?l1) (base-at-counter))
		:effect (and (not(at-location ?g ?l1)) (at-location ?g ?l2))
	)

	(:action move-gripper-from-drawer
		:parameters (?g - gripper ?d - drawer ?l - location)
		:precondition (and (gripper-empty) (at-location ?g ?d) (base-at-counter))
		:effect (and (not(at-location ?g ?d)) (at-location ?g ?l))
	)

	(:action move-item
		:parameters (?g - gripper ?i - item ?l1 - location ?l2 - location)
		:precondition (and (not(gripper-empty)) (at-location ?g ?l1) (at-location ?i ?l1) (in-gripper ?i) (location-empty ?l2) (base-at-counter))
		:effect (and (not(at-location ?g ?l1)) (not(at-location ?i ?l1)) (at-location ?g ?l2) (at-location ?i ?l2) (location-empty ?l1) (not (location-empty ?l2)))
	)

	(:action move-item-to-drawer
		:parameters (?g - gripper ?i - item ?l - location ?d - drawer)
		:precondition (and (not(gripper-empty)) (at-location ?g ?l) (at-location ?i ?l) (in-gripper ?i) (drawer-open) (location-empty ?d) (base-at-counter))
		:effect (and (not(at-location ?g ?l)) (not(at-location ?i ?l)) (at-location ?g ?d) (at-location ?i ?d) (location-empty ?l) (not (location-empty ?d)))
	)

	(:action move-item-from-drawer
		:parameters (?g - gripper ?i - item ?d - drawer ?l - location)
		:precondition (and (not(gripper-empty)) (at-location ?g ?d) (at-location ?i ?d) (in-gripper ?i) (location-empty ?l) (drawer-open) (base-at-counter))
		:effect (and (not(at-location ?g ?d)) (not(at-location ?i ?d)) (at-location ?g ?l) (at-location ?i ?l) (location-empty ?d) (not (location-empty ?l)))
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
)
	
		
	
