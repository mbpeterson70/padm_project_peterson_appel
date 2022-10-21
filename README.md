# PADM Final Project - Grant Appel and Mason Peterson

This README contains the logic and flow of the code in the repo

## Assumptions for our PDDL Domain

We first assumed that there were only three objects that we needed to consider - the sugar, the spam, and the drawer handles. The gripper was not thought of as a an object because we assumed that the location of the gripper would be known at all times and the gripper itself would not be moved around by anything and would act as the vehicle for the other objects to move. Additionally, we assume that the gripper can only grab one of the objects at a time; for example, if the gripper is holding the sugar, it is not able to grab the spam until the sugar has been release. The location aspect of the domain was covered by three different predicates:

on-burner ?i, in-drawer ?i, and on-counter ?i

where the ?i denotes an item type object (spam or sugar). Each of these is set to true if the item is at that location and set to false once the gripper moves them. The gripper location was covered by setting an additoinal predicate:

gripper-at-object ?o

for this predicate ?o denotes any object (the sugar, spam and drawer handle). This predicate is set true once an action is chosen for the gripper to move to an object. Alternatively, this predicate is set false after the object has been released and an action is called for the gripper to move away from the object. This means that after each object is moved, the gripper will "reset" at a default location, which will add a small amount of time to the execution, but this was determined to be acceptable. 

## PDDL File and Problem Formulation

The actual PDDL domain and all corresponding objects, predicates and actions can be found in the [sugar_spam_pddl.pddl](https://github.com/mbpeterson70/padm_project_peterson_appel/blob/main/src/sugar_spam_pddl.pddl) file and the problem statement can be found in the [p1.pddl](https://github.com/mbpeterson70/padm_project_peterson_appel/blob/main/src/p1.pddl) file. 

### Initial State

The initial state was given in our problem statement. This includes:

1. Sugar on the burner (on-burner su)
2. Spam on the counter (on-counter sp)
3. The gripper away from the counter and empty (gripper-empty)

### Goal State

The final state with respect to our problem formulations is:

1. Sugar on the counter (on-counter su)
2. Spam in the drawer (in-drawer sp)
3. The gripper is empty (gripper-empty)
4. The gripper robot is still at the counter (base-at-counter)
5. The gripper has "reset" to its default locaiton (gripper-away-from-objects)
6. The drawer is closed (not(drawer-open))

### Check

Our first step in checking that our pddle domain and problem statement were correct was getting them to work correctly with the provided pddl parser. This was accomplished and we were able to get a list of actions from the initial state to the goal state.



