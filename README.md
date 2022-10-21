# PADM Final Project - Grant Appel and Mason Peterson

This README contains the logic and flow of the code in the repo

## Assumptions for our PDDL Domain

We first assumed that there were only three objects that we needed to consider - the sugar, the spam, and the drawer handles. The gripper was not thought of as a an object because we assumed that the location of the gripper would be known at all times and we would set certain predicates true only when the object was in the gripper. Additionally, we assume that the gripper can only grab one of the objects at a time; for example, if the gripper is holding the sugar, it is not able to grab the spam until the sugar has been release. 

