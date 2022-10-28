import sys
from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.action import Action

# Parser
class ActivityPlanner():

    def __init__(self, domain, problem):
        parser = PDDL_Parser()
        
        parser.parse_domain(domain)
        parser.parse_problem(problem)
        # Parsed data

        state = parser.state
        goal_pos = parser.positive_goals
        goal_neg = parser.negative_goals

        # Do nothing
        # if self.applicable(state, goal_pos, goal_neg):
        #     return []
        # Remove typing through grounding process
        ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                ground_actions.append(act)

        # print(state)
        # print(goal_pos)
        # print(goal_neg)

    # -----------------------------------------------
    # Applicable
    # -----------------------------------------------

    def applicable(self, state, positive, negative):
        return positive.issubset(state) and negative.isdisjoint(state)

    # -----------------------------------------------
    # Apply
    # -----------------------------------------------

    def apply(self, state, positive, negative):
        return state.difference(negative).union(positive)

if __name__ == '__main__':
    import sys, time
    start_time = time.time()
    domain = sys.argv[1]
    problem = sys.argv[2]
    p = ActivityPlanner(domain, problem)
