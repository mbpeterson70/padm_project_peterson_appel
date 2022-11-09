from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.action import Action

# Parser
class ActivityPlanner():

    def __init__(self, domain, problem):
        parser = PDDL_Parser()
        
        parser.parse_domain(domain)
        parser.parse_problem(problem)
        # Parsed data

        self.init_state = parser.state
        self.goal_pos = parser.positive_goals
        self.goal_neg = parser.negative_goals

        # Do nothing
        if self.applicable(self.init_state, self.goal_pos, self.goal_neg):
            return []
        # Remove typing through grounding process
        self.ground_actions = []
        for action in parser.actions:
            for act in action.groundify(parser.objects, parser.types):
                self.ground_actions.append(act)

        print(self.init_state)
        # print(goal_pos)
        # print(goal_neg)
    
    def A_star_solver(self, heuristic='relaxed_moves', consistent=True):

        def h(state):
            if heuristic=='BFS':
                return len(self.BFS_solver(root_state=state))
            elif heuristic=='relaxed_moves':
                return self.relaxed_move_heuristic(root_state=state)

        q = [(h(self.init_state), self.init_state, [])]
        expanded=set()
        while q:
            _, search_state, action_list = q.pop(0)
            if consistent and search_state in expanded:
                continue
            expanded.add(search_state)
            if self.applicable(search_state, self.goal_pos, self.goal_neg):
                return action_list
            for act in self.ground_actions:
                if self.applicable(
                    search_state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(search_state, act.add_effects, act.del_effects)
                    if consistent and new_state in expanded:
                        continue
                    f = len(action_list) + 1 + h(new_state)
                    inserted = False
                    for i in range(len(q)):
                        if f < q[i][0]:
                            q.insert(i, (f, new_state, action_list + [act]))
                            inserted = True
                            break
                    if not inserted:
                        q.append((f, new_state, action_list + [act]))
        return []

    def BFS_solver(self, root_state=None):
        if root_state == None:
            root_state = self.init_state
        q = [(root_state, [])]
        visited = {root_state}
        while q:
            search_state, action_list = q.pop(0)
            if self.applicable(search_state, self.goal_pos, self.goal_neg):
                return action_list
            for act in self.ground_actions:
                if self.applicable(
                    search_state, act.positive_preconditions, act.negative_preconditions):
                    new_state = self.apply(search_state, act.add_effects, act.del_effects)
                    if new_state not in visited:
                        q.append((new_state, action_list + [act]))
                        visited.add(new_state)
        return []

    def relaxed_move_heuristic(self, root_state=None):
        if root_state == None:
            root_state = self.init_state

        relaxed_state = root_state
        pos_relaxed_state = root_state
        neg_relaxed_state = root_state

        action = 0
        
        while  (not self.goal_pos.issubset(pos_relaxed_state)) and (not self.goal_neg.issubset(neg_relaxed_state)):
            
            for act in self.ground_actions:
                if self.applicable(relaxed_state, act.positive_preconditions, act.negative_preconditions):
                    relaxed_state = relaxed_state.union(act.add_effects)
                    relaxed_state = relaxed_state.union(act.del_effects)
                    pos_relaxed_state = pos_relaxed_state.union(act.add_effects)
                    neg_relaxed_state = neg_relaxed_state.union(act.del_effects)
            
            action += 1

        return action




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
    domain = sys.argv[1]
    problem = sys.argv[2]
    p = ActivityPlanner(domain, problem)
    solution = p.BFS_solver()
    for act in solution:
        print(f'{act.name} {act.parameters}')

    print('\nTiming')
    total_time = 0
    for i in range(10):
        start_time = time.time()
        solution = p.BFS_solver()
        end_time = time.time()
        total_time += end_time-start_time
    print(f'BFS: {total_time/10}')
    total_time = 0
    for i in range(10):
        start_time = time.time()
        solution = p.A_star_solver(heuristic='relaxed_moves', consistent=True)
        end_time = time.time()
        total_time += end_time-start_time
    for act in solution:
        print(f'{act.name} {act.parameters}')
    print(f'A-star w/ expanded list (relaxed move heuristic): {total_time/10}')
    total_time = 0
    for i in range(10):
        start_time = time.time()
        solution = p.A_star_solver(heuristic='BFS', consistent=False)
        end_time = time.time()
        total_time += end_time-start_time
    print(f'A-star w/0 expanded list (BFS heuristic): {total_time/10}')
