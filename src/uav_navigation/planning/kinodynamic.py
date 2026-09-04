import ompl.base as ob 
import ompl.control as oc 

def is_state_valid(state): 
    # Placeholder collision checker 
    return True 

def propagate(start, control, duration, result): 
    # UAV state: [x, y, dx, dy] control: [ax, ay] 
    x = start[0] 
    y = start[1] 
    dx = start[2] 
    dy = start[3] 
    ax = control[0] 
    ay = control[1] 
    dt = duration 

    new_x = x + dx * dt + 0.5 * ax * dt**2 
    new_y = y + dy * dt + 0.5 * ay * dt**2 
    new_dx = dx + ax * dt 
    new_dy = dy + ay * dt 

    result[0] = new_x 
    result[1] = new_y 
    result[2] = new_dx 
    result[3] = new_dy 

def plan_kinodynamic_rrt_star(start_vals, goal_vals): 
    space = ob.RealVectorStateSpace(4) # [x,y,dx,dy] 
    bounds = ob.RealVectorBounds(4) 
    bounds.setLow(0, 0.0); bounds.setHigh(0, 50.0) # x bounds 
    bounds.setLow(1, 0.0); bounds.setHigh(1, 50.0) # y bounds 
    bounds.setLow(2, -5.0); bounds.setHigh(2, 5.0) # dx bounds 
    bounds.setLow(3, -5.0); bounds.setHigh(3, 5.0) # dy bounds 
    space.setBounds(bounds) 

    control_space = oc.RealVectorControlSpace(space, 2) # ax, ay 
    control_bounds = ob.RealVectorBounds(2) 
    control_bounds.setLow(-1.0) 
    control_bounds.setHigh(1.0) 
    control_space.setBounds(control_bounds) 

    si = oc.SpaceInformation(space, control_space) 
    si.setPropagationStepSize(0.1) 
    si.setMinMaxControlDuration(1, 10) 
    si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid)) 

    def state_propagator(start, control, duration, result): 
        propagate(start, control, duration, result) 
    si.setStatePropagator(oc.StatePropagatorFn(state_propagator)) 

    ss = oc.SimpleSetup(si) 
    start = ob.State(space) 
    for i in range(4): 
        start[i] = start_vals[i] 
    goal = ob.State(space) 
    for i in range(4): 
        goal[i] = goal_vals[i] 

    ss.setStartAndGoalStates(start, goal, 0.5) 
    planner = oc.KinodynamicRRTstar(si) 
    ss.setPlanner(planner) 

    solved = ss.solve(10.0) 
    if solved: 
        print("Found solution:") 
        path = ss.getSolutionPath() 
        path.printAsMatrix() 
        return path 
    else: 
        print("No solution found.") 
        return None 

if __name__ == "__main__": 
    start = [0.0, 0.0, 0.0, 0.0] 
    goal = [40.0, 40.0, 0.0, 0.0] 
    plan_kinodynamic_rrt_star(start, goal) 
