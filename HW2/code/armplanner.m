function[armplan] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
fprintf(1, 'PlanID\n', planner_id);
[armplan, armplanlength] = planner(envmap, armstart, armgoal, planner_id);

